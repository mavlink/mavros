/**
 * @brief Landing target plugin
 * @file landing_target.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015,2017,2019 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <algorithm>
#include <string>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/utils.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/msg/landing_target.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;
using mavlink::common::MAV_FRAME;
using mavlink::common::LANDING_TARGET_TYPE;

/**
 * @brief Landing Target plugin
 * @plugin landing_target
 *
 * This plugin is intended to publish the location of a landing area captured from a downward facing camera
 * to the FCU and/or receive landing target tracking data coming from the FCU.
 */
class LandingTargetPlugin : public plugin::Plugin,
  private plugin::TF2ListenerMixin<LandingTargetPlugin>
{
public:
  explicit LandingTargetPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "landing_target"),
    tf_rate(50.0),
    tf_send(true),
    tf_listen(false),
    frame_id("landing_target_1"),
    tf_frame_id("landing_target_1"),
    target_size_x(1.0),
    target_size_y(1.0),
    fov_x(2.0071286398),
    fov_y(2.0071286398),
    focal_length(2.8),
    image_width(640),
    image_height(480),
    mav_frame("LOCAL_NED"),
    land_target_type("VISION_FIDUCIAL")
  {
    enable_node_watch_parameters();

    // general params
    node_declare_and_watch_parameter(
      "frame_id", "landing_target_1", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    node_declare_and_watch_parameter(
      "listen_lt", false, [&](const rclcpp::Parameter & p) {
        auto listen_lt = p.as_bool();

        land_target_sub.reset();

        if (listen_lt) {
          land_target_sub = node->create_subscription<mavros_msgs::msg::LandingTarget>(
            "~/raw", 10, std::bind(
              &LandingTargetPlugin::landtarget_cb, this,
              _1));
        }
      });

    node_declare_and_watch_parameter(
      "mav_frame", "LOCAL_NED", [&](const rclcpp::Parameter & p) {
        mav_frame = p.as_string();
        frame = utils::mav_frame_from_str(mav_frame);
        // MAV_FRAME index based on given frame name (If unknown, defaults to GENERIC)
      });

    node_declare_and_watch_parameter(
      "land_target_type", "VISION_FIDUCIAL", [&](const rclcpp::Parameter & p) {
        land_target_type = p.as_string();
        type = utils::landing_target_type_from_str(land_target_type);
        // LANDING_TARGET_TYPE index based on given type name (If unknown, defaults to LIGHT_BEACON)
      });

    // target size
    node_declare_and_watch_parameter(
      "target_size.x", 1.0, [&](const rclcpp::Parameter & p) {
        target_size_x = p.as_double();  // [meters]
      });

    node_declare_and_watch_parameter(
      "target_size.y", 1.0, [&](const rclcpp::Parameter & p) {
        target_size_y = p.as_double();
      });

    // image size
    node_declare_and_watch_parameter(
      "image.width", 640, [&](const rclcpp::Parameter & p) {
        image_width = p.as_int();       // [pixels]
      });

    node_declare_and_watch_parameter(
      "image.height", 480, [&](const rclcpp::Parameter & p) {
        image_height = p.as_int();
      });

    // camera field-of-view -> should be precised using the calibrated camera intrinsics
    node_declare_and_watch_parameter(
      "camera.fov_x", 2.0071286398, [&](const rclcpp::Parameter & p) {
        fov_x = p.as_double();          // default: 115 degrees in [radians]
      });

    node_declare_and_watch_parameter(
      "camera.fov_y", 2.0071286398, [&](const rclcpp::Parameter & p) {
        fov_y = p.as_double();          // default: 115 degrees in [radians]
      });

    // camera focal length
    node_declare_and_watch_parameter(
      "camera.focal_length", 2.8, [&](const rclcpp::Parameter & p) {
        focal_length = p.as_double();   // ex: OpenMV Cam M7: 2.8 [mm]
      });

    // tf subsection
    node_declare_and_watch_parameter(
      "tf.rate_limit", 50.0, [&](const rclcpp::Parameter & p) {
        // no dynamic update here yet. need to modify the thread in
        // setpoint_mixin to handle new rates
        tf_rate = p.as_double();
      });

    node_declare_and_watch_parameter(
      "tf.send", true, [&](const rclcpp::Parameter & p) {
        tf_send = p.as_bool();
      });

    node_declare_and_watch_parameter(
      "tf.frame_id", frame_id, [&](const rclcpp::Parameter & p) {
        tf_frame_id = p.as_string();
      });

    node_declare_and_watch_parameter(
      "tf.child_frame_id", "camera_center", [&](const rclcpp::Parameter & p) {
        tf_child_frame_id = p.as_string();
      });

    node_declare_and_watch_parameter(
      "tf.listen", false, [&](const rclcpp::Parameter & p) {
        tf_listen = p.as_bool();
        if (!tf_listen) {
          return;
        }

        RCLCPP_INFO_STREAM(
          get_logger(),
          "LT: Listen to landing_target transform " << tf_frame_id <<
            " -> " << tf_child_frame_id);
        tf2_start("LandingTargetTF", &LandingTargetPlugin::transform_cb);
      });


    auto sensor_qos = rclcpp::SensorDataQoS();

    land_target_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose_in", sensor_qos);
    lt_marker_pub = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "~/lt_marker",
      sensor_qos);

    pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/pose", 10, std::bind(
        &LandingTargetPlugin::pose_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&LandingTargetPlugin::handle_landing_target)
    };
  }

private:
  friend class TF2ListenerMixin;

  double tf_rate;
  bool tf_send;
  bool tf_listen;
  rclcpp::Time last_transform_stamp;

  std::string frame_id;
  std::string tf_frame_id;
  std::string tf_child_frame_id;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr land_target_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr lt_marker_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  rclcpp::Subscription<mavros_msgs::msg::LandingTarget>::SharedPtr land_target_sub;

  double target_size_x, target_size_y;
  double fov_x, fov_y;
  double focal_length;
  int image_width, image_height;

  MAV_FRAME frame;
  std::string mav_frame;

  LANDING_TARGET_TYPE type;
  std::string land_target_type;

  /* -*- low-level send -*- */
  void landing_target(
    uint64_t time_usec,
    uint8_t target_num,
    uint8_t frame,
    Eigen::Vector2f angle,
    float distance,
    Eigen::Vector2f size,
    Eigen::Vector3d pos,
    Eigen::Quaterniond q,
    uint8_t type,
    uint8_t position_valid)
  {
    mavlink::common::msg::LANDING_TARGET lt {};

    lt.time_usec = time_usec;
    lt.target_num = target_num;
    lt.frame = frame;
    lt.distance = distance;
    lt.type = type;
    lt.position_valid = position_valid;
    lt.angle_x = angle.x();
    lt.angle_y = angle.y();
    lt.size_x = size.x();
    lt.size_y = size.y();
    lt.x = pos.x();
    lt.y = pos.y();
    lt.z = pos.z();

    ftf::quaternion_to_mavlink(q, lt.q);

    uas->send_message(lt);
  }

  /* -*- mid-level helpers -*- */
  /**
   * @brief Displacement: (not to be mixed with angular displacement)
   *
   * WITH angle_rad = atan(y / x) * (π / 180)
   * IF X & Y > 0: (1st quadrant)
   *      θ_x = angle_rad
   *      θ_y = - angle_rad
   * IF X < 0 & Y > 0: (2nd quadrant)
   *      θ_x = π - angle_rad
   *      θ_y = angle_rad
   * IF X < 0 & Y < 0: (3rd quadrant)
   *      θ_x = π + angle_rad
   *      θ_y = π - angle_rad
   * IF X > 0 & Y < 0: (4th quadrant)
   *      θ_x = - angle_rad
   *      θ_y = π + angle_rad
   */
  void inline cartesian_to_displacement(const Eigen::Vector3d & pos, Eigen::Vector2f & angle)
  {
    float angle_rad = atan(pos.y() / pos.x()) * (M_PI / 180.0);

    if (pos.x() > 0 && pos.y() > 0) {
      angle.x() = angle_rad;
      angle.y() = -angle_rad;
    } else if (pos.x() < 0 && pos.y() > 0) {
      angle.x() = M_PI - angle_rad;
      angle.y() = angle_rad;
    } else if (pos.x() < 0 && pos.y() < 0) {
      angle.x() = M_PI + angle_rad;
      angle.y() = M_PI - angle_rad;
    } else if (pos.x() > 0 && pos.y() < 0) {
      angle.x() = -angle_rad;
      angle.y() = M_PI + angle_rad;
    }
  }

  /**
   * @brief Send landing target transform to FCU
   */
  void send_landing_target(const rclcpp::Time & stamp, const Eigen::Affine3d & tr)
  {
    /**
     * @brief the position of the landing target WRT camera center - on the FCU,
     * the position WRT to the origin local NED frame can be computed to allow
     * the FCU to know where the landing target is in the local frame.
     */
    auto pos = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));

    /** @brief the orientation of the landing target WRT camera frame */
    auto q = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

    Eigen::Vector2f angle;
    Eigen::Vector2f size_rad;
    Eigen::Vector2f fov;

    // the norm of the position vector is considered the distance to the landing target
    float distance = pos.norm();

    // if the landing target type is a vision type, compute the angular offsets
    if (land_target_type.find("VISION")) {
      /**
       * @brief: the camera angular offsets can be computed by knowing the position
       * of the target center relative to the camera center, the field-of-view of
       * the camera and the image resolution being considered.
       * The target size is computed by the angle of view formula (similar to angular diameter).
       */
      angle.x() = (pos.x() - image_width / 2.0) * fov.x() / image_width;
      angle.y() = (pos.y() - image_height / 2.0) * fov.y() / image_height;
      /**
       * @brief Angular diameter:
       * δ = 2 * atan(d / (2 * D))
       * where,	d = actual diameter; D = distance to the object (or focal length of a camera)
       */
      size_rad = {2 * (M_PI / 180.0) * atan(target_size_x / (2 * focal_length)),
        2 * (M_PI / 180.0) * atan(target_size_y / (2 * focal_length))};
    } else {
      // else, the same values are computed considering the displacement
      // relative to X and Y axes of the camera frame reference
      cartesian_to_displacement(pos, angle);
      size_rad = {2 * (M_PI / 180.0) * atan(target_size_x / (2 * distance)),
        2 * (M_PI / 180.0) * atan(target_size_y / (2 * distance))};
    }

    if (last_transform_stamp == stamp) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(), 10, "LT: Same transform as last one, dropped.");
      return;
    }
    last_transform_stamp = stamp;


    // the last char of frame_id is considered the number of the target
    uint8_t id = static_cast<uint8_t>(frame_id.back());

    landing_target(
      stamp.nanoseconds() / 1000,
      id,
      utils::enum_value(frame),         // by default, in LOCAL_NED
      angle,
      distance,
      size_rad,
      pos,
      q,
      utils::enum_value(type),
      1);                               // position is valid from the first received msg
  }

  /**
   * @brief Receive landing target from FCU.
   */
  void handle_landing_target(
    [[maybe_unused]] const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::LANDING_TARGET & land_target,
    [[maybe_unused]] plugin::filter::SystemAndOk filter)
  {
    /** @todo these transforms should be applied according to the MAV_FRAME */
    auto position =
      ftf::transform_frame_ned_enu(
      Eigen::Vector3d(
        land_target.x, land_target.y,
        land_target.z));
    auto orientation = ftf::transform_orientation_aircraft_baselink(
      ftf::transform_orientation_ned_enu(
        ftf::mavlink_to_quaternion(land_target.q)));

    // auto rpy = ftf::quaternion_to_rpy(orientation);

    RCLCPP_DEBUG_STREAM_THROTTLE(
      get_logger(),
      *get_clock(), 10,
      "landing_target:\n" <<
        land_target.to_yaml());

    geometry_msgs::msg::PoseStamped pose;
    pose.header = uas->synchronized_header(frame_id, land_target.time_usec);

    pose.pose.position = tf2::toMsg(position);
    pose.pose.orientation = tf2::toMsg(orientation);

    land_target_pub->publish(pose);

    if (tf_send) {
      geometry_msgs::msg::TransformStamped transform;

      transform.header.stamp = pose.header.stamp;
      transform.header.frame_id = "landing_target_" + std::to_string(
        land_target.target_num);
      transform.child_frame_id = tf_child_frame_id;

      transform.transform.rotation = pose.pose.orientation;
      geometry_msgs::msg::Point translation_p = tf2::toMsg(position);
      transform.transform.translation.x = translation_p.x;
      transform.transform.translation.y = translation_p.y;
      transform.transform.translation.z = translation_p.z;

      uas->tf2_broadcaster.sendTransform(transform);
    }

    geometry_msgs::msg::Vector3Stamped tg_size_msg;
    tg_size_msg.vector.x = target_size_x;
    tg_size_msg.vector.y = target_size_y;
    tg_size_msg.vector.z = 0.0;

    lt_marker_pub->publish(tg_size_msg);
  }

  /* -*- callbacks -*- */
  /**
   * @brief callback for TF2 listener
   */
  void transform_cb(const geometry_msgs::msg::TransformStamped & transform)
  {
    Eigen::Affine3d tr = tf2::transformToEigen(transform.transform);

    send_landing_target(transform.header.stamp, tr);
  }

  /**
   * @brief callback for PoseStamped msgs topic
   */
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr req)
  {
    Eigen::Affine3d tr;
    tf2::fromMsg(req->pose, tr);

    rclcpp::Time sys_time(req->header.stamp, RCL_SYSTEM_TIME);
    send_landing_target(sys_time, tr);
  }

  /**
   * @brief callback for raw LandingTarget msgs topic - useful if one has the
   * data processed in another node
   */
  void landtarget_cb(const mavros_msgs::msg::LandingTarget::SharedPtr req)
  {
    Eigen::Affine3d tr;
    tf2::fromMsg(req->pose, tr);

    /** @todo these transforms should be applied according to the MAV_FRAME */
    auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
    auto orientation = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

    landing_target(
      rclcpp::Time(req->header.stamp).nanoseconds() / 1000,
      req->target_num,
      req->frame,               // by default, in LOCAL_NED
      Eigen::Vector2f(req->angle[0], req->angle[1]),
      req->distance,
      Eigen::Vector2f(req->size[0], req->size[1]),
      position,
      orientation,
      req->type,
      1);                       // position is valid from the first received msg
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::LandingTargetPlugin)
