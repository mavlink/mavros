/*
 * Copyright 2014,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointPosition plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <GeographicLib/Geocentric.hpp>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"


namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using mavlink::common::MAV_FRAME;

/**
 * @brief Setpoint position plugin
 * @plugin setpoint_position
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointPositionPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointPositionPlugin>,
  private plugin::SetPositionTargetGlobalIntMixin<SetpointPositionPlugin>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SetpointPositionPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_position")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "mav_frame", "LOCAL_NED", [&](const rclcpp::Parameter & p) {
        auto mav_frame_str = p.as_string();
        auto new_mav_frame = utils::mav_frame_from_str(mav_frame_str);

        if (new_mav_frame == MAV_FRAME::LOCAL_NED && mav_frame_str != "LOCAL_NED") {
          throw rclcpp::exceptions::InvalidParameterValueException(
            utils::format(
              "unknown MAV_FRAME: %s",
              mav_frame_str.c_str()));
        }
        mav_frame = new_mav_frame;
      });

    auto sensor_qos = rclcpp::SensorDataQoS();

    setpoint_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/local", sensor_qos, std::bind(
        &SetpointPositionPlugin::setpoint_cb, this,
        _1));
    setpointg_sub = node->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
      "~/global",
      sensor_qos, std::bind(
        &SetpointPositionPlugin::setpointg_cb, this,
        _1));
    setpointg2l_sub = node->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
      "~/global_to_local", sensor_qos,
      std::bind(&SetpointPositionPlugin::setpointg2l_cb, this, _1));

    gps_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
      "global_position/global",
      sensor_qos,
      std::bind(&SetpointPositionPlugin::gps_cb, this, _1));
    local_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "local_position/pose",
      sensor_qos,
      std::bind(&SetpointPositionPlugin::local_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointPositionPlugin>;
  friend class plugin::SetPositionTargetGlobalIntMixin<SetpointPositionPlugin>;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub;
  //! Global setpoint
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr setpointg_sub;
  //! Global setpoint converted to local setpoint
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr setpointg2l_sub;
  //! current GPS
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
  //! current local ENU
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub;

  /* Stores current gps state. */
  // sensor_msgs::NavSatFix current_gps_msg;

  //! geodetic coordinates LLA
  Eigen::Vector3d current_gps;
  //! Current local position in ENU
  Eigen::Vector3d current_local_pos;
  //! old time gps time stamp in [ms], to check if new gps msg is received
  uint32_t old_gps_stamp = 0;

  MAV_FRAME mav_frame;

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send setpoint to FCU position controller.
   *
   * @warning Send only XYZ, Yaw. ENU frame.
   */
  void send_position_target(const rclcpp::Time & stamp, const Eigen::Affine3d & tr)
  {
    using mavlink::common::MAV_FRAME;

    /* Documentation start from bit 1 instead 0;
     * Ignore velocity and accel vectors, yaw rate.
     *
     * In past versions on PX4 there been bug described in #273.
     * If you got similar issue please try update firmware first.
     */
    const uint16_t ignore_all_except_xyz_y = (1 << 11) | (7 << 6) | (7 << 3);

    auto p = [&]() {
        if (mav_frame == MAV_FRAME::BODY_NED ||
          mav_frame == MAV_FRAME::BODY_OFFSET_NED)
        {
          return ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(tr.translation()));
        } else {
          return ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
        }
      } ();

    auto q = [&]() {
        if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
          return ftf::transform_orientation_absolute_frame_aircraft_baselink(
            Eigen::Quaterniond(tr.rotation()));
        } else {
          return ftf::transform_orientation_enu_ned(
            ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));
        }
      } ();

    set_position_target_local_ned(
      get_time_boot_ms(stamp),
      utils::enum_value(mav_frame),
      ignore_all_except_xyz_y,
      p,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      ftf::quaternion_get_yaw(q), 0.0);
  }

  /* -*- callbacks -*- */

  void setpoint_cb(const geometry_msgs::msg::PoseStamped::SharedPtr req)
  {
    Eigen::Affine3d tr;
    tf2::fromMsg(req->pose, tr);

    send_position_target(req->header.stamp, tr);
  }

  /**
   * Gets setpoint position setpoint and send SET_POSITION_TARGET_GLOBAL_INT
   */
  void setpointg_cb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr req)
  {
    using mavlink::common::POSITION_TARGET_TYPEMASK;

    uint16_t type_mask = uint16_t(POSITION_TARGET_TYPEMASK::VX_IGNORE) |
      uint16_t(POSITION_TARGET_TYPEMASK::VY_IGNORE) |
      uint16_t(POSITION_TARGET_TYPEMASK::VZ_IGNORE) |
      uint16_t(POSITION_TARGET_TYPEMASK::AX_IGNORE) |
      uint16_t(POSITION_TARGET_TYPEMASK::AY_IGNORE) |
      uint16_t(POSITION_TARGET_TYPEMASK::AZ_IGNORE);

    Eigen::Quaterniond attitude = ftf::to_eigen(req->pose.orientation);
    Eigen::Quaterniond q = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(attitude));

    set_position_target_global_int(
      get_time_boot_ms(req->header.stamp),
      uint8_t(MAV_FRAME::GLOBAL_INT),
      type_mask,
      req->pose.position.latitude * 1e7,
      req->pose.position.longitude * 1e7,
      req->pose.position.altitude,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      ftf::quaternion_get_yaw(q),
      0);
  }

  /**
   * Gets gps setpoint, converts it to local ENU, and sends it to FCU
   */
  void setpointg2l_cb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr req)
  {
    /**
     * The idea is to convert the change in LLA(goal_gps-current_gps) to change in ENU
     * 1. convert current/goal gps points to current/goal ECEF points
     * 2. claculate offset in ECEF frame
     * 3. converts ECEF offset to ENU offset given current gps LLA
     * 4. adds ENU offset to current local ENU to that will be sent to FCU
     */

    GeographicLib::Geocentric earth(
      GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

    Eigen::Vector3d goal_gps(req->pose.position.latitude, req->pose.position.longitude,
      req->pose.position.altitude);

    // current gps -> curent ECEF
    Eigen::Vector3d current_ecef;
    earth.Forward(
      current_gps.x(), current_gps.y(), current_gps.z(),
      current_ecef.x(), current_ecef.y(), current_ecef.z());

    // goal gps -> goal ECEF
    Eigen::Vector3d goal_ecef;
    earth.Forward(
      goal_gps.x(), goal_gps.y(), goal_gps.z(),
      goal_ecef.x(), goal_ecef.y(), goal_ecef.z());

    // get ENU offset from ECEF offset
    Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;
    Eigen::Vector3d enu_offset = ftf::transform_frame_ecef_enu(ecef_offset, current_gps);

    // prepare yaw angle
    Eigen::Affine3d sp;                 // holds position setpoint
    Eigen::Quaterniond q;               // holds desired yaw

    tf2::fromMsg(req->pose.orientation, q);

    // set position setpoint
    sp.translation() = current_local_pos + enu_offset;
    // set desired orientation
    sp.linear() = q.toRotationMatrix();

    // Only send if current gps is updated, to avoid divergence
    if (get_time_boot_ms(req->header.stamp) > old_gps_stamp) {
      old_gps_stamp = get_time_boot_ms(req->header.stamp);
      send_position_target(req->header.stamp, sp);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "SPG: sp not sent.");
    }
  }

  /**
   * Current GPS coordinates
   */
  void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    current_gps = {msg->latitude, msg->longitude, msg->altitude};
  }

  /**
   * current local position in ENU
   */
  void local_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_local_pos = ftf::to_eigen(msg->pose.position);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointPositionPlugin)
