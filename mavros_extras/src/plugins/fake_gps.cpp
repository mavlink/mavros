/*
 * Copyright 2015 Christoph Tobler.
 * Copyright 2017 Nuno Marques.
 * Copyright 2019 Amilcar Lucas.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Fake GPS with local position source plugin
 * @file fake_gps.cpp
 * @author Christoph Tobler <toblech@ethz.ch>
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Amilcar Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include <tf2_eigen/tf2_eigen.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// the number of GPS leap seconds
#define GPS_LEAPSECONDS_MILLIS 18000ULL

#define MSEC_PER_WEEK (7ULL * 86400ULL * 1000ULL)
#define UNIX_OFFSET_MSEC (17000ULL * 86400ULL + 52ULL * 10ULL * MSEC_PER_WEEK - \
  GPS_LEAPSECONDS_MILLIS)

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT
using mavlink::common::GPS_FIX_TYPE;
using mavlink::common::GPS_INPUT_IGNORE_FLAGS;

/**
 * @brief Fake GPS plugin.
 * @plugin fake_gps
 *
 * Sends fake GPS from local position estimation source data (motion capture,
 * vision) to FCU - processed in HIL mode or out of it if parameter MAV_USEHILGPS
 * is set on PX4 Pro Autopilot Firmware; Ardupilot Firmware already supports it
 * without a flag set.
 */
class FakeGPSPlugin : public plugin::Plugin,
  private plugin::TF2ListenerMixin<FakeGPSPlugin>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FakeGPSPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "fake_gps"),
    // WGS-84 ellipsoid (a - equatorial radius, f - flattening of ellipsoid)
    earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()),
    use_mocap(true),
    use_vision(false),
    use_hil_gps(true),
    mocap_transform(true),
    mocap_withcovariance(false),
    tf_listen(false),
    eph(2.0),
    epv(2.0),
    horiz_accuracy(0.0f),
    vert_accuracy(0.0f),
    speed_accuracy(0.0f),
    gps_id(0),
    satellites_visible(5),
    fix_type(GPS_FIX_TYPE::NO_GPS),
    tf_rate(10.0),
    map_origin(0.0, 0.0, 0.0)
  {
    enable_node_watch_parameters();

    last_pos_time = rclcpp::Time(0.0);

    // general params
    node_declare_and_watch_parameter(
      "gps_id", 0, [&](const rclcpp::Parameter & p) {
        gps_id = p.as_int();
      });
    node_declare_and_watch_parameter(
      "fix_type", utils::enum_value(GPS_FIX_TYPE::NO_GPS), [&](const rclcpp::Parameter & p) {
        fix_type = static_cast<GPS_FIX_TYPE>( p.as_int());
      });
    node_declare_and_watch_parameter(
      "gps_rate", 5.0, [&](const rclcpp::Parameter & p) {
        rclcpp::Rate rate(p.as_double());

        gps_rate_period = rate.period();
      });
    node_declare_and_watch_parameter(
      "eph", 2.0, [&](const rclcpp::Parameter & p) {
        eph = p.as_double();
      });
    node_declare_and_watch_parameter(
      "epv", 2.0, [&](const rclcpp::Parameter & p) {
        epv = p.as_double();
      });
    node_declare_and_watch_parameter(
      "horiz_accuracy", 0.0, [&](const rclcpp::Parameter & p) {
        horiz_accuracy = p.as_double();
      });
    node_declare_and_watch_parameter(
      "vert_accuracy", 0.0, [&](const rclcpp::Parameter & p) {
        vert_accuracy = p.as_double();
      });
    node_declare_and_watch_parameter(
      "speed_accuracy", 0.0, [&](const rclcpp::Parameter & p) {
        speed_accuracy = p.as_double();
      });
    node_declare_and_watch_parameter(
      "satellites_visible", 5, [&](const rclcpp::Parameter & p) {
        satellites_visible = p.as_int();
      });

    // default origin/starting point: Zürich geodetic coordinates
    node_declare_and_watch_parameter(
      "geo_origin.lat", 47.3667, [&](const rclcpp::Parameter & p) {
        map_origin.x() = p.as_double();
      });
    node_declare_and_watch_parameter(
      "geo_origin.lon", 8.5500, [&](const rclcpp::Parameter & p) {
        map_origin.y() = p.as_double();
      });
    node_declare_and_watch_parameter(
      "geo_origin.alt", 408.0, [&](const rclcpp::Parameter & p) {
        map_origin.z() = p.as_double();
      });

    try {
      /**
       * @brief Conversion of the origin from geodetic coordinates (LLA)
       * to ECEF (Earth-Centered, Earth-Fixed)
       */
      earth.Forward(
        map_origin.x(), map_origin.y(), map_origin.z(),
        ecef_origin.x(), ecef_origin.y(), ecef_origin.z());
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "FGPS: Caught exception: " << e.what());
    }

    // source set params
    node_declare_and_watch_parameter(
      // listen to MoCap source
      "use_mocap", true, [&](const rclcpp::Parameter & p) {
        use_mocap = p.as_bool();
      });
    node_declare_and_watch_parameter(
      // listen to MoCap source (TransformStamped if true; PoseStamped if false)
      "mocap_transform", true, [&](const rclcpp::Parameter & p) {
        mocap_transform = p.as_bool();
      });
    node_declare_and_watch_parameter(
      // ~mocap/pose uses PoseWithCovarianceStamped Message
      "mocap_withcovariance", false, [&](const rclcpp::Parameter & p) {
        mocap_withcovariance = p.as_bool();
      });

    node_declare_and_watch_parameter(
      // listen to Vision source
      "use_vision", false, [&](const rclcpp::Parameter & p) {
        use_vision = p.as_bool();
      });
    node_declare_and_watch_parameter(
      "use_hil_gps", false, [&](const rclcpp::Parameter & p) {
        // send HIL_GPS MAVLink messages if true,
        // send GPS_INPUT mavlink messages if false
        use_hil_gps = p.as_bool();
      });

    // tf params
    node_declare_and_watch_parameter(
      "tf.frame_id", "map", [&](const rclcpp::Parameter & p) {
        tf_frame_id = p.as_string();
      });
    node_declare_and_watch_parameter(
      "tf.child_frame_id", "base_link", [&](const rclcpp::Parameter & p) {
        tf_child_frame_id = p.as_string();
      });
    node_declare_and_watch_parameter(
      "tf.rate_limit", 10.0, [&](const rclcpp::Parameter & p) {
        tf_rate = p.as_double();
      });
    node_declare_and_watch_parameter(
      "tf.listen", false, [&](const rclcpp::Parameter & p) {
        tf_listen = p.as_bool();
      });


    if (use_mocap) {
      if (mocap_transform) {                // MoCap data in TransformStamped msg
        mocap_tf_sub =
          node->create_subscription<geometry_msgs::msg::TransformStamped>(
          "~/mocap/tf", 10,
          std::bind(&FakeGPSPlugin::mocap_tf_cb, this, _1));
      } else if (mocap_withcovariance) {    // MoCap data in PoseWithCovarianceStamped msg
        mocap_pose_cov_sub =
          node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "~/mocap/pose_cov", 10, std::bind(&FakeGPSPlugin::mocap_pose_cov_cb, this, _1));
      } else {                              // MoCap data in PoseStamped msg
        mocap_pose_sub =
          node->create_subscription<geometry_msgs::msg::PoseStamped>(
          "~/mocap/pose", 10,
          std::bind(&FakeGPSPlugin::mocap_pose_cb, this, _1));
      }
    } else if (use_vision) {                // Vision data in PoseStamped msg
      vision_pose_sub =
        node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/vision", 10,
        std::bind(&FakeGPSPlugin::vision_cb, this, _1));
    } else if (tf_listen) {                 // Pose acquired from TF Listener
      RCLCPP_INFO_STREAM(
        get_logger(), "Listen to transform " << tf_frame_id <<
          " -> " << tf_child_frame_id);
      tf2_start("FakeGPSVisionTF", &FakeGPSPlugin::transform_cb);
    } else {
      RCLCPP_ERROR(get_logger(), "No pose source!");
    }
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class TF2ListenerMixin;

  std::chrono::nanoseconds gps_rate_period;
  rclcpp::Time last_pos_time;

  // Constructor for a ellipsoid
  GeographicLib::Geocentric earth;

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr mocap_tf_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mocap_pose_cov_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_sub;

  bool use_mocap;                       //!< set use of mocap data (PoseStamped msg)
  bool use_vision;                      //!< set use of vision data
  bool use_hil_gps;                     //!< set use of use_hil_gps MAVLink messages
  bool mocap_transform;                 //!< set use of mocap data (TransformStamped msg)
  bool mocap_withcovariance;            //!< ~mocap/pose uses PoseWithCovarianceStamped Message
  bool tf_listen;                       //!< set use of TF Listener data

  double eph, epv;
  float horiz_accuracy;
  float vert_accuracy;
  float speed_accuracy;
  int gps_id;
  int satellites_visible;
  GPS_FIX_TYPE fix_type;

  double tf_rate;
  std::string tf_frame_id;
  std::string tf_child_frame_id;
  rclcpp::Time last_transform_stamp;

  Eigen::Vector3d map_origin;           //!< geodetic origin [lla]
  Eigen::Vector3d ecef_origin;          //!< geocentric origin [m]
  Eigen::Vector3d old_ecef;             //!< previous geocentric position [m]
  double old_stamp;                     //!< previous stamp [s]

  /* -*- mid-level helpers and low-level send -*- */

  /**
   * @brief Send fake GPS coordinates through HIL_GPS or GPS_INPUT Mavlink msg
   */
  void send_fake_gps(const rclcpp::Time & stamp, const Eigen::Vector3d & ecef_offset)
  {
    auto now_ = node->now();

    // Throttle incoming messages
    if ((now_ - last_pos_time).to_chrono<std::chrono::nanoseconds>() < gps_rate_period) {
      return;
    }
    last_pos_time = now_;

    Eigen::Vector3d geodetic;
    Eigen::Vector3d current_ecef(ecef_origin.x() + ecef_offset.x(),
      ecef_origin.y() + ecef_offset.y(),
      ecef_origin.z() + ecef_offset.z());

    try {
      earth.Reverse(
        current_ecef.x(), current_ecef.y(), current_ecef.z(),
        geodetic.x(), geodetic.y(), geodetic.z());
    } catch (const std::exception & e) {
      RCLCPP_INFO_STREAM(get_logger(), "FGPS: Caught exception: " << e.what());
    }

    Eigen::Vector3d vel = (old_ecef - current_ecef) / (stamp.seconds() - old_stamp);    // [m/s]

    // store old values
    old_stamp = stamp.seconds();
    old_ecef = current_ecef;

    if (use_hil_gps) {
      /**
       * @note: <a href="https://mavlink.io/en/messages/common.html#HIL_GPS">HIL_GPS MAVLink message</a>
       * is supported by both Ardupilot and PX4 Firmware.
       * But on PX4 Firmware are only acceped out of HIL mode
       * if use_hil_gps flag is set (param MAV_USEHILGPS = 1).
       */
      mavlink::common::msg::HIL_GPS hil_gps {};

      vel *= 1e2;                   // [cm/s]

      // compute course over ground
      double cog;
      if (vel.x() == 0 && vel.y() == 0) {
        cog = 0;
      } else if (vel.x() >= 0 && vel.y() < 0) {
        cog = M_PI * 5 / 2 - atan2(vel.x(), vel.y());
      } else {
        cog = M_PI / 2 - atan2(vel.x(), vel.y());
      }

      // Fill in and send message
      hil_gps.time_usec = get_time_usec(stamp);                 // [useconds]
      hil_gps.lat = geodetic.x() * 1e7;                         // [degrees * 1e7]
      hil_gps.lon = geodetic.y() * 1e7;                         // [degrees * 1e7]
      hil_gps.alt = (geodetic.z() + GeographicLib::Geoid::ELLIPSOIDTOGEOID *
        (*uas->data.egm96_5)(geodetic.x(), geodetic.y())) * 1e3;    // [meters * 1e3]
      hil_gps.vel = vel.block<2, 1>(0, 0).norm();               // [cm/s]
      hil_gps.vn = vel.x();                                     // [cm/s]
      hil_gps.ve = vel.y();                                     // [cm/s]
      hil_gps.vd = vel.z();                                     // [cm/s]
      hil_gps.cog = cog * 1e2;                                  // [degrees * 1e2]
      hil_gps.eph = eph * 1e2;                                  // [cm]
      hil_gps.epv = epv * 1e2;                                  // [cm]
      hil_gps.fix_type = utils::enum_value(fix_type);
      hil_gps.satellites_visible = satellites_visible;

      uas->send_message(hil_gps);
    } else {
      /**
       * @note: <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">GPS_INPUT MAVLink message</a>
       * is currently only supported by Ardupilot firmware
       */
      mavlink::common::msg::GPS_INPUT gps_input {};

      // Fill in and send message
      gps_input.time_usec = get_time_usec(stamp);                      // [useconds]
      gps_input.gps_id = gps_id;                                //
      gps_input.ignore_flags = 0;
      if (speed_accuracy == 0.0f) {
        gps_input.ignore_flags |= utils::enum_value(GPS_INPUT_IGNORE_FLAGS::FLAG_SPEED_ACCURACY);
      }
      if (eph == 0.0f) {
        gps_input.ignore_flags |= utils::enum_value(GPS_INPUT_IGNORE_FLAGS::FLAG_HDOP);
      }
      if (epv == 0.0f) {
        gps_input.ignore_flags |= utils::enum_value(GPS_INPUT_IGNORE_FLAGS::FLAG_VDOP);
      }
      if (fabs(vel.x()) <= 0.01f && fabs(vel.y()) <= 0.01f) {
        gps_input.ignore_flags |= utils::enum_value(GPS_INPUT_IGNORE_FLAGS::FLAG_VEL_HORIZ);
      }
      if (fabs(vel.z()) <= 0.01f) {
        gps_input.ignore_flags |= utils::enum_value(GPS_INPUT_IGNORE_FLAGS::FLAG_VEL_VERT);
      }
      int64_t tdiff = (gps_input.time_usec / 1000) - UNIX_OFFSET_MSEC;
      gps_input.time_week = tdiff / MSEC_PER_WEEK;
      gps_input.time_week_ms = tdiff - (gps_input.time_week * MSEC_PER_WEEK);
      gps_input.speed_accuracy = speed_accuracy;        // [m/s] TODO how can this be dynamicaly calculated ???   // NOLINT
      gps_input.horiz_accuracy = horiz_accuracy;        // [m] will either use the static parameter value, or the dynamic covariance from function mocap_pose_cov_cb() bellow  // NOLINT
      gps_input.vert_accuracy = vert_accuracy;          // [m] will either use the static parameter value, or the dynamic covariance from function mocap_pose_cov_cb() bellow  // NOLINT
      gps_input.lat = geodetic.x() * 1e7;               // [degrees * 1e7]
      gps_input.lon = geodetic.y() * 1e7;               // [degrees * 1e7]
      gps_input.alt = (geodetic.z() + GeographicLib::Geoid::ELLIPSOIDTOGEOID *
        (*uas->data.egm96_5)(geodetic.x(), geodetic.y()));  // [meters]
      gps_input.vn = vel.x();                               // [m/s]
      gps_input.ve = vel.y();                               // [m/s]
      gps_input.vd = vel.z();                               // [m/s]
      gps_input.hdop = eph;                                 // [m]
      gps_input.vdop = epv;                                 // [m]
      gps_input.fix_type = utils::enum_value(fix_type);
      gps_input.satellites_visible = satellites_visible;

      uas->send_message(gps_input);
    }
  }

  /* -*- callbacks -*- */
  void mocap_tf_cb(const geometry_msgs::msg::TransformStamped::SharedPtr trans)
  {
    Eigen::Affine3d pos_enu; tf2::fromMsg(trans->transform, pos_enu);

    send_fake_gps(
      trans->header.stamp,
      ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
  }

  void mocap_pose_cov_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr req)
  {
    Eigen::Affine3d pos_enu; tf2::fromMsg(req->pose.pose, pos_enu);
    horiz_accuracy = (req->pose.covariance[0] + req->pose.covariance[7]) / 2.0f;
    vert_accuracy = req->pose.covariance[14];

    send_fake_gps(
      req->header.stamp,
      ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
  }

  void mocap_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr req)
  {
    Eigen::Affine3d pos_enu; tf2::fromMsg(req->pose, pos_enu);

    send_fake_gps(
      req->header.stamp,
      ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
  }

  void vision_cb(const geometry_msgs::msg::PoseStamped::SharedPtr req)
  {
    Eigen::Affine3d pos_enu; tf2::fromMsg(req->pose, pos_enu);

    send_fake_gps(
      req->header.stamp,
      ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
  }

  void transform_cb(const geometry_msgs::msg::TransformStamped & trans)
  {
    Eigen::Affine3d pos_enu; tf2::fromMsg(trans.transform, pos_enu);

    send_fake_gps(
      trans.header.stamp,
      ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::FakeGPSPlugin)
