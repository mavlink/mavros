/*
 * Copyright 2022 Sanket Sharma, Randy Mackay.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Guided target plugin
 * @file guided_target.cpp
 * @author Randy Mackay <rmackay9@yahoo.com> , Sanket Sharma <sharma.sanket272@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include <GeographicLib/Geocentric.hpp>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/global_position_target.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief guided target plugin
 * @plugin guided_target
 *
 * Send and receive setpoint positions from FCU controller.
 */
class GuidedTargetPlugin : public plugin::Plugin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit GuidedTargetPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "guided_target"),
    is_map_init(false),
    prev()
  {
    enable_node_watch_parameters();

    // frame params:
    node_declare_and_watch_parameter(
      "frame_id", "map", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    // Publish targets received from FCU
    setpointg_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10);


    // Subscriber for global origin (aka map origin).
    gp_origin_sub = node->create_subscription<geographic_msgs::msg::GeoPointStamped>(
      "global_position/gp_origin", 10, std::bind(&GuidedTargetPlugin::gp_origin_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&GuidedTargetPlugin::handle_position_target_global_int)
    };
  }

private:
  //! global position target from FCU
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpointg_pub;
  //! global origin LLA
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gp_origin_sub;

  Eigen::Vector3d current_gps;          //!< geodetic coordinates LLA
  Eigen::Vector3d current_local_pos;    //!< Current local position in ENU

  Eigen::Vector3d map_origin {};        //!< oigin of map frame [lla]
  Eigen::Vector3d ecef_origin {};       //!< geocentric origin [m]

  //! old time gps time stamp in [ms], to check if new gps msg is received
  uint32_t old_gps_stamp = 0;

  std::string frame_id;
  bool is_map_init;

  Eigen::Vector2d prev;

  /* -*- mid-level helpers -*- */

  /**
   * global origin in LLA
   */
  void gp_origin_cb(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg)
  {
    ecef_origin = {msg->position.latitude, msg->position.longitude, msg->position.altitude};
    /**
         * @brief Conversion from ECEF (Earth-Centered, Earth-Fixed) to geodetic coordinates (LLA)
        */
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
      GeographicLib::Constants::WGS84_f());
    try {
      earth.Reverse(
        ecef_origin.x(), ecef_origin.y(), ecef_origin.z(),
        map_origin.x(), map_origin.y(), map_origin.z());
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(get_logger(), "setpoint: Caught exception: " << e.what() << std::endl);
      return;
    }

    is_map_init = true;
  }


  /* -*- rx handler -*- */

  /**
   * @brief handle POSITION_TARGET_GLOBAL_INT mavlink msg
   * handles and publishes position target received from FCU
   */
  void handle_position_target_global_int(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::POSITION_TARGET_GLOBAL_INT & position_target,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    using GPT = mavros_msgs::msg::GlobalPositionTarget;
    auto lg = get_logger();

    /* check if type_mask field ignores position*/
    if ((position_target.type_mask & (GPT::IGNORE_LATITUDE | GPT::IGNORE_LONGITUDE)) > 0) {
      RCLCPP_WARN(lg, "lat and/or lon ignored");
      return;
    }

    /* check origin has been set */
    if (!is_map_init) {
      RCLCPP_WARN(lg, "PositionTargetGlobal failed because no origin");
    }

    /* convert lat/lon target to ECEF */
    Eigen::Vector3d pos_target_ecef {};         //!< local ECEF coordinates on map frame [m]
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
      GeographicLib::Constants::WGS84_f());
    try {
      earth.Forward(
        position_target.lat_int / 1E7, position_target.lon_int / 1E7, position_target.alt / 1E3,
        pos_target_ecef.x(), pos_target_ecef.y(), pos_target_ecef.z());
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(lg, "Caught exception: " << e.what() << std::endl);
      return;
    }

    /* create position target PoseStamped message */
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header = uas->synchronized_header(frame_id, position_target.time_boot_ms);
    pose.pose.orientation.w = 1.0;              // unit quaternion with no rotation

    /* convert ECEF target to ENU */
    const Eigen::Vector3d local_ecef = pos_target_ecef - ecef_origin;
    pose.pose.position = tf2::toMsg(ftf::transform_frame_ecef_enu(local_ecef, map_origin));
    pose.pose.position.z = 0.0;                 // force z-axis to zero

    /* publish target */
    if (pose.pose.position.x != prev.x() || pose.pose.position.y != prev.y()) {
      setpointg_pub->publish(pose);
    }

    prev.x() = pose.pose.position.x;
    prev.y() = pose.pose.position.y;
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::GuidedTargetPlugin)
