/**
 * @brief SIM_STATE plugin
 * @file sim_state.cpp
 * @author Zeke Sarosi
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2025 Zeke Sarosi
 *
 * This file is part of the MAVROS package and subject to the license terms
 * in the top-level LICENSE file of the MAVROS repository.
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include "mavros/frame_tf.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;

/**
 * @brief SIM_STATE plugin.
 * @plugin sim_state
 *
 * Adds support for MAVLink SIM_STATE (id 108) messages and republishes fields to ROS 2 topics.
 * Intended for simulation use as a high-accuracy ground-truth feed when developing autonomy.
 * Currently verified with ArduCopter SITL.
 */
class SimStatePlugin : public plugin::Plugin
{
public:
  explicit SimStatePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "sim_state")
  {
//! Publish IMU attitude (orientation + angular velocity) in ENU/base_link
   //! from MAVLink SIM_STATE.
    attitude_pub = node->create_publisher<sensor_msgs::msg::Imu>(
      "~/attitude", 10);

   //! Publish linear acceleration (m/s^2) in ENU/map from MAVLink SIM_STATE.
    acceleration_pub = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "~/acceleration", 10);

   //! Publish body-frame velocity (linear + angular) in base_link from
   //! MAVLink SIM_STATE.
    velocity_body_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/velocity_body", 10);

   //! Publish local-frame velocity (linear + angular) in ENU/map from
   //! MAVLink SIM_STATE.
    velocity_local_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/velocity_local", 10);

   //! Publish global position (WGS84 NavSatFix) from MAVLink SIM_STATE.
    global_position_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>(
      "~/global_position", 10);
  }

  Subscriptions get_subscriptions() override
  {
    // Subscribe to MAVLink SIM_STATE (id 108) messages and process via handle_sim_state()
    return {
      make_handler(&SimStatePlugin::handle_sim_state)
    };
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr attitude_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acceleration_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_body_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_local_pub;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_pub;

  void handle_sim_state(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SIM_STATE & sim_state,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    const rclcpp::Time stamp = node->now();

    // Publish attitude (orientation + angular velocity) in ENU/base_link
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = "base_link";
    // SIM_STATE quaternion ordering: q1=w, q2=x, q3=y, q4=z (aircraft->NED)
    auto ned_aircraft_orientation = Eigen::Quaterniond(
      sim_state.q1, sim_state.q2, sim_state.q3, sim_state.q4);
    auto enu_baselink_orientation = mavros::ftf::transform_orientation_aircraft_baselink(
      mavros::ftf::transform_orientation_ned_enu(ned_aircraft_orientation));
    imu_msg.orientation = tf2::toMsg(enu_baselink_orientation);
    // Unknown orientation covariance when not provided
    imu_msg.orientation_covariance[0] = -1.0;
    // Angular velocity (rad/s) given in aircraft frame -> rotate to base_link
    auto gyro_frd = Eigen::Vector3d(sim_state.xgyro, sim_state.ygyro, sim_state.zgyro);
    auto gyro_flu = mavros::ftf::transform_frame_aircraft_baselink(gyro_frd);
    tf2::toMsg(gyro_flu, imu_msg.angular_velocity);
    // Mark angular velocity covariance unknown
    imu_msg.angular_velocity_covariance[0] = -1.0;
    // Do not duplicate linear acceleration here; it is published on a dedicated topic
    imu_msg.linear_acceleration_covariance[0] = -1.0;
    attitude_pub->publish(imu_msg);

    // Publish linear acceleration (m/s^2) in ENU/map
    auto accel_msg = geometry_msgs::msg::Vector3Stamped();
    accel_msg.header.stamp = stamp;
    accel_msg.header.frame_id = "map";
    auto acc_ned = Eigen::Vector3d(sim_state.xacc, sim_state.yacc, sim_state.zacc);
    auto acc_enu = mavros::ftf::transform_frame_ned_enu(acc_ned);
    tf2::toMsg(acc_enu, accel_msg.vector);
    acceleration_pub->publish(accel_msg);

    // Compute velocities: NED -> ENU and ENU -> base_link
    auto vel_ned = Eigen::Vector3d(sim_state.vn, sim_state.ve, sim_state.vd);
    auto vel_enu = mavros::ftf::transform_frame_ned_enu(vel_ned);
    auto baselink_linear = mavros::ftf::transform_frame_enu_baselink(
      vel_enu, enu_baselink_orientation.inverse());

    // Publish body velocity (base_link)
    auto twist_body = geometry_msgs::msg::TwistStamped();
    twist_body.header.stamp = stamp;
    twist_body.header.frame_id = "base_link";
    tf2::toMsg(baselink_linear, twist_body.twist.linear);
    tf2::toMsg(gyro_flu, twist_body.twist.angular);
    velocity_body_pub->publish(twist_body);

    // Publish local velocity (ENU)
    auto twist_local = geometry_msgs::msg::TwistStamped();
    twist_local.header.stamp = stamp;
    twist_local.header.frame_id = "map";
    tf2::toMsg(vel_enu, twist_local.twist.linear);
    auto enu_angular = mavros::ftf::transform_frame_baselink_enu(
      gyro_flu, enu_baselink_orientation);
    tf2::toMsg(enu_angular, twist_local.twist.angular);
    velocity_local_pub->publish(twist_local);

    // Publish global position as NavSatFix (lat, lon, alt)
    auto fix_msg = sensor_msgs::msg::NavSatFix();
    fix_msg.header.stamp = stamp;
    fix_msg.header.frame_id = "base_link";

    // Prefer higher precision int fields when available
    const bool have_int_lat = sim_state.lat_int != 0;
    const bool have_int_lon = sim_state.lon_int != 0;
    if (have_int_lat) {
      fix_msg.latitude = static_cast<double>(sim_state.lat_int) / 1e7;
    } else {
      fix_msg.latitude = static_cast<double>(sim_state.lat);
    }
    if (have_int_lon) {
      fix_msg.longitude = static_cast<double>(sim_state.lon_int) / 1e7;
    } else {
      fix_msg.longitude = static_cast<double>(sim_state.lon);
    }
    fix_msg.altitude = static_cast<double>(sim_state.alt);

    // No explicit status info in SIM_STATE; mark as unknown
    fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix_msg.status.service = 0;

    // Covariance: use provided std devs if present (>0), otherwise unknown
    const double std_h = static_cast<double>(sim_state.std_dev_horz);
    const double std_v = static_cast<double>(sim_state.std_dev_vert);
    if (std_h > 0.0 && std_v > 0.0) {
      const double var_h = std_h * std_h;
      const double var_v = std_v * std_v;
      // Diagonal covariance: [x, y, z]
      fix_msg.position_covariance[0] = var_h;
      fix_msg.position_covariance[4] = var_h;
      fix_msg.position_covariance[8] = var_v;
      fix_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      fix_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    global_position_pub->publish(fix_msg);
  }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::SimStatePlugin)

/** @} */
