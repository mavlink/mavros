/*
 * Copyright 2014 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointVelocity plugin
 * @file setpoint_velocity.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using mavlink::common::MAV_FRAME;

/**
 * @brief Setpoint velocity plugin
 * @plugin setpoint_velocity
 *
 * Send setpoint velocities to FCU controller.
 */
class SetpointVelocityPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin>
{
public:
  explicit SetpointVelocityPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_velocity")
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

    // cmd_vel usually is the topic used for velocity control in many controllers / planners
    vel_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel", sensor_qos, std::bind(
        &SetpointVelocityPlugin::vel_cb, this,
        _1));
    vel_unstamped_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel_unstamped",
      sensor_qos, std::bind(
        &SetpointVelocityPlugin::vel_unstamped_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin>;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_unstamped_sub;

  MAV_FRAME mav_frame;

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send velocity to FCU velocity controller
   *
   * @warning Send only VX VY VZ. ENU frame.
   */
  void send_setpoint_velocity(
    const rclcpp::Time & stamp, const Eigen::Vector3d & vel_enu,
    const double yaw_rate)
  {
    /**
     * Documentation start from bit 1 instead 0;
     * Ignore position and accel vectors, yaw.
     */
    uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);
    auto vel = [&]() {
        if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
          return ftf::transform_frame_baselink_aircraft(vel_enu);
        } else {
          return ftf::transform_frame_enu_ned(vel_enu);
        }
      } ();

    auto yr = [&]() {
        if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
          return ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(0.0, 0.0, yaw_rate));
        } else {
          return ftf::transform_frame_ned_enu(Eigen::Vector3d(0.0, 0.0, yaw_rate));
        }
      } ();

    set_position_target_local_ned(
      get_time_boot_ms(stamp),
      utils::enum_value(mav_frame),
      ignore_all_except_v_xyz_yr,
      Eigen::Vector3d::Zero(),
      vel,
      Eigen::Vector3d::Zero(),
      0.0, yr.z());
  }

  /* -*- callbacks -*- */

  void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr req)
  {
    Eigen::Vector3d vel_enu;

    tf2::fromMsg(req->twist.linear, vel_enu);
    send_setpoint_velocity(
      req->header.stamp, vel_enu,
      req->twist.angular.z);
  }

  void vel_unstamped_cb(const geometry_msgs::msg::Twist::SharedPtr req)
  {
    Eigen::Vector3d vel_enu;

    tf2::fromMsg(req->linear, vel_enu);
    send_setpoint_velocity(
      node->now(), vel_enu,
      req->angular.z);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointVelocityPlugin)
