/*
 * Copyright 2014 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointAcceleration plugin
 * @file setpoint_accel.cpp
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

#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Setpoint acceleration/force plugin
 * @plugin setpoint_accel
 *
 * Send setpoint accelerations/forces to FCU controller.
 */
class SetpointAccelerationPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointAccelerationPlugin>
{
public:
  explicit SetpointAccelerationPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_accel")
  {
    node->declare_parameter("send_force", false);

    auto sensor_qos = rclcpp::SensorDataQoS();

    accel_sub = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "~/accel", sensor_qos, std::bind(
        &SetpointAccelerationPlugin::accel_cb, this,
        _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointAccelerationPlugin>;

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_sub;

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send acceleration/force to FCU acceleration controller.
   *
   * @warning Send only AFX AFY AFZ. ENU frame.
   */
  void send_setpoint_acceleration(const rclcpp::Time & stamp, const Eigen::Vector3d & accel_enu)
  {
    using mavlink::common::MAV_FRAME;

    bool send_force;
    node->get_parameter("send_force", send_force);

    /* Documentation start from bit 1 instead 0.
     * Ignore position and velocity vectors, yaw and yaw rate
     */
    uint16_t ignore_all_except_a_xyz = (3 << 10) | (7 << 3) | (7 << 0);

    if (send_force) {
      ignore_all_except_a_xyz |= (1 << 9);
    }

    auto accel = ftf::transform_frame_enu_ned(accel_enu);

    set_position_target_local_ned(
      get_time_boot_ms(stamp),
      utils::enum_value(MAV_FRAME::LOCAL_NED),
      ignore_all_except_a_xyz,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      accel,
      0.0, 0.0);
  }

  /* -*- callbacks -*- */

  void accel_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr req)
  {
    Eigen::Vector3d accel_enu;

    tf2::fromMsg(req->vector, accel_enu);
    send_setpoint_acceleration(req->header.stamp, accel_enu);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointAccelerationPlugin)
