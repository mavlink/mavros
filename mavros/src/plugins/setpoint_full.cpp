/**
 * @brief FullSetpoint plugin
 * @file setpoint_full.cpp
 * @author Eesh Vij <evij@uci.edu>
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

#include "mavros_msgs/msg/full_setpoint.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Full setpoint plugin
 * @plugin setpoint_full
 *
 * Send a full localNED setpoint to FCU controller.
 */
class SetpointFullPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointFullPlugin>
{
public:
  explicit SetpointFullPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_full")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    fullset_sub = node->create_subscription<mavros_msgs::msg::FullSetpoint>(
      "~/cmd_full_setpoint", sensor_qos, std::bind(
        &SetpointFullPlugin::setpoint_cb, this,
        _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointFullPlugin>;

  rclcpp::Subscription<mavros_msgs::msg::FullSetpoint>::SharedPtr fullset_sub;

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send a full localNED setpoint to FCU controller.
   *
   */
  void send_setpoint_full(const rclcpp::Time & stamp, const uint16_t type_mask,
                          const Eigen::Vector3d & pos_ned,
                          const Eigen::Vector3d & vel_ned,
                          const Eigen::Vector3d & accel_ned,
                          const double yaw, const double yaw_rate)
  {
    using mavlink::common::MAV_FRAME;

    // auto accel = ftf::transform_frame_enu_ned(accel_enu);

    set_position_target_local_ned(
      get_time_boot_ms(stamp),
      utils::enum_value(MAV_FRAME::LOCAL_NED),
      type_mask,
      pos_ned,
      vel_ned,
      accel_ned,
      yaw, yaw_rate);
  }

  /* -*- callbacks -*- */

  void setpoint_cb(const mavros_msgs::msg::FullSetpoint::SharedPtr req)
  {
    Eigen::Vector3d pos_ned;
    Eigen::Vector3d vel_ned;
    Eigen::Vector3d accel_ned;

    tf2::fromMsg(req->position, pos_ned);
    tf2::fromMsg(req->velocity, vel_ned);
    tf2::fromMsg(req->acceleration, accel_ned);
    send_setpoint_full(
      req->header.stamp,
      req->type_mask,
      pos_ned,
      vel_ned,
      accel_ned,
      req->yaw,
      req->yaw_rate);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointFullPlugin)
