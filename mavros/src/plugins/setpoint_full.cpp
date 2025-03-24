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
  void send_setpoint_full(const rclcpp::Time & stamp, const Eigen::Vector3d & accel_enu) //TODO: Adjust params
  {
    using mavlink::common::MAV_FRAME;

    /* Documentation start from bit 1 instead 0.
     * Ignore position and velocity vectors, yaw and yaw rate
     */
    uint16_t ignore_all_except_a_xyz = (3 << 10) | (7 << 3) | (7 << 0);

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

  void setpoint_cb(const mavros_msgs::msg::FullSetpoint::SharedPtr req)
  {
    //TODO: Adjust params
    // Eigen::Vector3d accel_enu;

    // tf2::fromMsg(req->vector, accel_enu);
    // send_setpoint_acceleration(req->header.stamp, accel_enu);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointFullPlugin)
