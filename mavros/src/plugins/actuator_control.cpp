/*
 * Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief ActuatorControl plugin
 * @file actuator_control.cpp
 * @author Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/actuator_control.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief ActuatorControl plugin
 * @plugin actuator_control
 *
 * Sends actuator controls to FCU controller.
 */
class ActuatorControlPlugin : public plugin::Plugin
{
public:
  explicit ActuatorControlPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "actuator_control")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    target_actuator_control_pub = node->create_publisher<mavros_msgs::msg::ActuatorControl>(
      "target_actuator_control", sensor_qos);
    actuator_control_sub = node->create_subscription<mavros_msgs::msg::ActuatorControl>(
      "actuator_control", sensor_qos, std::bind(
        &ActuatorControlPlugin::actuator_control_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ActuatorControlPlugin::handle_actuator_control_target),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr target_actuator_control_pub;
  rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr actuator_control_sub;

  /* -*- rx handlers -*- */

  void handle_actuator_control_target(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ACTUATOR_CONTROL_TARGET & act,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    auto ract = mavros_msgs::msg::ActuatorControl();
    ract.header.stamp = uas->synchronise_stamp(act.time_usec);
    ract.group_mix = act.group_mlx;
    ract.controls = act.controls;

    target_actuator_control_pub->publish(ract);
  }

  /* -*- callbacks -*- */

  void actuator_control_cb(const mavros_msgs::msg::ActuatorControl::SharedPtr req)
  {
    //! about groups, mixing and channels: @p https://pixhawk.org/dev/mixing
    //! message definiton here: @p https://mavlink.io/en/messages/common.html#SET_ACTUATOR_CONTROL_TARGET
    mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET act{};

    act.time_usec = get_time_usec(req->header.stamp);
    act.group_mlx = req->group_mix;
    uas->msg_set_target(act);
    act.controls = req->controls;

    uas->send_message(act);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ActuatorControlPlugin)
