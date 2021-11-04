/*
 * Copyright 2018 Tanja Baumann.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Companion Status plugin
 * @file companion_status.cpp
 * @author Tanja Baumann <tanja@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/companion_process_status.hpp"

namespace mavros
{
namespace extra_plugins
{

//! Mavlink enumerations
using mavlink::minimal::MAV_TYPE;
using mavlink::minimal::MAV_STATE;
using mavlink::minimal::MAV_COMPONENT;
using mavlink::minimal::MAV_AUTOPILOT;
using mavlink::minimal::MAV_MODE_FLAG;
using utils::enum_value;

/**
 * @brief Obstacle companion process status plugin
 * @plugin companion_process_status
 *
 * Publishes the status of components running on the companion computer
 * @see status_cb()
 */
class CompanionProcessStatusPlugin : public plugin::Plugin
{
public:
  explicit CompanionProcessStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "companion_process")
  {
    status_sub = node->create_subscription<mavros_msgs::msg::CompanionProcessStatus>(
      "~/status", 10, std::bind(
        &CompanionProcessStatusPlugin::status_cb, this,
        std::
        placeholders::_1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr status_sub;

  /**
   * @brief Send companion process status to FCU over a heartbeat message
   *
   * Message specification: https://mavlink.io/en/messages/common.html#HEARTBEAT
   * @param req	received CompanionProcessStatus msg
   */
  void status_cb(const mavros_msgs::msg::CompanionProcessStatus::SharedPtr req)
  {
    mavlink::minimal::msg::HEARTBEAT heartbeat {};

    heartbeat.type = enum_value(MAV_TYPE::ONBOARD_CONTROLLER);
    heartbeat.autopilot = enum_value(MAV_AUTOPILOT::PX4);
    heartbeat.base_mode = enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
    heartbeat.system_status = req->state;               // enum="MAV_STATE" System status flag

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "companion process component id: " << utils::to_string_enum<MAV_COMPONENT>(req->component) <<
        " companion process status: " << utils::to_string_enum<MAV_STATE>(
        heartbeat.system_status) << std::endl << heartbeat.to_yaml());

    uas->send_message(heartbeat, req->component);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::CompanionProcessStatusPlugin)
