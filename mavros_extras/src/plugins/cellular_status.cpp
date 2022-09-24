/*
 * Copyright 2021 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Cellular status plugin
 * @file cellular_status.cpp
 * @author Rui Mendes <rui.mendes@beyond-vision.pt>
 *
 * @addtogroup plugin
 * @{
 */

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/cellular_status.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Cellular status plugin.
 * @plugin cellular_status
 *
 * Users must publish to the topic the CellularStatus message and it
 * will be relayed to the mavlink components.
 */
class CellularStatusPlugin : public plugin::Plugin
{
public:
  explicit CellularStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "cellular_status")
  {
    sub_status = node->create_subscription<mavros_msgs::msg::CellularStatus>(
      "~/status", 1, std::bind(
        &CellularStatusPlugin::status_cb, this,
        _1));
  }

  Subscriptions get_subscriptions()
  {
    return {};
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::CellularStatus>::SharedPtr sub_status;

  /**
   * @brief Send Cellular Status messages to mavlink system
   *
   * Message specification: https://mavlink.io/en/messages/common.html#CELLULAR_STATUS
   * @param msg	received CellularStatus msg
   */
  void status_cb(const mavros_msgs::msg::CellularStatus::SharedPtr msg)
  {
    mavlink::common::msg::CELLULAR_STATUS cs{};

    cs.status = msg->status;
    cs.failure_reason = msg->failure_reason;
    cs.type = msg->type;
    cs.quality = msg->quality;
    cs.mcc = msg->mcc;
    cs.mnc = msg->mnc;
    cs.lac = msg->lac;

    uas->send_message(cs);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::CellularStatusPlugin)
