/*
 * Copyright 2020 Morten Fyhn Amundsen <morten.fyhn.amundsen@gmail.com>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Onboard Computer Status plugin
 * @file onboard_computer_status.cpp
 * @author Morten Fyhn Amundsen <morten.fyhn.amundsen@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <cstring>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/play_tune_v2.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Play Tune service
 * @plugin play_tune
 */
class PlayTunePlugin : public plugin::Plugin
{
public:
  explicit PlayTunePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "play_tune")
  {
    sub =
      node->create_subscription<mavros_msgs::msg::PlayTuneV2>(
      "play_tune", 1,
      std::bind(&PlayTunePlugin::callback, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* No subscriptions */};
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::PlayTuneV2>::SharedPtr sub;

  void callback(const mavros_msgs::msg::PlayTuneV2::SharedPtr tune)
  {
    auto msg = mavlink::common::msg::PLAY_TUNE_V2{};

    uas->msg_set_target(msg);
    msg.format = tune->format;
    mavlink::set_string_z(msg.tune, tune->tune);

    uas->send_message(msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::PlayTunePlugin)
