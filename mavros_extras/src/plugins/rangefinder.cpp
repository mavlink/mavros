/*
 * Copyright 2016 Ardupilot.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Rangefinder plugin
 * @file rangefinder.cpp
 * @author Pierre Kancir <khancyr@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "sensor_msgs/msg/range.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Ardupilot Rangefinder plugin.
 * @plugin rangefinder
 *
 * This plugin allows publishing rangefinder sensor data from Ardupilot FCU to ROS.
 */
class RangefinderPlugin : public plugin::Plugin
{
public:
  explicit RangefinderPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "rangefinder")
  {
    rangefinder_pub = node->create_publisher<sensor_msgs::msg::Range>("~/rangefinder", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&RangefinderPlugin::handle_rangefinder)
    };
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr rangefinder_pub;

  void handle_rangefinder(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::RANGEFINDER & rangefinder,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto rangefinder_msg = sensor_msgs::msg::Range();

    rangefinder_msg.header.stamp = node->now();
    rangefinder_msg.header.frame_id = "/rangefinder";
    rangefinder_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    rangefinder_msg.field_of_view = 0;
    rangefinder_msg.min_range = 0;
    rangefinder_msg.max_range = 1000;
    rangefinder_msg.range = rangefinder.distance;

    rangefinder_pub->publish(rangefinder_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::RangefinderPlugin)
