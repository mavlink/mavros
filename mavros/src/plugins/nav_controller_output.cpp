/*
 * Copyright 2019 Karthik Desai
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief NavControllerOutput plugin
 * @file nav_controller_output.cpp
 * @author Karthik Desai <karthik.desai@iav.de>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/nav_controller_output.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief nav controller output plugin.
 * @plugin nav_controller_output
 *
 * Publishes nav_controller_output message https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT
 */
class NavControllerOutputPlugin : public plugin::Plugin
{
public:
  explicit NavControllerOutputPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "nav_controller_output")
  {
    nco_pub = node->create_publisher<mavros_msgs::msg::NavControllerOutput>("~/output", 10);
  }

  Subscriptions get_subscriptions()
  {
    return {
      make_handler(&NavControllerOutputPlugin::handle_nav_controller_output),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::NavControllerOutput>::SharedPtr nco_pub;

  void handle_nav_controller_output(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::NAV_CONTROLLER_OUTPUT & nav_controller_output,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto nco_msg = mavros_msgs::msg::NavControllerOutput();

    nco_msg.header.stamp = node->now();
    nco_msg.nav_roll = nav_controller_output.nav_roll;
    nco_msg.nav_pitch = nav_controller_output.nav_pitch;
    nco_msg.nav_bearing = nav_controller_output.nav_bearing;
    nco_msg.target_bearing = nav_controller_output.target_bearing;
    nco_msg.wp_dist = nav_controller_output.wp_dist;
    nco_msg.alt_error = nav_controller_output.alt_error;
    nco_msg.aspd_error = nav_controller_output.aspd_error;
    nco_msg.xtrack_error = nav_controller_output.xtrack_error;

    nco_pub->publish(nco_msg);
  }
};
}   // namespace std_plugins
}   // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::NavControllerOutputPlugin)
