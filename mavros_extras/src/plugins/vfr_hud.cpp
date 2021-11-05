/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief VFR HUD plugin
 * @file vfr_hud.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/vfr_hud.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief VFR HUD plugin.
 * @plugin vfr_hud
 */
class VfrHudPlugin : public plugin::Plugin
{
public:
  explicit VfrHudPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "vfr_hud")
  {
    vfr_pub = node->create_publisher<mavros_msgs::msg::VfrHud>("vfr_hud", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&VfrHudPlugin::handle_vfr_hud),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::VfrHud>::SharedPtr vfr_pub;

  void handle_vfr_hud(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::VFR_HUD & vfr_hud,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto vmsg = mavros_msgs::msg::VfrHud();

    vmsg.header.stamp = node->now();
    vmsg.airspeed = vfr_hud.airspeed;
    vmsg.groundspeed = vfr_hud.groundspeed;
    vmsg.heading = vfr_hud.heading;
    vmsg.throttle = vfr_hud.throttle / 100.0;   // comes in 0..100 range
    vmsg.altitude = vfr_hud.alt;
    vmsg.climb = vfr_hud.climb;

    vfr_pub->publish(vmsg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VfrHudPlugin)
