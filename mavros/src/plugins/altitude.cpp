/*
 * Copyright 2015 Andreas Antener <andreas@uaventure.com>.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Altitude plugin
 * @file altitude.cpp
 * @author Andreas Antener <andreas@uaventure.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/altitude.hpp"

namespace mavros
{
namespace std_plugins
{

/**
 * @brief Altitude plugin.
 * @plugin altitude
 */
class AltitudePlugin : public plugin::Plugin
{
public:
  explicit AltitudePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "altitude")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "frame_id", "map", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    auto sensor_qos = rclcpp::SensorDataQoS();

    altitude_pub = node->create_publisher<mavros_msgs::msg::Altitude>("altitude", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&AltitudePlugin::handle_altitude),
    };
  }

private:
  std::string frame_id;

  rclcpp::Publisher<mavros_msgs::msg::Altitude>::SharedPtr altitude_pub;

  void handle_altitude(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ALTITUDE & altitude, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto ros_msg = mavros_msgs::msg::Altitude();
    ros_msg.header = uas->synchronized_header(frame_id, altitude.time_usec);
    ros_msg.monotonic = altitude.altitude_monotonic;
    ros_msg.amsl = altitude.altitude_amsl;
    ros_msg.local = altitude.altitude_local;
    ros_msg.relative = altitude.altitude_relative;
    ros_msg.terrain = altitude.altitude_terrain;
    ros_msg.bottom_clearance = altitude.bottom_clearance;

    altitude_pub->publish(ros_msg);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::AltitudePlugin)
