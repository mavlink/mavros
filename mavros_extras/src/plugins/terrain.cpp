/*
 * Copyright 2021 Ardupilot.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Terrain plugin
 * @file terrain.cpp
 * @author Matt Anderson <anderson_rayner@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/terrain_report.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Terrain height plugin.
 * @plugin terrain
 *
 * This plugin allows publishing of terrain height estimate from FCU to ROS.
 *
 */
class TerrainPlugin : public plugin::Plugin
{
public:
  explicit TerrainPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "terrain")
  {
    terrain_report_pub = node->create_publisher<mavros_msgs::msg::TerrainReport>("~/report", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&TerrainPlugin::handle_terrain_report)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::TerrainReport>::SharedPtr terrain_report_pub;

  void handle_terrain_report(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TERRAIN_REPORT & report,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto terrain_report_msg = mavros_msgs::msg::TerrainReport();

    terrain_report_msg.header.stamp = node->now();
    terrain_report_msg.header.frame_id = "terrain";

    terrain_report_msg.latitude = static_cast<double>(report.lat) / 1e7;
    terrain_report_msg.longitude = static_cast<double>(report.lon) / 1e7;
    terrain_report_msg.spacing = report.spacing;
    terrain_report_msg.terrain_height = report.terrain_height;
    terrain_report_msg.current_height = report.current_height;
    terrain_report_msg.pending = report.pending;
    terrain_report_msg.loaded = report.loaded;

    terrain_report_pub->publish(terrain_report_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TerrainPlugin)
