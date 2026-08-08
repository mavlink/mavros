/*
 * Copyright 2021 Ardupilot.
 * Copyright 2026 Zeke Sarosi.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Terrain plugin
 * @file terrain.cpp
 * @author Matt Anderson <anderson_rayner@hotmail.com>
 * @author Zeke Sarosi <zeke.sarosi@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *
 * Handles the MAVLink terrain protocol.  Heavy lifting (SRTM tile
 * download, caching, elevation lookup) is handled by a separate
 * terrain_server node (Python, mavros_extras).
 */

#include <cmath>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/terrain_data.hpp"
#include "mavros_msgs/msg/terrain_report.hpp"
#include "mavros_msgs/msg/terrain_request.hpp"
#include "mavros_msgs/srv/terrain_check.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Terrain plugin.
 * @plugin terrain
 *
 * Bridges the MAVLink terrain protocol between the FCU and a companion
 * terrain_server node that serves SRTM elevation data.
 *
 * Protocol spec: https://mavlink.io/en/services/terrain.html
 *
 * TERRAIN_REQUEST from the FCU is forwarded to the server for SRTM
 * lookup; the server responds with TERRAIN_DATA blocks that are sent
 * back to the FCU.  TERRAIN_CHECK point queries are handled via a
 * service call to the server, which responds with elevation data
 * that is returned to the FCU as TERRAIN_REPORT.
 */
class TerrainPlugin : public plugin::Plugin
{
public:
  explicit TerrainPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "terrain")
  {
    // Terrain height reports from FCU and check-service responses

    //! Publish terrain reports from MAVLink TERRAIN_REPORT.
    report_pub_ = node->create_publisher<mavros_msgs::msg::TerrainReport>(
      "~/report", 10);
    // Grid data requests forwarded from FCU for SRTM lookup

    //! Publish terrain requests from MAVLink TERRAIN_REQUEST.
    request_pub_ = node->create_publisher<mavros_msgs::msg::TerrainRequest>(
      "~/request", 10);

    // Filled terrain grid blocks from terrain_server

    //! Subscribe to TerrainData to send as TERRAIN_DATA to the FCU.
    data_sub_ = node->create_subscription<mavros_msgs::msg::TerrainData>(
      "~/data", 64,
      std::bind(&TerrainPlugin::data_cb, this, _1));

    // Point elevation query handled by terrain_server

    //! Client to query terrain elevation (terrain/check).
    check_client_ = node->create_client<mavros_msgs::srv::TerrainCheck>(
      "~/check");
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&TerrainPlugin::handle_terrain_report),
      make_handler(&TerrainPlugin::handle_terrain_request),
      make_handler(&TerrainPlugin::handle_terrain_check),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::TerrainReport>::SharedPtr report_pub_;
  rclcpp::Publisher<mavros_msgs::msg::TerrainRequest>::SharedPtr request_pub_;
  rclcpp::Subscription<mavros_msgs::msg::TerrainData>::SharedPtr data_sub_;
  rclcpp::Client<mavros_msgs::srv::TerrainCheck>::SharedPtr check_client_;

  void handle_terrain_report(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TERRAIN_REPORT & report,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto ros_msg = mavros_msgs::msg::TerrainReport();
    ros_msg.header.stamp = node->now();
    ros_msg.header.frame_id = "terrain";
    ros_msg.latitude = static_cast<double>(report.lat) / 1e7;
    ros_msg.longitude = static_cast<double>(report.lon) / 1e7;
    ros_msg.spacing = report.spacing;
    ros_msg.terrain_height = report.terrain_height;
    ros_msg.current_height = report.current_height;
    ros_msg.pending = report.pending;
    ros_msg.loaded = report.loaded;

    report_pub_->publish(ros_msg);
  }

  void handle_terrain_request(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TERRAIN_REQUEST & request,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto ros_msg = mavros_msgs::msg::TerrainRequest();
    ros_msg.header.stamp = node->now();
    ros_msg.header.frame_id = "terrain";
    ros_msg.latitude = static_cast<double>(request.lat) / 1e7;
    ros_msg.longitude = static_cast<double>(request.lon) / 1e7;
    ros_msg.grid_spacing = request.grid_spacing;
    ros_msg.mask = request.mask;

    request_pub_->publish(ros_msg);
  }

  void handle_terrain_check(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TERRAIN_CHECK & check,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (!check_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *node->get_clock(), 5000,
        "terrain/check service not available");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::TerrainCheck::Request>();
    req->latitude = check.lat / 1e7;
    req->longitude = check.lon / 1e7;

    check_client_->async_send_request(req,
      [this, check](
        rclcpp::Client<mavros_msgs::srv::TerrainCheck>::SharedFuture future)
      {
        auto result = future.get();
        if (!result || !result->success) {
          RCLCPP_WARN(get_logger(),
            "TERRAIN_CHECK: no data for lat=%.7f lon=%.7f",
            check.lat / 1e7, check.lon / 1e7);
          return;
        }

        mavlink::common::msg::TERRAIN_REPORT rpt{};
        rpt.lat = check.lat;
        rpt.lon = check.lon;
        rpt.spacing = 0;
        rpt.terrain_height = result->terrain_height;
        rpt.current_height = 0;
        rpt.pending = 0;
        rpt.loaded = 1;
        uas->send_message(rpt);

        auto ros_rpt = mavros_msgs::msg::TerrainReport();
        ros_rpt.header.stamp = node->now();
        ros_rpt.header.frame_id = "terrain";
        ros_rpt.latitude = check.lat / 1e7;
        ros_rpt.longitude = check.lon / 1e7;
        ros_rpt.spacing = 0;
        ros_rpt.terrain_height = result->terrain_height;
        ros_rpt.current_height = 0;
        ros_rpt.pending = 0;
        ros_rpt.loaded = 1;
        report_pub_->publish(ros_rpt);
      });
  }

  void data_cb(const mavros_msgs::msg::TerrainData::SharedPtr msg)
  {
    mavlink::common::msg::TERRAIN_DATA td{};
    td.lat = std::lround(msg->latitude * 1e7);
    td.lon = std::lround(msg->longitude * 1e7);
    td.grid_spacing = msg->grid_spacing;
    td.gridbit = msg->gridbit;
    std::copy(msg->data.begin(), msg->data.end(), std::begin(td.data));

    uas->send_message(td);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TerrainPlugin)
