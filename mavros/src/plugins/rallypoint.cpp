/*
 * Copyright 2021 Charlie Burge.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Rallypoint plugin
 * @file rallypoint.cpp
 * @author Charlie Burge <charlieburge@yahoo.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "mavros/mission_protocol_base.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT

/**
 * @brief Rallypoint manipulation plugin
 * @plugin rallypoint
 */
class RallypointPlugin : public plugin::MissionBase
{
public:
  explicit RallypointPlugin(plugin::UASPtr uas_)
  : MissionBase(uas_, "rallypoint", plugin::MTYPE::RALLY, "RP", 20s)
  {
    enable_node_watch_parameters();

    // NOTE(vooon): I'm not quite sure that this option would work with mavros router
    node_declare_and_watch_parameter(
      "pull_after_gcs", true, [&](const rclcpp::Parameter & p) {
        do_pull_after_gcs = p.as_bool();
      });

    node_declare_and_watch_parameter(
      "use_mission_item_int", true, [&](const rclcpp::Parameter & p) {
        use_mission_item_int = p.as_bool();
      });

    auto rp_qos = rclcpp::QoS(10).transient_local();

    rp_list_pub = node->create_publisher<mavros_msgs::msg::WaypointList>("~/rallypoints", rp_qos);

    pull_srv =
      node->create_service<mavros_msgs::srv::WaypointPull>(
      "~/pull",
      std::bind(&RallypointPlugin::pull_cb, this, _1, _2));
    push_srv =
      node->create_service<mavros_msgs::srv::WaypointPush>(
      "~/push",
      std::bind(&RallypointPlugin::push_cb, this, _1, _2));
    clear_srv =
      node->create_service<mavros_msgs::srv::WaypointClear>(
      "~/clear",
      std::bind(&RallypointPlugin::clear_cb, this, _1, _2));

    enable_connection_cb();
    enable_capabilities_cb();
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr rp_list_pub;

  rclcpp::Service<mavros_msgs::srv::WaypointPull>::SharedPtr pull_srv;
  rclcpp::Service<mavros_msgs::srv::WaypointPush>::SharedPtr push_srv;
  rclcpp::Service<mavros_msgs::srv::WaypointClear>::SharedPtr clear_srv;

  /* -*- mid-level helpers -*- */

  // Acts when capabilities of the fcu are changed
  void capabilities_cb(uas::MAV_CAP capabilities [[maybe_unused]]) override
  {
    lock_guard lock(mutex);

    if (uas->has_capability(uas::MAV_CAP::MISSION_INT)) {
      use_mission_item_int = true;
      mission_item_int_support_confirmed = true;
      RCLCPP_INFO(get_logger(), "%s: Using MISSION_ITEM_INT", log_prefix);
    } else {
      use_mission_item_int = false;
      mission_item_int_support_confirmed = false;
      RCLCPP_WARN(get_logger(), "%s: Falling back to MISSION_ITEM", log_prefix);
    }
  }

  // Act on first heartbeat from FCU
  void connection_cb(bool connected) override
  {
    lock_guard lock(mutex);

    if (connected) {
      schedule_pull(BOOTUP_TIME);
    } else if (schedule_timer) {
      schedule_timer->cancel();
    }
  }

  //! @brief publish the updated waypoint list after operation
  void publish_waypoints() override
  {
    auto wpl = mavros_msgs::msg::WaypointList();
    unique_lock lock(mutex);

    wpl.current_seq = wp_cur_active;
    wpl.waypoints.reserve(waypoints.size());
    for (auto & it : waypoints) {
      wpl.waypoints.push_back(it);
    }

    lock.unlock();
    rp_list_pub->publish(wpl);
  }

  void publish_reached(const uint16_t seq [[maybe_unused]]) override
  {}

  /* -*- ROS callbacks -*- */

  void pull_cb(
    const mavros_msgs::srv::WaypointPull::Request::SharedPtr req [[maybe_unused]],
    mavros_msgs::srv::WaypointPull::Response::SharedPtr res)
  {
    unique_lock lock(mutex);

    if (wp_state != WP::IDLE) {
      // Wrong initial state, other operation in progress?
      return;
    }

    wp_state = WP::RXLIST;
    wp_count = 0;
    restart_timeout_timer();

    lock.unlock();
    mission_request_list();
    res->success = wait_fetch_all();
    lock.lock();

    res->wp_received = waypoints.size();
    go_idle();  // not nessessary, but prevents from blocking
  }

  void push_cb(
    const mavros_msgs::srv::WaypointPush::Request::SharedPtr req,
    mavros_msgs::srv::WaypointPush::Response::SharedPtr res)
  {
    unique_lock lock(mutex);

    if (wp_state != WP::IDLE) {
      // Wrong initial state, other operation in progress?
      return;
    }

    if (req->start_index) {
      // Partial update not supported for rallypoints
      RCLCPP_WARN(get_logger(), "%s: Partial update for rallypoints not supported", log_prefix);
      res->success = false;
      return;
    }

    // Full waypoint update
    wp_state = WP::TXLIST;

    send_waypoints.clear();
    send_waypoints.reserve(req->waypoints.size());
    for (auto & wp : req->waypoints) {
      send_waypoints.emplace_back(wp);
    }

    wp_count = send_waypoints.size();
    wp_end_id = wp_count;
    wp_cur_id = 0;
    restart_timeout_timer();

    lock.unlock();
    mission_count(wp_count);
    res->success = wait_push_all();
    lock.lock();

    res->wp_transfered = wp_cur_id + 1;
    go_idle();  // same as in pull_cb
  }

  void clear_cb(
    const mavros_msgs::srv::WaypointClear::Request::SharedPtr req [[maybe_unused]],
    mavros_msgs::srv::WaypointClear::Response::SharedPtr res)
  {
    unique_lock lock(mutex);

    if (wp_state != WP::IDLE) {
      return;
    }

    wp_state = WP::CLEAR;
    restart_timeout_timer();

    lock.unlock();
    mission_clear_all();
    res->success = wait_push_all();

    lock.lock();
    go_idle();  // same as in pull_cb
  }
};

}  // namespace std_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::RallypointPlugin)
