/*
 * Copyright 2014,2015,2016,2017,2018,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Waypoint plugin
 * @file waypoint.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "mavros/mission_protocol_base.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/srv/waypoint_set_current.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT

/**
 * @brief Mission manupulation plugin
 * @plugin waypoint
 */
class WaypointPlugin : public plugin::MissionBase
{
public:
  explicit WaypointPlugin(plugin::UASPtr uas_)
  : MissionBase(uas_, "mission"),
    enable_partial_push_auto(true)
  {
    rcl_interfaces::msg::ParameterDescriptor desc_pp{};
#ifndef USE_OLD_DECLARE_PARAMETER
    desc_pp.dynamic_typing = true;
#endif

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

    node_declare_and_watch_parameter(
      "enable_partial_push", 2, [&](const rclcpp::Parameter & p) {
        RCLCPP_DEBUG_STREAM(get_logger(), log_prefix << ": enable_partial_push = " << p);

        if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
          auto v = p.as_int();

          enable_partial_push_auto = v >= 2;
          if (enable_partial_push_auto) {
            enable_partial_push = detect_partial_push();
          } else {
            enable_partial_push = v != 0;
          }
        }

        if (p.get_type() == rclcpp::PARAMETER_BOOL) {
          enable_partial_push = p.as_bool();
        }
      }, desc_pp);

    auto wp_qos = rclcpp::QoS(10).transient_local();

    wp_list_pub = node->create_publisher<mavros_msgs::msg::WaypointList>("~/waypoints", wp_qos);
    wp_reached_pub = node->create_publisher<mavros_msgs::msg::WaypointReached>("~/reached", wp_qos);

    pull_srv =
      node->create_service<mavros_msgs::srv::WaypointPull>(
      "~/pull",
      std::bind(&WaypointPlugin::pull_cb, this, _1, _2));
    push_srv =
      node->create_service<mavros_msgs::srv::WaypointPush>(
      "~/push",
      std::bind(&WaypointPlugin::push_cb, this, _1, _2));
    clear_srv =
      node->create_service<mavros_msgs::srv::WaypointClear>(
      "~/clear",
      std::bind(&WaypointPlugin::clear_cb, this, _1, _2));
    set_cur_srv = node->create_service<mavros_msgs::srv::WaypointSetCurrent>(
      "~/set_current", std::bind(
        &WaypointPlugin::set_cur_cb, this, _1, _2));

    enable_connection_cb();
    enable_capabilities_cb();
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr wp_list_pub;
  rclcpp::Publisher<mavros_msgs::msg::WaypointReached>::SharedPtr wp_reached_pub;

  rclcpp::Service<mavros_msgs::srv::WaypointPull>::SharedPtr pull_srv;
  rclcpp::Service<mavros_msgs::srv::WaypointPush>::SharedPtr push_srv;
  rclcpp::Service<mavros_msgs::srv::WaypointClear>::SharedPtr clear_srv;
  rclcpp::Service<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr set_cur_srv;

  bool enable_partial_push_auto;

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

      if (enable_partial_push_auto) {
        enable_partial_push = detect_partial_push();

        RCLCPP_INFO_STREAM(
          get_logger(), log_prefix << ": detected enable_partial_push: " << enable_partial_push);
      }
    } else if (schedule_timer) {
      schedule_timer->cancel();
    }
  }

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
    wp_list_pub->publish(wpl);
  }

  void publish_reached(const uint16_t seq) override
  {
    auto wr = mavros_msgs::msg::WaypointReached();

    wr.header.stamp = node->now();
    wr.wp_seq = seq;

    wp_reached_pub->publish(wr);
  }

  bool detect_partial_push()
  {
    return uas->is_ardupilotmega();
  }

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
      // Partial Waypoint update

      if (!enable_partial_push) {
        RCLCPP_WARN(
          get_logger(), "%s: Partial Push not enabled. (Only supported on APM)", log_prefix);
        res->success = false;
        res->wp_transfered = 0;
        return;
      }

      if (waypoints.size() < req->start_index + req->waypoints.size()) {
        RCLCPP_WARN(get_logger(), "%s: Partial push out of range rejected.", log_prefix);
        res->success = false;
        res->wp_transfered = 0;
        return;
      }

      wp_state = WP::TXPARTIAL;
      send_waypoints = waypoints;

      uint16_t seq = req->start_index;
      for (auto & it : req->waypoints) {
        send_waypoints[seq++] = it;
      }

      wp_count = req->waypoints.size();
      wp_start_id = req->start_index;
      wp_end_id = req->start_index + wp_count;
      wp_cur_id = req->start_index;
      restart_timeout_timer();

      lock.unlock();
      mission_write_partial_list(wp_start_id, wp_end_id);
      res->success = wait_push_all();
      lock.lock();

      res->wp_transfered = wp_cur_id - wp_start_id + 1;
    } else {
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
    }

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

  void set_cur_cb(
    const mavros_msgs::srv::WaypointSetCurrent::Request::SharedPtr req,
    mavros_msgs::srv::WaypointSetCurrent::Response::SharedPtr res)
  {
    unique_lock lock(mutex);

    if (wp_state != WP::IDLE) {
      return;
    }

    wp_state = WP::SET_CUR;
    wp_set_active = req->wp_seq;
    restart_timeout_timer();

    lock.unlock();
    mission_set_current(wp_set_active);
    res->success = wait_push_all();

    lock.lock();
    go_idle();  // same as in pull_cb
  }
};

}  // namespace std_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::WaypointPlugin)
