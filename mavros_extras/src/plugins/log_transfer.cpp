/*
 * Copyright 2018 mlvov <mlvov@cnord.ru>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Log Transfer plugin
 * @file log_transfer.cpp
 * @author mlvov <mlvov@cnord.ru>
 *
 * @addtogroup plugin
 * @{
 */

#include <atomic>
#include <algorithm>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/log_data.hpp"
#include "mavros_msgs/msg/log_entry.hpp"
#include "mavros_msgs/srv/log_request_data.hpp"
#include "mavros_msgs/srv/log_request_end.hpp"
#include "mavros_msgs/srv/log_request_list.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Log Transfer plugin
 * @plugin log_transfer
 */
class LogTransferPlugin : public plugin::Plugin
{
public:
  explicit LogTransferPlugin(plugin::UASPtr uas_)
  : plugin::Plugin(uas_, "log_transfer")
  {
    log_entry_pub = node->create_publisher<mavros_msgs::msg::LogEntry>("~/raw/log_entry", 1000);
    log_data_pub = node->create_publisher<mavros_msgs::msg::LogData>("~/raw/log_data", 1000);

    log_request_list_srv = node->create_service<mavros_msgs::srv::LogRequestList>(
      "~/raw/log_request_list", std::bind(&LogTransferPlugin::log_request_list_cb, this, _1, _2));
    log_request_data_srv = node->create_service<mavros_msgs::srv::LogRequestData>(
      "~/raw/log_request_data", std::bind(&LogTransferPlugin::log_request_data_cb, this, _1, _2));
    log_request_end_srv = node->create_service<mavros_msgs::srv::LogRequestEnd>(
      "~/raw/log_request_end", std::bind(&LogTransferPlugin::log_request_end_cb, this, _1, _2));
    log_request_erase_srv = node->create_service<std_srvs::srv::Trigger>(
      "~/raw/log_request_erase", std::bind(
        &LogTransferPlugin::log_request_erase_cb, this, _1,
        _2));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&LogTransferPlugin::handle_log_entry),
      make_handler(&LogTransferPlugin::handle_log_data),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::LogEntry>::SharedPtr log_entry_pub;
  rclcpp::Publisher<mavros_msgs::msg::LogData>::SharedPtr log_data_pub;

  rclcpp::Service<mavros_msgs::srv::LogRequestList>::SharedPtr log_request_list_srv;
  rclcpp::Service<mavros_msgs::srv::LogRequestData>::SharedPtr log_request_data_srv;
  rclcpp::Service<mavros_msgs::srv::LogRequestEnd>::SharedPtr log_request_end_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr log_request_erase_srv;

  void handle_log_entry(
    const mavlink::mavlink_message_t * mmsg [[maybe_unused]],
    mavlink::common::msg::LOG_ENTRY & le,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto msg = mavros_msgs::msg::LogEntry();

    msg.header.stamp = node->now();
    msg.id = le.id;
    msg.num_logs = le.num_logs;
    msg.last_log_num = le.last_log_num;
    msg.time_utc = rclcpp::Time(le.time_utc);
    msg.size = le.size;

    log_entry_pub->publish(msg);
  }

  void handle_log_data(
    const mavlink::mavlink_message_t * mmsg [[maybe_unused]],
    mavlink::common::msg::LOG_DATA & ld,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto msg = mavros_msgs::msg::LogData();

    msg.header.stamp = node->now();
    msg.id = ld.id;
    msg.offset = ld.ofs;

    auto count = std::min<size_t>(ld.count, ld.data.max_size());
    msg.data.insert(msg.data.cbegin(), ld.data.cbegin(), ld.data.cbegin() + count);

    log_data_pub->publish(msg);
  }

  void log_request_list_cb(
    const mavros_msgs::srv::LogRequestList::Request::SharedPtr req,
    mavros_msgs::srv::LogRequestList::Response::SharedPtr res)
  {
    mavlink::common::msg::LOG_REQUEST_LIST msg = {};

    uas->msg_set_target(msg);
    msg.start = req->start;
    msg.end = req->end;

    uas->send_message(msg);

    // NOTE(vooon): with ROS2 router it's not possible to detect drops
    res->success = true;
  }

  void log_request_data_cb(
    mavros_msgs::srv::LogRequestData::Request::SharedPtr req,
    mavros_msgs::srv::LogRequestData::Response::SharedPtr res)
  {
    mavlink::common::msg::LOG_REQUEST_DATA msg = {};

    uas->msg_set_target(msg);
    msg.id = req->id;
    msg.ofs = req->offset;
    msg.count = req->count;

    uas->send_message(msg);

    // NOTE(vooon): with ROS2 router it's not possible to detect drops
    res->success = true;
  }

  void log_request_end_cb(
    mavros_msgs::srv::LogRequestEnd::Request::SharedPtr req [[maybe_unused]],
    mavros_msgs::srv::LogRequestEnd::Response::SharedPtr res)
  {
    mavlink::common::msg::LOG_REQUEST_END msg = {};

    uas->msg_set_target(msg);

    uas->send_message(msg);

    // NOTE(vooon): with ROS2 router it's not possible to detect drops
    res->success = true;
  }

  void log_request_erase_cb(
    std_srvs::srv::Trigger::Request::SharedPtr req [[maybe_unused]],
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    mavlink::common::msg::LOG_ERASE msg{};

    uas->msg_set_target(msg);

    uas->send_message(msg);

    // NOTE(vooon): with ROS2 router it's not possible to detect drops
    res->success = true;
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::LogTransferPlugin)
