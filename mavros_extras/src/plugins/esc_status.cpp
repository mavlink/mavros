/*
 * Copyright 2020 Ricardo Marques <marques.ricardo17@gmail.com>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief ESC status plugin
 * @file esc_status.cpp
 * @author Ricardo Marques <marques.ricardo17@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/esc_info.hpp"
#include "mavros_msgs/msg/esc_status.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief ESC status plugin
 * @plugin esc_status
 */
class ESCStatusPlugin : public plugin::Plugin
{
public:
  explicit ESCStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "esc_status"),
    _max_esc_count(0),
    _max_esc_info_index(0),
    _max_esc_status_index(0)
  {
    esc_info_pub = node->create_publisher<mavros_msgs::msg::ESCInfo>("~/info", 10);
    esc_status_pub = node->create_publisher<mavros_msgs::msg::ESCStatus>("~/status", 10);

    enable_connection_cb();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ESCStatusPlugin::handle_esc_info),
      make_handler(&ESCStatusPlugin::handle_esc_status),
    };
  }

private:
  using lock_guard = std::lock_guard<std::mutex>;

  rclcpp::Publisher<mavros_msgs::msg::ESCInfo>::SharedPtr esc_info_pub;
  rclcpp::Publisher<mavros_msgs::msg::ESCStatus>::SharedPtr esc_status_pub;

  std::mutex mutex;
  mavros_msgs::msg::ESCInfo _esc_info;
  mavros_msgs::msg::ESCStatus _esc_status;
  uint8_t _max_esc_count;
  uint8_t _max_esc_info_index;
  uint8_t _max_esc_status_index;
  const uint8_t batch_size = 4;

  void handle_esc_info(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ESC_INFO & esc_info,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    lock_guard lock(mutex);

    _esc_info.header.stamp = uas->synchronise_stamp(esc_info.time_usec);

    uint8_t esc_index = esc_info.index;

    _esc_info.counter = esc_info.counter;
    _esc_info.count = esc_info.count;
    _esc_info.connection_type = esc_info.connection_type;
    _esc_info.info = esc_info.info;

    if (_esc_info.count > _max_esc_count) {
      _max_esc_count = _esc_info.count;
    }

    if (_esc_info.esc_info.size() < _max_esc_count) {
      _esc_info.esc_info.resize(_max_esc_count);
    }

    for (int i = 0; i < std::min<ssize_t>(batch_size, ssize_t(_max_esc_count) - esc_index); i++) {
      _esc_info.esc_info[esc_index + i].header = _esc_info.header;
      _esc_info.esc_info[esc_index + i].failure_flags = esc_info.failure_flags[i];
      _esc_info.esc_info[esc_index + i].error_count = esc_info.error_count[i];
      _esc_info.esc_info[esc_index + i].temperature = esc_info.temperature[i] * 1E2;  //!< [degC]
    }

    _max_esc_info_index = std::max(_max_esc_info_index, esc_info.index);

    if (_max_esc_info_index == esc_info.index) {
      esc_info_pub->publish(_esc_info);
    }
  }

  void handle_esc_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ESC_STATUS & esc_status,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    lock_guard lock(mutex);

    uint8_t esc_index = esc_status.index;

    if (_esc_status.esc_status.size() < _max_esc_count) {
      _esc_status.esc_status.resize(_max_esc_count);
    }

    _esc_status.header.stamp = uas->synchronise_stamp(esc_status.time_usec);

    for (int i = 0; i < std::min<ssize_t>(batch_size, ssize_t(_max_esc_count) - esc_index); i++) {
      _esc_status.esc_status[esc_index + i].header = _esc_status.header;
      _esc_status.esc_status[esc_index + i].rpm = esc_status.rpm[i];
      _esc_status.esc_status[esc_index + i].voltage = esc_status.voltage[i];
      _esc_status.esc_status[esc_index + i].current = esc_status.current[i];
    }

    _max_esc_status_index = std::max(_max_esc_status_index, esc_status.index);

    if (_max_esc_status_index == esc_status.index) {
      esc_status_pub->publish(_esc_status);
    }
  }

  void connection_cb(bool connected [[maybe_unused]]) override
  {
    lock_guard lock(mutex);

    _max_esc_count = 0;
    _max_esc_status_index = 0;
    _max_esc_info_index = 0;
    _esc_info.esc_info.resize(0);
    _esc_status.esc_status.resize(0);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::ESCStatusPlugin)
