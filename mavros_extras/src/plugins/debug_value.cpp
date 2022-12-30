/*
 * Copyright 2017 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Debug messages plugin
 * @file debug_value.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>
#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/debug_value.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;  // NOLINT

/**
 * @brief Plugin for Debug msgs from MAVLink API
 * @plugin debug_value
 */
class DebugValuePlugin : public plugin::Plugin
{
public:
  explicit DebugValuePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "debug_value")
  {
    // subscribers
    debug_sub =
      node->create_subscription<DV>(
      "~/send", 10,
      std::bind(&DebugValuePlugin::debug_cb, this, _1));

    // publishers
    debug_pub = node->create_publisher<DV>("~/debug", 10);
    debug_vector_pub = node->create_publisher<DV>("~/debug_vector", 10);
    debug_float_array_pub = node->create_publisher<DV>("~/debug_float_array", 10);
    named_value_float_pub = node->create_publisher<DV>("~/named_value_float", 10);
    named_value_int_pub = node->create_publisher<DV>("~/named_value_int", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&DebugValuePlugin::handle_debug),
      make_handler(&DebugValuePlugin::handle_debug_vector),
      make_handler(&DebugValuePlugin::handle_debug_float_array),
      make_handler(&DebugValuePlugin::handle_named_value_float),
      make_handler(&DebugValuePlugin::handle_named_value_int)
    };
  }

private:
  using DV = mavros_msgs::msg::DebugValue;

  rclcpp::Subscription<DV>::SharedPtr debug_sub;

  rclcpp::Publisher<DV>::SharedPtr debug_pub;
  rclcpp::Publisher<DV>::SharedPtr debug_vector_pub;
  rclcpp::Publisher<DV>::SharedPtr debug_float_array_pub;
  rclcpp::Publisher<DV>::SharedPtr named_value_float_pub;
  rclcpp::Publisher<DV>::SharedPtr named_value_int_pub;

  /* -*- helpers -*- */

  /**
   * @brief Helper function to log debug messages
   * @param type    Type of debug message
   * @param dv      Data value
   */
  void debug_logger(const std::string & type, const DV & dv)
  {
    std::string name = (dv.name == "") ? "UNK" : dv.name;

    std::ostringstream ss;
    if (dv.type == DV::TYPE_NAMED_VALUE_INT) {
      ss << dv.value_int;
    } else if (dv.type == DV::TYPE_DEBUG_VECT) {
      ss << "[";
      bool is_first = true;
      for (auto v : dv.data) {
        if (!is_first) {
          ss << ", ";
        }

        ss << v;
        is_first = false;
      }

      ss << "]";
    } else {
      ss << dv.value_float;
    }

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      type << "\t" <<
        dv.header.stamp.sec << "." << dv.header.stamp.nanosec << "\t" <<
        name << "\t[" <<
        dv.index << "]\tvalue:" <<
        ss.str());
  }

  /* -*- message handlers -*- */

  /**
   * @brief Handle DEBUG message.
   * Message specification: https://mavlink.io/en/messages/common.html#DEBUG
   * @param msg     Received Mavlink msg
   * @param debug   DEBUG msg
   */
  void handle_debug(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::DEBUG & debug,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // [[[cog:
    // p = "dv_msg"
    // val = "debug"
    //
    // def common_filler(type_, time_f, index, name, disable_array_id = True):
    //     if isinstance(index, str):
    //         index = val + "." + index
    //
    //     cog.outl(f"""DV {p};""")
    //     cog.outl(f"""{p}.header.stamp = uas->synchronise_stamp({val}.{time_f});""")
    //     cog.outl(f"""{p}.type = DV::{type_};""")
    //     cog.outl(f"""{p}.index = {index};""")
    //     if disable_array_id:
    //         cog.outl(f"""{p}.array_id = -1;""")
    //     if name:
    //         cog.outl(f"""{p}.name = mavlink::to_string({val}.{name});""")
    //
    // common_filler("TYPE_DEBUG", "time_boot_ms", "ind", None)
    // cog.outl(f"""{p}.value_float = {val}.value;""")
    // ]]]
    DV dv_msg;
    dv_msg.header.stamp = uas->synchronise_stamp(debug.time_boot_ms);
    dv_msg.type = DV::TYPE_DEBUG;
    dv_msg.index = debug.ind;
    dv_msg.array_id = -1;
    dv_msg.value_float = debug.value;
    // [[[end]]] (checksum: ef695729241176edd2e06592ed20549b)

    debug_logger(debug.get_name(), dv_msg);
    debug_pub->publish(dv_msg);
  }

  /**
   * @brief Handle DEBUG_VECT message.
   * Message specification: https://mavlink.io/en/messages/common.html#DEBUG_VECT
   * @param msg     Received Mavlink msg
   * @param debug   DEBUG_VECT msg
   */
  void handle_debug_vector(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::DEBUG_VECT & debug,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // [[[cog:
    // common_filler("TYPE_DEBUG_VECT", "time_usec", -1, "name")
    //
    // fields = "xyz"
    // pd = p + ".data"
    // cog.outl(f"""{pd}.resize({len(fields)});""")
    // for i, f in enumerate(fields):
    //     cog.outl(f"""{pd}[{i}] = {val}.{f};""")
    // ]]]
    DV dv_msg;
    dv_msg.header.stamp = uas->synchronise_stamp(debug.time_usec);
    dv_msg.type = DV::TYPE_DEBUG_VECT;
    dv_msg.index = -1;
    dv_msg.array_id = -1;
    dv_msg.name = mavlink::to_string(debug.name);
    dv_msg.data.resize(3);
    dv_msg.data[0] = debug.x;
    dv_msg.data[1] = debug.y;
    dv_msg.data[2] = debug.z;
    // [[[end]]] (checksum: 8abb1284bdb29874a87fee9808570f05)

    debug_logger(debug.get_name(), dv_msg);
    debug_vector_pub->publish(dv_msg);
  }

  /**
   * @brief Handle DEBUG_FLOAT_ARRAY message.
   * Message specification: https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY
   * @param msg	Received Mavlink msg
   * @param debug	DEBUG_FLOAT_ARRAY msg
   */
  void handle_debug_float_array(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::DEBUG_FLOAT_ARRAY & debug,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // [[[cog:
    // common_filler("TYPE_DEBUG_FLOAT_ARRAY", "time_usec", -1, "name", False)
    //
    // cog.outl("{p}.array_id = {val}.array_id;".format(**locals()))
    // cog.outl("{p}.data.assign({val}.data.begin(), {val}.data.end());".format(**locals()))
    // ]]]
    DV dv_msg;
    dv_msg.header.stamp = uas->synchronise_stamp(debug.time_usec);
    dv_msg.type = DV::TYPE_DEBUG_FLOAT_ARRAY;
    dv_msg.index = -1;
    dv_msg.name = mavlink::to_string(debug.name);
    dv_msg.array_id = debug.array_id;
    dv_msg.data.assign(debug.data.begin(), debug.data.end());
    // [[[end]]] (checksum: e13bbba22bff5b74db32092d8787c6b4)

    debug_logger(debug.get_name(), dv_msg);
    debug_float_array_pub->publish(dv_msg);
  }

  /**
   * @brief Handle NAMED_VALUE_FLOAT message.
   * Message specification: https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT
   * @param msg     Received Mavlink msg
   * @param value   NAMED_VALUE_FLOAT msg
   */
  void handle_named_value_float(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::NAMED_VALUE_FLOAT & value,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // [[[cog:
    // val="value"
    // common_filler("TYPE_NAMED_VALUE_FLOAT", "time_boot_ms", -1, "name")
    // cog.outl(f"""{p}.value_float = {val}.value;""")
    // ]]]
    DV dv_msg;
    dv_msg.header.stamp = uas->synchronise_stamp(value.time_boot_ms);
    dv_msg.type = DV::TYPE_NAMED_VALUE_FLOAT;
    dv_msg.index = -1;
    dv_msg.array_id = -1;
    dv_msg.name = mavlink::to_string(value.name);
    dv_msg.value_float = value.value;
    // [[[end]]] (checksum: 8c243c3e607db7bf0758cd4ac3aca976)

    debug_logger(value.get_name(), dv_msg);
    named_value_float_pub->publish(dv_msg);
  }

  /**
   * @brief Handle NAMED_VALUE_INT message.
   * Message specification: https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT
   * @param msg     Received Mavlink msg
   * @param value   NAMED_VALUE_INT msg
   */
  void handle_named_value_int(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::NAMED_VALUE_INT & value,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // [[[cog:
    // common_filler("TYPE_NAMED_VALUE_INT", "time_boot_ms", -1, "name")
    // cog.outl(f"""{p}.value_int = {val}.value;""")
    // ]]]
    DV dv_msg;
    dv_msg.header.stamp = uas->synchronise_stamp(value.time_boot_ms);
    dv_msg.type = DV::TYPE_NAMED_VALUE_INT;
    dv_msg.index = -1;
    dv_msg.array_id = -1;
    dv_msg.name = mavlink::to_string(value.name);
    dv_msg.value_int = value.value;
    // [[[end]]] (checksum: 32cb48d5dad85c622997aeb6d34c255e)

    debug_logger(value.get_name(), dv_msg);
    named_value_int_pub->publish(dv_msg);
  }

  /* -*- callbacks -*- */

  /**
   * @brief Debug callbacks
   * @param req     pointer to mavros_msgs/Debug.msg being published
   */
  void debug_cb(const mavros_msgs::msg::DebugValue::SharedPtr req)
  {
    switch (req->type) {
      case DV::TYPE_DEBUG: {
          mavlink::common::msg::DEBUG debug {};

          debug.time_boot_ms = get_time_boot_ms(req->header.stamp);
          debug.ind = req->index;
          debug.value = req->value_float;

          uas->send_message(debug);
          break;
        }
      case DV::TYPE_DEBUG_VECT: {
          mavlink::common::msg::DEBUG_VECT debug {};

          debug.time_usec = get_time_usec(req->header.stamp);
          mavlink::set_string(debug.name, req->name);
          // [[[cog:
          // for i, f in enumerate("xyz"):
          //     cog.outl(f"debug.{f} = req->data[{i}];")
          // ]]]
          debug.x = req->data[0];
          debug.y = req->data[1];
          debug.z = req->data[2];
          // [[[end]]] (checksum: e3359b14c75adf35f430840dcf01ef18)

          uas->send_message(debug);
          break;
        }
      case DV::TYPE_DEBUG_FLOAT_ARRAY: {
          mavlink::common::msg::DEBUG_FLOAT_ARRAY debug {};

          debug.time_usec = get_time_usec(req->header.stamp);
          mavlink::set_string(debug.name, req->name);
          std::copy_n(
            req->data.begin(), std::min(req->data.size(), debug.data.size()),
            std::begin(debug.data));

          uas->send_message(debug);
          break;
        }
      case DV::TYPE_NAMED_VALUE_FLOAT: {
          mavlink::common::msg::NAMED_VALUE_FLOAT value {};

          value.time_boot_ms = get_time_boot_ms(req->header.stamp);
          mavlink::set_string(value.name, req->name);
          value.value = req->value_float;

          uas->send_message(value);
          break;
        }
      case DV::TYPE_NAMED_VALUE_INT: {
          mavlink::common::msg::NAMED_VALUE_INT value {};

          value.time_boot_ms = get_time_boot_ms(req->header.stamp);
          mavlink::set_string(value.name, req->name);
          value.value = req->value_int;

          uas->send_message(value);
          break;
        }
      default:
        RCLCPP_ERROR(get_logger(), "Wrong debug type (%d). Droping!...", req->type);
        return;
    }
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::DebugValuePlugin)
