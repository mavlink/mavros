/*
 * Copyright 2014,2015,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief RC IO plugin
 * @file rc_io.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <vector>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief RC IO plugin
 * @plugin rc_io
 */
class RCIOPlugin : public plugin::Plugin
{
public:
  explicit RCIOPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "rc"),
    raw_rc_in(0),
    raw_rc_out(0),
    has_rc_channels_msg(false)
  {
    rc_in_pub = node->create_publisher<mavros_msgs::msg::RCIn>("~/in", 10);
    rc_out_pub = node->create_publisher<mavros_msgs::msg::RCOut>("~/out", 10);
    override_sub = node->create_subscription<mavros_msgs::msg::OverrideRCIn>(
      "~/override", 10, std::bind(
        &RCIOPlugin::override_cb, this,
        _1));

    enable_connection_cb();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&RCIOPlugin::handle_rc_channels_raw),
      make_handler(&RCIOPlugin::handle_rc_channels),
      make_handler(&RCIOPlugin::handle_servo_output_raw),
    };
  }

private:
  using lock_guard = std::lock_guard<std::mutex>;
  std::mutex mutex;

  std::vector<uint16_t> raw_rc_in;
  std::vector<uint16_t> raw_rc_out;
  std::atomic<bool> has_rc_channels_msg;

  rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_in_pub;
  rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr rc_out_pub;
  rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr override_sub;

  /* -*- rx handlers -*- */

  void handle_rc_channels_raw(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::RC_CHANNELS_RAW & port,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    /* if we receive RC_CHANNELS, drop RC_CHANNELS_RAW */
    if (has_rc_channels_msg) {
      return;
    }

    lock_guard lock(mutex);

    size_t offset = port.port * 8;
    if (raw_rc_in.size() < offset + 8) {
      raw_rc_in.resize(offset + 8);
    }

    // [[[cog:
    // for i in range(1, 9):
    //     cog.outl(f"raw_rc_in[offset + {i - 1}] = port.chan{i}_raw;")
    // ]]]
    raw_rc_in[offset + 0] = port.chan1_raw;
    raw_rc_in[offset + 1] = port.chan2_raw;
    raw_rc_in[offset + 2] = port.chan3_raw;
    raw_rc_in[offset + 3] = port.chan4_raw;
    raw_rc_in[offset + 4] = port.chan5_raw;
    raw_rc_in[offset + 5] = port.chan6_raw;
    raw_rc_in[offset + 6] = port.chan7_raw;
    raw_rc_in[offset + 7] = port.chan8_raw;
    // [[[end]]] (checksum: 7ae5a061d1f05239433e9a78b4b1887a)

    auto rcin_msg = mavros_msgs::msg::RCIn();
    rcin_msg.header.stamp = uas->synchronise_stamp(port.time_boot_ms);
    rcin_msg.rssi = port.rssi;
    rcin_msg.channels = raw_rc_in;

    rc_in_pub->publish(rcin_msg);
  }

  void handle_rc_channels(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::RC_CHANNELS & channels,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    constexpr size_t MAX_CHANCNT = 18;
    lock_guard lock(mutex);

    RCLCPP_INFO_EXPRESSION(
      get_logger(), !has_rc_channels_msg.exchange(
        true), "RC_CHANNELS message detected!");

    if (channels.chancount > MAX_CHANCNT) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 60000,
        "FCU receives %u RC channels, but RC_CHANNELS can store %zu",
        channels.chancount, MAX_CHANCNT);

      channels.chancount = MAX_CHANCNT;
    }

    raw_rc_in.resize(channels.chancount);

    // switch works as start point selector.
    switch (channels.chancount) {
      // [[[cog:
      // for i in range(18, 0, -1):
      //     cog.outl(f"case {i}: raw_rc_in[{i - 1}] = channels.chan{i}_raw; [[fallthrough]];")
      // ]]]
      case 18: raw_rc_in[17] = channels.chan18_raw; [[fallthrough]];
      case 17: raw_rc_in[16] = channels.chan17_raw; [[fallthrough]];
      case 16: raw_rc_in[15] = channels.chan16_raw; [[fallthrough]];
      case 15: raw_rc_in[14] = channels.chan15_raw; [[fallthrough]];
      case 14: raw_rc_in[13] = channels.chan14_raw; [[fallthrough]];
      case 13: raw_rc_in[12] = channels.chan13_raw; [[fallthrough]];
      case 12: raw_rc_in[11] = channels.chan12_raw; [[fallthrough]];
      case 11: raw_rc_in[10] = channels.chan11_raw; [[fallthrough]];
      case 10: raw_rc_in[9] = channels.chan10_raw; [[fallthrough]];
      case 9: raw_rc_in[8] = channels.chan9_raw; [[fallthrough]];
      case 8: raw_rc_in[7] = channels.chan8_raw; [[fallthrough]];
      case 7: raw_rc_in[6] = channels.chan7_raw; [[fallthrough]];
      case 6: raw_rc_in[5] = channels.chan6_raw; [[fallthrough]];
      case 5: raw_rc_in[4] = channels.chan5_raw; [[fallthrough]];
      case 4: raw_rc_in[3] = channels.chan4_raw; [[fallthrough]];
      case 3: raw_rc_in[2] = channels.chan3_raw; [[fallthrough]];
      case 2: raw_rc_in[1] = channels.chan2_raw; [[fallthrough]];
      case 1: raw_rc_in[0] = channels.chan1_raw; [[fallthrough]];
      // [[[end]]] (checksum: d88127d1dce2f6b42244d905e5686cb3)
      case 0: break;
    }

    auto rcin_msg = mavros_msgs::msg::RCIn();

    rcin_msg.header.stamp = uas->synchronise_stamp(channels.time_boot_ms);
    rcin_msg.rssi = channels.rssi;
    rcin_msg.channels = raw_rc_in;

    rc_in_pub->publish(rcin_msg);
  }

  void handle_servo_output_raw(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SERVO_OUTPUT_RAW & port,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    lock_guard lock(mutex);

    uint8_t num_channels;

    // If using Mavlink protocol v2, number of available servo channels is 16;
    // otherwise, 8
    if (msg->magic == MAVLINK_STX) {
      num_channels = 16;
    } else {
      num_channels = 8;
    }

    size_t offset = port.port * num_channels;
    if (raw_rc_out.size() < offset + num_channels) {
      raw_rc_out.resize(offset + num_channels);
    }

    // [[[cog:
    // for i in range(1, 9):
    //     cog.outl(f"raw_rc_out[offset + {i - 1}] = port.servo{i}_raw;")
    // ]]]
    raw_rc_out[offset + 0] = port.servo1_raw;
    raw_rc_out[offset + 1] = port.servo2_raw;
    raw_rc_out[offset + 2] = port.servo3_raw;
    raw_rc_out[offset + 3] = port.servo4_raw;
    raw_rc_out[offset + 4] = port.servo5_raw;
    raw_rc_out[offset + 5] = port.servo6_raw;
    raw_rc_out[offset + 6] = port.servo7_raw;
    raw_rc_out[offset + 7] = port.servo8_raw;
    // [[[end]]] (checksum: 46b3fd22ff05ec22fed73852907b4f45)
    if (msg->magic == MAVLINK_STX) {
      // [[[cog:
      // for i in range(9, 17):
      //     cog.outl(f"raw_rc_out[offset + {i - 1}] = port.servo{i}_raw;")
      // ]]]
      raw_rc_out[offset + 8] = port.servo9_raw;
      raw_rc_out[offset + 9] = port.servo10_raw;
      raw_rc_out[offset + 10] = port.servo11_raw;
      raw_rc_out[offset + 11] = port.servo12_raw;
      raw_rc_out[offset + 12] = port.servo13_raw;
      raw_rc_out[offset + 13] = port.servo14_raw;
      raw_rc_out[offset + 14] = port.servo15_raw;
      raw_rc_out[offset + 15] = port.servo16_raw;
      // [[[end]]] (checksum: c008714176d3c498f792098d65557830)
    }

    auto rcout_msg = mavros_msgs::msg::RCOut();

    // XXX: Why time_usec is 32 bit? We should test that.
    uint64_t time_usec = port.time_usec;

    rcout_msg.header.stamp = uas->synchronise_stamp(time_usec);
    rcout_msg.channels = raw_rc_out;

    rc_out_pub->publish(rcout_msg);
  }

  /* -*- callbacks -*- */

  void connection_cb([[maybe_unused]] bool connected) override
  {
    lock_guard lock(mutex);
    raw_rc_in.clear();
    raw_rc_out.clear();
    has_rc_channels_msg = false;
  }

  void override_cb(const mavros_msgs::msg::OverrideRCIn::SharedPtr req)
  {
    if (!uas->is_ardupilotmega() && !uas->is_px4()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 30000, "RC override not supported by this FCU!");
    }

    mavlink::common::msg::RC_CHANNELS_OVERRIDE ovr = {};
    uas->msg_set_target(ovr);

    // [[[cog:
    // for i in range(1, 19):
    //     cog.outl(f"ovr.chan{i}_raw = req->channels[{i - 1}];")
    // ]]]
    ovr.chan1_raw = req->channels[0];
    ovr.chan2_raw = req->channels[1];
    ovr.chan3_raw = req->channels[2];
    ovr.chan4_raw = req->channels[3];
    ovr.chan5_raw = req->channels[4];
    ovr.chan6_raw = req->channels[5];
    ovr.chan7_raw = req->channels[6];
    ovr.chan8_raw = req->channels[7];
    ovr.chan9_raw = req->channels[8];
    ovr.chan10_raw = req->channels[9];
    ovr.chan11_raw = req->channels[10];
    ovr.chan12_raw = req->channels[11];
    ovr.chan13_raw = req->channels[12];
    ovr.chan14_raw = req->channels[13];
    ovr.chan15_raw = req->channels[14];
    ovr.chan16_raw = req->channels[15];
    ovr.chan17_raw = req->channels[16];
    ovr.chan18_raw = req->channels[17];
    // [[[end]]] (checksum: f878252945b6d1ca47c416b4f3aec145)

    uas->send_message(ovr);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::RCIOPlugin)
