/*
 * Copyright 2018 Alexis Paques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief GPS RTK plugin
 * @file gps_rtk.cpp
 * @author Alexis Paques <alexis.paques@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/rtcm.hpp"
#include "mavros_msgs/msg/rtk_baseline.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief GPS RTK plugin
 * @plugin gps_rtk
 *
 * 1. Publish the RTCM messages from ROS to the FCU
 * 2. Publish RTK baseline data from the FCU to ROS
 */
class GpsRtkPlugin : public plugin::Plugin
{
public:
  explicit GpsRtkPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "gps_rtk")
  {
    gps_rtk_sub =
      node->create_subscription<mavros_msgs::msg::RTCM>(
      "~/send_rtcm", 10,
      std::bind(&GpsRtkPlugin::rtcm_cb, this, _1));

    // TODO(vooon): set QoS for latched topic
    rtk_baseline_pub = node->create_publisher<mavros_msgs::msg::RTKBaseline>("~/rtk_baseline", 1);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&GpsRtkPlugin::handle_baseline_msg)
    };
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr gps_rtk_sub;
  rclcpp::Publisher<mavros_msgs::msg::RTKBaseline>::SharedPtr rtk_baseline_pub;

  mavros_msgs::msg::RTKBaseline rtk_baseline_;
  std::atomic_uint rtcm_seq;

  /* -*- callbacks -*- */

  /**
   * @brief Handle mavros_msgs::RTCM message
   * It converts the message to the MAVLink GPS_RTCM_DATA message for GPS injection.
   * Message specification: https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA
   * @param msg		Received ROS msg
   */
  void rtcm_cb(const mavros_msgs::msg::RTCM::SharedPtr msg)
  {
    mavlink::common::msg::GPS_RTCM_DATA rtcm_data = {};
    const size_t max_frag_len = rtcm_data.data.size();

    uint8_t seq_u5 = uint8_t(rtcm_seq.fetch_add(1) & 0x1F) << 3;

    if (msg->data.size() > 4 * max_frag_len) {
      RCLCPP_ERROR(
        get_logger(),
        "gps_rtk: RTCM message received is bigger than the maximal possible size.");
      return;
    }

    auto data_it = msg->data.begin();
    auto end_it = msg->data.end();

    if (msg->data.size() <= max_frag_len) {
      rtcm_data.len = msg->data.size();
      rtcm_data.flags = seq_u5;
      std::copy(data_it, end_it, rtcm_data.data.begin());
      std::fill(rtcm_data.data.begin() + rtcm_data.len, rtcm_data.data.end(), 0);
      uas->send_message(rtcm_data);
    } else {
      for (uint8_t fragment_id = 0; fragment_id < 4 && data_it < end_it; fragment_id++) {
        uint8_t len = std::min(static_cast<size_t>(std::distance(data_it, end_it)), max_frag_len);
        rtcm_data.flags = 1;                    // LSB set indicates message is fragmented
        rtcm_data.flags |= fragment_id << 1;    // Next 2 bits are fragment id
        rtcm_data.flags |= seq_u5;              // Next 5 bits are sequence id
        rtcm_data.len = len;

        std::copy(data_it, data_it + len, rtcm_data.data.begin());
        std::fill(rtcm_data.data.begin() + len, rtcm_data.data.end(), 0);
        uas->send_message(rtcm_data);
        std::advance(data_it, len);
      }
    }
  }

  /* MAvlink msg handlers */
  /**
   * @brief Publish GPS_RTK message (MAvlink Common) received from FCU.
   * The message is already decoded by Mavlink, we only need to convert to ROS.
   * Details and units: https://mavlink.io/en/messages/common.html#GPS_RTK
   */

  void handle_baseline_msg(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GPS_RTK & rtk_bsln,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // [[[cog:
    // import pymavlink.dialects.v20.common as common
    //
    // for field in common.MAVLink_gps_rtk_message.fieldnames:
    //     if field in ['time_usec']:
    //         continue
    //     cog.outl(f"rtk_baseline_.{field} = rtk_bsln.{field};")
    // ]]]
    rtk_baseline_.time_last_baseline_ms = rtk_bsln.time_last_baseline_ms;
    rtk_baseline_.rtk_receiver_id = rtk_bsln.rtk_receiver_id;
    rtk_baseline_.wn = rtk_bsln.wn;
    rtk_baseline_.tow = rtk_bsln.tow;
    rtk_baseline_.rtk_health = rtk_bsln.rtk_health;
    rtk_baseline_.rtk_rate = rtk_bsln.rtk_rate;
    rtk_baseline_.nsats = rtk_bsln.nsats;
    rtk_baseline_.baseline_coords_type = rtk_bsln.baseline_coords_type;
    rtk_baseline_.baseline_a_mm = rtk_bsln.baseline_a_mm;
    rtk_baseline_.baseline_b_mm = rtk_bsln.baseline_b_mm;
    rtk_baseline_.baseline_c_mm = rtk_bsln.baseline_c_mm;
    rtk_baseline_.accuracy = rtk_bsln.accuracy;
    rtk_baseline_.iar_num_hypotheses = rtk_bsln.iar_num_hypotheses;
    // [[[end]]] (checksum: c123d29c2e0bce3becce956a29ed6152)
    rtk_baseline_.header = uas->synchronized_header("", rtk_bsln.time_last_baseline_ms * 1000);

    rtk_baseline_pub->publish(rtk_baseline_);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::GpsRtkPlugin)
