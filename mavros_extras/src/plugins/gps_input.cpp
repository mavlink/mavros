/*
 * Copyright 2019 Amilcar Lucas.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief GPS_INPUT plugin
 * @file gps_input.cpp
 * @author Amilcar Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/gpsinput.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT
using mavlink::common::GPS_FIX_TYPE;
using mavlink::common::GPS_INPUT_IGNORE_FLAGS;

/**
 * @brief GPS_INPUT GPS plugin.
 * @plugin gps_input
 *
 * Sends <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">GPS_INPUT MAVLink messages</a>
 */
class GpsInputPlugin : public plugin::Plugin
{
public:
  explicit GpsInputPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "gps_input")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "gps_rate", 5.0, [&](const rclcpp::Parameter & p) {
        rclcpp::Rate rate(p.as_double());

        rate_period = rate.period();
      });

    gps_input_sub = node->create_subscription<mavros_msgs::msg::GPSINPUT>(
      "~/gps_input", 1, std::bind(
        &GpsInputPlugin::send_cb, this,
        _1));
  }

  Subscriptions get_subscriptions()
  {
    return { /* Rx disabled */};
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::GPSINPUT>::SharedPtr gps_input_sub;

  std::chrono::nanoseconds rate_period;
  rclcpp::Time last_pos_time;

  /* -*- callbacks -*- */

  /**
   * @brief Send GPS coordinates through GPS_INPUT Mavlink message
   */
  void send_cb(const mavros_msgs::msg::GPSINPUT::SharedPtr ros_msg)
  {
    auto now_ = node->now();

    // Throttle incoming messages
    if ((now_ - last_pos_time).to_chrono<std::chrono::nanoseconds>() < rate_period) {
      return;
    }
    last_pos_time = now_;

    /**
     * @note: <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">GPS_INPUT MAVLink message</a>
     * is currently only supported by Ardupilot firmware
     */
    mavlink::common::msg::GPS_INPUT gps_input {};

    // [[[cog:
    // import pymavlink.dialects.v20.common as common
    //
    // for field in common.MAVLink_gps_input_message.fieldnames:
    //     if field in ['time_usec']:
    //         continue
    //     cog.outl(f"gps_input.{field} = ros_msg->{field};")
    // ]]]
    gps_input.gps_id = ros_msg->gps_id;
    gps_input.ignore_flags = ros_msg->ignore_flags;
    gps_input.time_week_ms = ros_msg->time_week_ms;
    gps_input.time_week = ros_msg->time_week;
    gps_input.fix_type = ros_msg->fix_type;
    gps_input.lat = ros_msg->lat;
    gps_input.lon = ros_msg->lon;
    gps_input.alt = ros_msg->alt;
    gps_input.hdop = ros_msg->hdop;
    gps_input.vdop = ros_msg->vdop;
    gps_input.vn = ros_msg->vn;
    gps_input.ve = ros_msg->ve;
    gps_input.vd = ros_msg->vd;
    gps_input.speed_accuracy = ros_msg->speed_accuracy;
    gps_input.horiz_accuracy = ros_msg->horiz_accuracy;
    gps_input.vert_accuracy = ros_msg->vert_accuracy;
    gps_input.satellites_visible = ros_msg->satellites_visible;
    gps_input.yaw = ros_msg->yaw;
    // [[[end]]] (checksum: 303dffa9e430561ad0e254448d3f403a)

    gps_input.time_usec = get_time_usec(ros_msg->header.stamp);

    uas->send_message(gps_input);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::GpsInputPlugin)
