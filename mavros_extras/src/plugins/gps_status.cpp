/**
 * @brief GPS status plugin
 * @file gps_status.cpp
 * @author Amilcar Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Ardupilot.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/gpsraw.hpp"
#include "mavros_msgs/msg/gpsrtk.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT
using mavlink::common::RTK_BASELINE_COORDINATE_SYSTEM;

/**
 * @brief Mavlink GPS status plugin.
 * @plugin gps_status
 *
 * This plugin publishes GPS sensor data from a Mavlink compatible FCU to ROS.
 */
class GpsStatusPlugin : public plugin::Plugin
{
public:
  explicit GpsStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "gpsstatus")
  {
    gps1_raw_pub = node->create_publisher<mavros_msgs::msg::GPSRAW>("~/gps1/raw", 10);
    gps2_raw_pub = node->create_publisher<mavros_msgs::msg::GPSRAW>("~/gps2/raw", 10);
    gps1_rtk_pub = node->create_publisher<mavros_msgs::msg::GPSRTK>("~/gps1/rtk", 10);
    gps2_rtk_pub = node->create_publisher<mavros_msgs::msg::GPSRTK>("~/gps2/rtk", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&GpsStatusPlugin::handle_gps_raw_int),
      make_handler(&GpsStatusPlugin::handle_gps2_raw),
      make_handler(&GpsStatusPlugin::handle_gps_rtk),
      make_handler(&GpsStatusPlugin::handle_gps2_rtk)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::GPSRAW>::SharedPtr gps1_raw_pub;
  rclcpp::Publisher<mavros_msgs::msg::GPSRAW>::SharedPtr gps2_raw_pub;
  rclcpp::Publisher<mavros_msgs::msg::GPSRTK>::SharedPtr gps1_rtk_pub;
  rclcpp::Publisher<mavros_msgs::msg::GPSRTK>::SharedPtr gps2_rtk_pub;

  /* -*- callbacks -*- */
  /**
   * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS_RAW_INT">mavlink GPS_RAW_INT message</a> into the gps1/raw topic.
   */
  void handle_gps_raw_int(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GPS_RAW_INT & mav_msg,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto ros_msg = mavros_msgs::msg::GPSRAW();

    // [[[cog:
    // import pymavlink.dialects.v20.common as common
    //
    // def outl_raw_msg(msg):
    //     for field in msg.fieldnames:
    //         if field in ['time_usec']:
    //             continue
    //         cog.outl(f"ros_msg.{field} = mav_msg.{field};")
    //
    // outl_raw_msg(common.MAVLink_gps_raw_int_message)
    // ]]]
    ros_msg.fix_type = mav_msg.fix_type;
    ros_msg.lat = mav_msg.lat;
    ros_msg.lon = mav_msg.lon;
    ros_msg.alt = mav_msg.alt;
    ros_msg.eph = mav_msg.eph;
    ros_msg.epv = mav_msg.epv;
    ros_msg.vel = mav_msg.vel;
    ros_msg.cog = mav_msg.cog;
    ros_msg.satellites_visible = mav_msg.satellites_visible;
    ros_msg.alt_ellipsoid = mav_msg.alt_ellipsoid;
    ros_msg.h_acc = mav_msg.h_acc;
    ros_msg.v_acc = mav_msg.v_acc;
    ros_msg.vel_acc = mav_msg.vel_acc;
    ros_msg.hdg_acc = mav_msg.hdg_acc;
    ros_msg.yaw = mav_msg.yaw;
    // [[[end]]] (checksum: 5803a4026c6f569e7cc00b66156640f9)
    ros_msg.header = uas->synchronized_header("/wgs84", mav_msg.time_usec);
    ros_msg.dgps_numch = UINT8_MAX;     // information not available in GPS_RAW_INT mavlink message
    ros_msg.dgps_age = UINT32_MAX;      // information not available in GPS_RAW_INT mavlink message

    gps1_raw_pub->publish(ros_msg);
  }

  /**
   * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS2_RAW">mavlink GPS2_RAW message</a> into the gps2/raw topic.
   */
  void handle_gps2_raw(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GPS2_RAW & mav_msg,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto ros_msg = mavros_msgs::msg::GPSRAW();

    // [[[cog:
    // outl_raw_msg(common.MAVLink_gps2_raw_message)
    // ]]]
    ros_msg.fix_type = mav_msg.fix_type;
    ros_msg.lat = mav_msg.lat;
    ros_msg.lon = mav_msg.lon;
    ros_msg.alt = mav_msg.alt;
    ros_msg.eph = mav_msg.eph;
    ros_msg.epv = mav_msg.epv;
    ros_msg.vel = mav_msg.vel;
    ros_msg.cog = mav_msg.cog;
    ros_msg.satellites_visible = mav_msg.satellites_visible;
    ros_msg.dgps_numch = mav_msg.dgps_numch;
    ros_msg.dgps_age = mav_msg.dgps_age;
    ros_msg.yaw = mav_msg.yaw;
    ros_msg.alt_ellipsoid = mav_msg.alt_ellipsoid;
    ros_msg.h_acc = mav_msg.h_acc;
    ros_msg.v_acc = mav_msg.v_acc;
    ros_msg.vel_acc = mav_msg.vel_acc;
    ros_msg.hdg_acc = mav_msg.hdg_acc;
    // [[[end]]] (checksum: 1d71e875394bf6abb6c46e39801cdc19)
    ros_msg.header = uas->synchronized_header("/wgs84", mav_msg.time_usec);

    gps2_raw_pub->publish(ros_msg);
  }

  template<typename MMsg>
  mavros_msgs::msg::GPSRTK convert_rtk(MMsg mav_msg)
  {
    auto ros_msg = mavros_msgs::msg::GPSRTK();

    std::string frame_id = "unknown";
    switch (static_cast<RTK_BASELINE_COORDINATE_SYSTEM>(mav_msg.baseline_coords_type)) {
      case RTK_BASELINE_COORDINATE_SYSTEM::ECEF:
        frame_id = "earth";
        break;
      case RTK_BASELINE_COORDINATE_SYSTEM::NED:
        frame_id = "map";
        break;
      default:
        RCLCPP_ERROR(
          get_logger(),
          "GPS_RTK.baseline_coords_type MAVLink field has unknown \"%d\" value",
          mav_msg.baseline_coords_type);
    }

    ros_msg.header = uas->synchronized_header(
      frame_id,
      mav_msg.time_last_baseline_ms * 1000);

    ros_msg.rtk_receiver_id = mav_msg.rtk_receiver_id;
    ros_msg.wn = mav_msg.wn;
    ros_msg.tow = mav_msg.tow;
    ros_msg.rtk_health = mav_msg.rtk_health;
    ros_msg.rtk_rate = mav_msg.rtk_rate;
    ros_msg.nsats = mav_msg.nsats;
    ros_msg.baseline_a = mav_msg.baseline_a_mm;
    ros_msg.baseline_b = mav_msg.baseline_b_mm;
    ros_msg.baseline_c = mav_msg.baseline_c_mm;
    ros_msg.accuracy = mav_msg.accuracy;
    ros_msg.iar_num_hypotheses = mav_msg.iar_num_hypotheses;

    return ros_msg;
  }

  /**
   * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS_RTK">mavlink GPS_RTK message</a> into the gps1/rtk topic.
   */
  void handle_gps_rtk(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GPS_RTK & mav_msg,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    gps1_rtk_pub->publish(convert_rtk(mav_msg));
  }

  /**
   * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS2_RTK">mavlink GPS2_RTK message</a> into the gps2/rtk topic.
   */
  void handle_gps2_rtk(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GPS2_RTK & mav_msg,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    gps2_rtk_pub->publish(convert_rtk(mav_msg));
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::GpsStatusPlugin)
