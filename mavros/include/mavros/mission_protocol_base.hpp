/*
 * Copyright 2014,2015,2016,2017,2018,2021 Vladimir Ermakov.
 * Copyright 2021 Charlie Burge.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Mission base plugin
 * @file mission_protocol_base.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Charlie Burge <charlieburge@yahoo.com>
 *
 * @addtogroup plugin
 * @{
 */

#pragma once

#ifndef MAVROS__MISSION_PROTOCOL_BASE_HPP_
#define MAVROS__MISSION_PROTOCOL_BASE_HPP_

#include <chrono>
#include <sstream>
#include <iomanip>
#include <string>
#include <condition_variable>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"
#include "mavros_msgs/srv/waypoint_pull.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"

namespace mavros
{
namespace plugin
{
using namespace std::placeholders;          // NOLINT
using namespace std::chrono_literals;       // NOLINT

using mavlink::common::MAV_CMD;
using mavlink::common::MAV_FRAME;
using MRES = mavlink::common::MAV_MISSION_RESULT;
using utils::enum_value;
using mavlink::common::MAV_FRAME;
using WP_ITEM = mavlink::common::msg::MISSION_ITEM;
using WP_ITEM_INT = mavlink::common::msg::MISSION_ITEM_INT;
using WP_TYPE = mavlink::common::MAV_MISSION_TYPE;
using Filter = plugin::filter::SystemAndOk;

// [[[cog:
//
// from pymavlink.dialects.v20 import common
// e = common.enums['MAV_FRAME']
// all_names = [ee.name[len('MAV_FRAME_'):] for ee in e.values()]
// all_names.pop() # remove ENUM_END
// global_names = [v for v in all_names if v.startswith('GLOBAL')]
// local_names = [v for v in all_names if v.startswith(('LOCAL', 'BODY', 'MOCAP', 'VISION', 'ESTIM'))]
// other_names = ['MISSION']
//
// waypoint_item_msg = [(v, v) if isinstance(v, str) else v for v in (
//     'seq',
//     'frame',
//     'command',
//     ('is_current', 'current'),
//     'autocontinue',
//     'param1',
//     'param2',
//     'param3',
//     'param4',
// )]
// waypoint_coords = [
//     ('x_lat', 'x'),
//     ('y_long', 'y'),
//     ('z_alt', 'z'),
// ]
// ]]]
// [[[end]]] (checksum: d41d8cd98f00b204e9800998ecf8427e)


//! Thin wrapper for Waypoint message
class WaypointItem : public mavros_msgs::msg::Waypoint
{
public:
  //! sequence number, not a part of ros message
  uint16_t seq;

  static constexpr double encode_factor(const uint8_t frame)
  {
    switch (frame) {
      // [[[cog:
      // for names, factor in [(global_names, 10000000), (local_names, 10000), (other_names, 1)]:
      //      for name in names:
      //              cog.outl(f"case enum_value(MAV_FRAME::{name}):")
      //      cog.outl(f"  return {factor};")
      //
      // cog.outl("default:\n  return 1;")
      // ]]]
      case enum_value(MAV_FRAME::GLOBAL):
      case enum_value(MAV_FRAME::GLOBAL_RELATIVE_ALT):
      case enum_value(MAV_FRAME::GLOBAL_INT):
      case enum_value(MAV_FRAME::GLOBAL_RELATIVE_ALT_INT):
      case enum_value(MAV_FRAME::GLOBAL_TERRAIN_ALT):
      case enum_value(MAV_FRAME::GLOBAL_TERRAIN_ALT_INT):
        return 10000000;
      case enum_value(MAV_FRAME::LOCAL_NED):
      case enum_value(MAV_FRAME::LOCAL_ENU):
      case enum_value(MAV_FRAME::LOCAL_OFFSET_NED):
      case enum_value(MAV_FRAME::BODY_NED):
      case enum_value(MAV_FRAME::BODY_OFFSET_NED):
      case enum_value(MAV_FRAME::BODY_FRD):
      case enum_value(MAV_FRAME::LOCAL_FRD):
      case enum_value(MAV_FRAME::LOCAL_FLU):
        return 10000;
      case enum_value(MAV_FRAME::MISSION):
        return 1;
      default:
        return 1;
        // [[[end]]] (checksum: 1e7f1d387a168ae02c6d21bad66fbf30)
    }
  }

  template<class ITEM>
  explicit WaypointItem(const ITEM & mav_msg)
  : Waypoint()
    // [[[cog:
    // for a, b in waypoint_item_msg + waypoint_coords:
    //     cog.outl(f", {a}(mav_msg.{b})")
    // ]]]
    , frame(mav_msg.frame),
    command(mav_msg.command),
    is_current(mav_msg.current),
    autocontinue(mav_msg.autocontinue),
    param1(mav_msg.param1),
    param2(mav_msg.param2),
    param3(mav_msg.param3),
    param4(mav_msg.param4),
    x_lat(mav_msg.x),
    y_long(mav_msg.y),
    z_alt(mav_msg.z)
    // [[[end]]] (checksum: 1596cd95b1f835bb12444147fdfd83e2)
  {}

  template< >
  explicit WaypointItem(const WP_ITEM_INT & mav_msg)
  : Waypoint()
    // [[[cog:
    // for a, b in waypoint_item_msg + waypoint_coords:
    //     if a.startswith(('x', 'y')):
    //         cog.outl(f", {a}(mav_msg.{b} / encode_factor(mav_msg.frame))")
    //     else:
    //         cog.outl(f", {a}(mav_msg.{b})")
    // ]]]
    , frame(mav_msg.frame),
    command(mav_msg.command),
    is_current(mav_msg.current),
    autocontinue(mav_msg.autocontinue),
    param1(mav_msg.param1),
    param2(mav_msg.param2),
    param3(mav_msg.param3),
    param4(mav_msg.param4),
    x_lat(mav_msg.x / encode_factor(mav_msg.frame)),
    y_long(mav_msg.y / encode_factor(mav_msg.frame)),
    z_alt(mav_msg.z)
    // [[[end]]] (checksum: 7c467f038381eb9ef718723c223c497c)
  {}

  template<class ITEM>
  ITEM to_msg(const uint16_t seq, const WP_TYPE type = WP_TYPE::MISSION) const
  {
    ITEM ret {};

    // [[[cog:
    // for a, b in waypoint_item_msg + waypoint_coords:
    //     cog.outl(f"ret.{b} = {a};")
    // ]]]
    ret.frame = wp.frame;
    ret.command = wp.command;
    ret.current = wp.is_current;
    ret.autocontinue = wp.autocontinue;
    ret.param1 = wp.param1;
    ret.param2 = wp.param2;
    ret.param3 = wp.param3;
    ret.param4 = wp.param4;
    ret.x = wp.x_lat;
    ret.y = wp.y_long;
    ret.z = wp.z_alt;
    // [[[end]]] (checksum: d121a97a1214456f845cab6f5b66e317)

    ret.seq = seq;
    ret.mission_type = enum_value(type);

    return ret;
  }

  template< >
  WP_ITEM_INT to_msg(const uint16_t seq, const WP_TYPE type = WP_TYPE::MISSION) const
  {
    WP_ITEM_INT ret {};

    // [[[cog:
    // for a, b in waypoint_item_msg + waypoint_coords:
    //     if b.startswith(('x', 'y')):
    //         cog.outl(f"ret.{b} = int32_t({a} * waypoint_encode_factor(frame));")
    //     else:
    //         cog.outl(f"ret.{b} = {a};")
    // ]]]
    ret.frame = wp.frame;
    ret.command = wp.command;
    ret.current = wp.is_current;
    ret.autocontinue = wp.autocontinue;
    ret.param1 = wp.param1;
    ret.param2 = wp.param2;
    ret.param3 = wp.param3;
    ret.param4 = wp.param4;
    ret.x = int32_t(wp.x_lat * waypoint_encode_factor(wp.frame));
    ret.y = int32_t(wp.y_long * waypoint_encode_factor(wp.frame));
    ret.z = wp.z_alt;
    // [[[end]]] (checksum: be915294cea6806c5897a17259983877)

    ret.seq = seq;
    ret.mission_type = enum_value(type);

    return ret;
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss.precision(7);
    ss << '#' << seq << (current ? '*' : ' ') << " F:" << frame << " C:" <<
      std::setw(3) << command;
    ss << " p: " << param1 << ' ' << param2 << ' ' << param3 << ' ' << param4 <<
      " x: " << x_lat << " y: " << y_long << " z: " << z_alt;
    return ss.str();
  }

};


/**
 * @brief Mission protocol base plugin
 */
class MissionBase : public plugin::Plugin
{
public:
  MissionBase(
    plugin::UASPtr uas_, const std::string & name_,
    const std::chorno::nanoseconds bootup_time_ = 15s)
  : Plugin(uas_, name_),
    wp_state(WP::IDLE),
    wp_type(WP_TYPE::MISSION),
    wp_count(0),
    wp_retries(RETRIES_COUNT),
    wp_cur_id(0),
    wp_cur_active(0),
    wp_set_active(0),
    is_timedout(false),
    do_pull_after_gcs(false),
    enable_partial_push(false),
    reschedule_pull(false),
    BOOTUP_TIME(bootup_time_),
    LIST_TIMEOUT(30s),
    WP_TIMEOUT(1s),
    RESCHEDULE_DT(5s)
  {
    wp_timer = node->create_wall_timer(WP_TIMEOUT, std::bind(&MissionBase::timeout_cb, this, _1));
    wp_timer->cancel();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&MissionBase::handle_mission_item),
      make_handler(&MissionBase::handle_mission_item_int),
      make_handler(&MissionBase::handle_mission_request),
      make_handler(&MissionBase::handle_mission_request_int),
      make_handler(&MissionBase::handle_mission_count),
      make_handler(&MissionBase::handle_mission_ack),
      make_handler(&MissionBase::handle_mission_current),
      make_handler(&MissionBase::handle_mission_item_reached),
    };
  }

protected:
  using unique_lock = std::unique_lock<std::recursive_mutex>;
  using lock_guard = std::lock_guard<std::recursive_mutex>;

  std::recursive_mutex mutex;
  std::vector<mavros_msgs::Waypoint> waypoints;
  std::vector<mavros_msgs::Waypoint> send_waypoints;

  enum class WP
  {
    IDLE,
    RXLIST,
    RXWP,
    RXWPINT,
    TXLIST,
    TXPARTIAL,
    TXWP,
    TXWPINT,
    CLEAR,
    SET_CUR
  };
  WP wp_state;

  WP_TYPE wp_type;
  size_t wp_count;
  size_t wp_start_id;
  size_t wp_end_id;
  size_t wp_cur_id;
  size_t wp_cur_active;
  size_t wp_set_active;
  size_t wp_retries;
  bool is_timedout;
  std::mutex recv_cond_mutex;
  std::mutex send_cond_mutex;
  std::condition_variable list_receiving;
  std::condition_variable list_sending;

  rclcpp::TimerBase::SharedPtr wp_timer;
  rclcpp::TimerBase::SharedPtr schedule_timer;
  bool do_pull_after_gcs;
  bool enable_partial_push;

  bool reschedule_pull;

  bool use_mission_item_int;
  bool mission_item_int_support_confirmed;

  static constexpr int RETRIES_COUNT = 3;
  static constexpr unsigned int MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4;

  const std::chrono::nanoseconds BOOTUP_TIME;
  const std::chrono::nanoseconds LIST_TIMEOUT;
  const std::chrono::nanoseconds WP_TIMEOUT;
  const std::chrono::nanoseconds RESCHEDULE_TIME;

  /* -*- rx handlers -*- */

  /**
   * @brief handle MISSION_ITEM_INT mavlink msg
   * handles and stores mission items when pulling waypoints
   * @param msg		Received Mavlink msg
   * @param wpi		WaypointItemInt from msg
   */
  virtual void handle_mission_item_int(const mavlink::mavlink_message_t * msg [[maybe_unused]],
    WP_ITEM_INT & wpi,
    Filter filter [[maybe_unused]]);


  /**
   * @brief handle MISSION_ITEM mavlink msg
   * handles and stores mission items when pulling waypoints
   * @param msg		Received Mavlink msg
   * @param wpi		WaypointItem from msg
   */
  virtual void handle_mission_item(const mavlink::mavlink_message_t * msg, WP_ITEM & wpi,
    Filter filter [[maybe_unused]]);

  /**
   * @brief checks for a sequence mismatch between a
   * MISSION_REQUEST(_INT) sequence and the current
   * waypoint that should be sent.
   * @param seq	The seq member of a MISSION_REQUEST(_INT)
   * @return		True if there is a sequence mismatch
   */
  virtual bool sequence_mismatch(const uint16_t & seq);

  /**
   * @brief handle MISSION_REQUEST mavlink msg
   * handles and acts on misison request from FCU
   * @param msg		Received Mavlink msg
   * @param mreq		MISSION_REQUEST from msg
   */
  virtual void handle_mission_request(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::MISSION_REQUEST & mreq, Filter filter [[maybe_unused]]);

  /**
   * @brief handle MISSION_REQUEST_INT mavlink msg
   * handles and acts on misison request from FCU
   * @param msg		Received Mavlink msg
   * @param mreq		MISSION_REQUEST_INT from msg
   */
  virtual void handle_mission_request_int(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::MISSION_REQUEST_INT & mreq, Filter filter [[maybe_unused]]);

  /**
   * @brief handle MISSION_COUNT mavlink msg
   * Handles a mission count from FCU in a Waypoint Pull
   * Triggers a pull GCS seems to be requesting mission
   * @param msg		Received Mavlink msg
   * @param mcnt		MISSION_COUNT from msg
   */
  virtual void handle_mission_count(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::MISSION_COUNT & mcnt, Filter filter [[maybe_unused]]);

  /**
   * @brief handle MISSION_ACK mavlink msg
   * Handles a MISSION_ACK which marks the end of a push, or a failure
   * @param msg		Received Mavlink msg
   * @param mack		MISSION_ACK from msg
   */
  virtual void handle_mission_ack(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::MISSION_ACK & mack, Filter filter [[maybe_unused]]);

  /* -*- mid-level helpers -*- */

  /**
   * @brief Act on a timeout
   * Resend the message that may have been lost
   */
  void timeout_cb();

  //! @brief Callback for scheduled waypoint pull
  void scheduled_pull_cb()
  {
    lock_guard lock(mutex);
    if (wp_state != WP::IDLE) {
      /* try later */
      RCLCPP_DEBUG(get_logger(), "MP: busy, reschedule pull");
      schedule_pull(RESCHEDULE_TIME);
      return;
    }

    RCLCPP_DEBUG(get_logger(), "MP: start scheduled pull");
    wp_state = WP::RXLIST;
    wp_count = 0;
    restart_timeout_timer();
    mission_request_list();

    // run once
    shedule_timer->cancel();
  }

  //! @brief Send ACK back to FCU after pull
  void request_mission_done(void)
  {
    /* possibly not needed if count == 0 (QGC impl) */
    mission_ack(MRES::ACCEPTED);

    go_idle();
    list_receiving.notify_all();
    RCLCPP_INFO(get_logger(), "MP: mission received");
  }

  void go_idle(void)
  {
    reschedule_pull = false;
    wp_state = WP::IDLE;
    wp_timer->cancel();
  }

  void restart_timeout_timer(void)
  {
    wp_retries = RETRIES_COUNT;
    restart_timeout_timer_int();
  }

  void restart_timeout_timer_int(void)
  {
    is_timedout = false;
    wp_timer->reset();
  }

  void schedule_pull(const std::chrono::nanoseconds & dt)
  {
    if (schedule_timer) {
      shedule_timer->cancel();
      shedule_timer.reset();
    }

    shedule_timer =
      node->create_wall_timer(dt, std::bind(&MissionBase::scheduled_pull_cb, this, _1));
  }

  //! @brief send a single waypoint to FCU
  template<class ITEM>
  void send_waypoint(size_t seq)
  {
    if (seq < send_waypoints.size()) {
      auto wp_msg = send_waypoints.at(seq);
      auto wpi = mav_from_msg<ITEM>(wp_msg, seq, wp_type);
      mission_send(wpi);
      RCLCPP_DEBUG_STREAM(get_logger(), "MP: send item " << waypoint_to_string<ITEM>(wpi));
    }
  }

  /**
   * @brief wait until a waypoint pull is complete.
   * Pull happens asynchronously, this function blocks until it is done.
   */
  bool wait_fetch_all()
  {
    std::unique_lock<std::mutex> lock(recv_cond_mutex);

    return list_receiving.wait_for(lock, LIST_TIMEOUT) == std::cv_status::no_timeout &&
           !is_timedout;
  }

  /**
   * @brief wait until a waypoint push is complete.
   * Push happens asynchronously, this function blocks until it is done.
   */
  bool wait_push_all()
  {
    std::unique_lock<std::mutex> lock(send_cond_mutex);

    return list_sending.wait_for(lock, LIST_TIMEOUT) == std::cv_status::no_timeout &&
           !is_timedout;
  }

  //! @brief set the FCU current waypoint
  void set_current_waypoint(size_t seq)
  {
    auto i = 0;
    for (auto & it : waypoints) {
      it.is_current = (i == seq) ? true : false;
      i++;
    }
  }

  //! @brief publish the updated waypoint list after operation
  virtual void publish_waypoints() = 0;

  /* -*- low-level send functions -*- */

  template<class ITEM>
  void mission_send(const ITEM & wp)
  {
    auto wpi = wp;
    uas->msg_set_target(wpi);
    uas->send_message(wpi);
  }

  void mission_request(const uint16_t seq)
  {
    RCLCPP_DEBUG(get_logger(), "MP:m: request #%u", seq);

    mavlink::common::msg::MISSION_REQUEST mrq {};
    uas->msg_set_target(mrq);
    mrq.seq = seq;
    mrq.mission_type = enum_value(wp_type);

    uas->send_message(mrq);
  }

  void mission_request_int(const uint16_t seq)
  {
    RCLCPP_DEBUG(get_logger(), "MP:m: request_int #%u", seq);

    mavlink::common::msg::MISSION_REQUEST_INT mrq {};
    uas->msg_set_target(mrq);
    mrq.seq = seq;
    mrq.mission_type = enum_value(wp_type);

    uas->send_message(mrq);
  }

  void mission_set_current(const uint16_t seq)
  {
    RCLCPP_DEBUG(get_logger(), "MP:m: set current #%u", seq);

    mavlink::common::msg::MISSION_SET_CURRENT msc {};
    uas->msg_set_target(msc);
    msc.seq = seq;
    // msc.mission_type = enum_value(wp_type);

    uas->send_message(msc);
  }

  void mission_request_list()
  {
    RCLCPP_DEBUG(get_logger(), "MP:m: request list");

    mavlink::common::msg::MISSION_REQUEST_LIST mrl {};
    uas->msg_set_target(mrl);
    mrl.mission_type = enum_value(wp_type);

    uas->send_message(mrl);
  }

  void mission_count(const uint16_t cnt)
  {
    RCLCPP_DEBUG(get_logger(), "MP:m: count %u", cnt);

    mavlink::common::msg::MISSION_COUNT mcnt {};
    uas->msg_set_target(mcnt);
    mcnt.count = cnt;
    mcnt.mission_type = enum_value(wp_type);

    uas->send_message(mcnt);
  }

  void mission_write_partial_list(const uint16_t start_index, const uint16_t end_index)
  {
    RCLCPP_DEBUG(
      get_logger(), "MP:m: write partial list %u - %u",
      start_index, end_index);

    mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST mwpl {};
    uas->msg_set_target(mwpl);
    mwpl.start_index = start_index;
    mwpl.end_index = end_index;
    mwpl.mission_type = enum_value(wp_type);

    uas->send_message(mwpl);
  }

  void mission_clear_all()
  {
    RCLCPP_DEBUG(get_logger(), "MP:m: clear all");

    mavlink::common::msg::MISSION_CLEAR_ALL mclr {};
    uas->msg_set_target(mclr);
    mclr.mission_type = enum_value(wp_type);

    uas->send_message(mclr);
  }

  void mission_ack(const MRES type)
  {
    RCLCPP_DEBUG(get_logger(), "MP:m: ACK %u", enum_value(type));

    mavlink::common::msg::MISSION_ACK mack {};
    uas->msg_set_target(mack);
    mack.type = enum_value(type);
    mack.mission_type = enum_value(wp_type);

    uas->send_message(mack);
  }
};

}       // namespace plugin
}       // namespace mavros

#endif  // MAVROS__MISSION_PROTOCOL_BASE_HPP_
