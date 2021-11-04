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
#include <vector>
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

using utils::enum_value;
using mavlink::common::MAV_CMD;
using mavlink::common::MAV_FRAME;
using mavlink::common::MAV_PROTOCOL_CAPABILITY;
using mavlink::common::msg::MISSION_ITEM;
using mavlink::common::msg::MISSION_ITEM_INT;
using MRES = mavlink::common::MAV_MISSION_RESULT;
using MTYPE = mavlink::common::MAV_MISSION_TYPE;
using MFilter = plugin::filter::SystemAndOk;

// [[[cog:
//
// from pymavlink.dialects.v20 import common
// e = common.enums['MAV_FRAME']
// all_names = [ee.name[len('MAV_FRAME_'):] for ee in e.values()]
// all_names.pop() # remove ENUM_END
// global_names = [v for v in all_names if v.startswith('GLOBAL')]
// local_names = [v for v in all_names if v.startswith(('LOCAL', 'BODY', 'MOCAP',
//     'VISION', 'ESTIM'))]
// other_names = ['MISSION']
//
// waypoint_item_msg = [(v, v) if isinstance(v, str) else v for v in (
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
// mission_item_msg = [
//     ('seq', 'seq'),
//     ('mission_type', 'mission_type'),
// ]
// ]]]
// [[[end]]] (checksum: d41d8cd98f00b204e9800998ecf8427e)

//! Thin wrapper for Waypoint message
class MissionItem : public mavros_msgs::msg::Waypoint
{
public:
  uint16_t seq;             //!< sequence number, not a part of ros message
  uint8_t mission_type;     //!< MAV_MISSION_TYPE

  static constexpr double encode_factor(const uint8_t frame)
  {
    switch (frame) {
      // [[[cog:
      // for names, factor in [(global_names, 10000000), (local_names, 10000), (other_names, 1)]:
      //      for name in names:
      //              cog.outl(f"case enum_value(MAV_FRAME::{name}):")
      //      cog.outl(f"  return {factor:1.1f};")
      //
      // cog.outl("default:\n  return 1.0;")
      // ]]]
      case enum_value(MAV_FRAME::GLOBAL):
      case enum_value(MAV_FRAME::GLOBAL_RELATIVE_ALT):
      case enum_value(MAV_FRAME::GLOBAL_INT):
      case enum_value(MAV_FRAME::GLOBAL_RELATIVE_ALT_INT):
      case enum_value(MAV_FRAME::GLOBAL_TERRAIN_ALT):
      case enum_value(MAV_FRAME::GLOBAL_TERRAIN_ALT_INT):
        return 10000000.0;
      case enum_value(MAV_FRAME::LOCAL_NED):
      case enum_value(MAV_FRAME::LOCAL_ENU):
      case enum_value(MAV_FRAME::LOCAL_OFFSET_NED):
      case enum_value(MAV_FRAME::BODY_NED):
      case enum_value(MAV_FRAME::BODY_OFFSET_NED):
      case enum_value(MAV_FRAME::BODY_FRD):
      case enum_value(MAV_FRAME::LOCAL_FRD):
      case enum_value(MAV_FRAME::LOCAL_FLU):
        return 10000.0;
      case enum_value(MAV_FRAME::MISSION):
        return 1.0;
      default:
        return 1.0;
        // [[[end]]] (checksum: 62e411e9acae54305a129fdf57f5b732)
    }
  }

  explicit MissionItem(const MISSION_ITEM & wpi)
  : mavros_msgs::msg::Waypoint()
  {
    // [[[cog:
    // fields = mission_item_msg + waypoint_item_msg + waypoint_coords
    // for i, (a, b) in enumerate(fields):
    //     cog.outl(f"{a} = wpi.{b};")
    // ]]]
    seq = wpi.seq;
    mission_type = wpi.mission_type;
    frame = wpi.frame;
    command = wpi.command;
    is_current = wpi.current;
    autocontinue = wpi.autocontinue;
    param1 = wpi.param1;
    param2 = wpi.param2;
    param3 = wpi.param3;
    param4 = wpi.param4;
    x_lat = wpi.x;
    y_long = wpi.y;
    z_alt = wpi.z;
    // [[[end]]] (checksum: 64a5d4f7b3428e8e72d5ce216d51935c)
  }

  explicit MissionItem(const MISSION_ITEM_INT & wpi)
  : mavros_msgs::msg::Waypoint()
  {
    // [[[cog:
    // fields = mission_item_msg + waypoint_item_msg + waypoint_coords
    // for i, (a, b) in enumerate(fields):
    //     if a.startswith(('x', 'y')):
    //         cog.outl(f"{a} = wpi.{b} / encode_factor(wpi.frame);")
    //     else:
    //         cog.outl(f"{a} = wpi.{b};")
    // ]]]
    seq = wpi.seq;
    mission_type = wpi.mission_type;
    frame = wpi.frame;
    command = wpi.command;
    is_current = wpi.current;
    autocontinue = wpi.autocontinue;
    param1 = wpi.param1;
    param2 = wpi.param2;
    param3 = wpi.param3;
    param4 = wpi.param4;
    x_lat = wpi.x / encode_factor(wpi.frame);
    y_long = wpi.y / encode_factor(wpi.frame);
    z_alt = wpi.z;
    // [[[end]]] (checksum: 2e01c028ecde023cc049aba04f6a6df5)
  }

  explicit MissionItem(const mavros_msgs::msg::Waypoint & other)
  {
    *this = other;
  }

  MissionItem & operator=(const mavros_msgs::msg::Waypoint & other)
  {
    // [[[cog:
    // fields = waypoint_item_msg + waypoint_coords
    // for a, b in fields:
    //     cog.outl(f"{a} = other.{a};")
    // ]]]
    frame = other.frame;
    command = other.command;
    is_current = other.is_current;
    autocontinue = other.autocontinue;
    param1 = other.param1;
    param2 = other.param2;
    param3 = other.param3;
    param4 = other.param4;
    x_lat = other.x_lat;
    y_long = other.y_long;
    z_alt = other.z_alt;
    // [[[end]]] (checksum: 6879b829d6e6e1e28a879dd2ca3afdac)

    return *this;
  }

  void to_msg(MISSION_ITEM & out) const
  {
    // [[[cog:
    // fields = mission_item_msg + waypoint_item_msg + waypoint_coords
    // for a, b in fields:
    //     cog.outl(f"out.{b} = {a};")
    // ]]]
    out.seq = seq;
    out.mission_type = mission_type;
    out.frame = frame;
    out.command = command;
    out.current = is_current;
    out.autocontinue = autocontinue;
    out.param1 = param1;
    out.param2 = param2;
    out.param3 = param3;
    out.param4 = param4;
    out.x = x_lat;
    out.y = y_long;
    out.z = z_alt;
    // [[[end]]] (checksum: 799ed1c97af022223371c42c8c1f09d4)
  }

  void to_msg(MISSION_ITEM_INT & out) const
  {
    // [[[cog:
    // fields = mission_item_msg + waypoint_item_msg + waypoint_coords
    // for a, b in fields:
    //     if b.startswith(('x', 'y')):
    //         cog.outl(f"out.{b} = int32_t({a} * encode_factor(frame));")
    //     else:
    //         cog.outl(f"out.{b} = {a};")
    // ]]]
    out.seq = seq;
    out.mission_type = mission_type;
    out.frame = frame;
    out.command = command;
    out.current = is_current;
    out.autocontinue = autocontinue;
    out.param1 = param1;
    out.param2 = param2;
    out.param3 = param3;
    out.param4 = param4;
    out.x = int32_t(x_lat * encode_factor(frame));
    out.y = int32_t(y_long * encode_factor(frame));
    out.z = z_alt;
    // [[[end]]] (checksum: b79ee415a09746786ba2a7cec99c5ab5)
  }

  friend std::ostream & operator<<(std::ostream & os, const MissionItem & mi);
};

std::ostream & operator<<(std::ostream & os, const MissionItem & mi);


/**
 * @brief Mission protocol base plugin
 */
class MissionBase : public plugin::Plugin
{
public:
  MissionBase(
    plugin::UASPtr uas_, const std::string & name_, MTYPE mission_type_ = MTYPE::MISSION,
    const char * log_prefix_ = "WP",
    const std::chrono::nanoseconds bootup_time_ = 15s)
  : Plugin(uas_, name_),
    mission_type(mission_type_),
    log_prefix(log_prefix_),
    wp_state(WP::IDLE),
    wp_count(0),
    wp_start_id(0),
    wp_end_id(0),
    wp_cur_id(0),
    wp_cur_active(0),
    wp_set_active(0),
    wp_retries(RETRIES_COUNT),
    is_timedout(false),
    reschedule_pull(false),
    do_pull_after_gcs(false),
    enable_partial_push(false),
    use_mission_item_int(false),
    mission_item_int_support_confirmed(false),
    BOOTUP_TIME(bootup_time_),
    LIST_TIMEOUT(30s),
    WP_TIMEOUT(1s),
    RESCHEDULE_TIME(5s)
  {
    timeout_timer = node->create_wall_timer(WP_TIMEOUT, std::bind(&MissionBase::timeout_cb, this));
    timeout_timer->cancel();
  }

  Subscriptions get_subscriptions() override
  {
    Subscriptions ret{
      make_handler(&MissionBase::handle_mission_item),
      make_handler(&MissionBase::handle_mission_item_int),
      make_handler(&MissionBase::handle_mission_request),
      make_handler(&MissionBase::handle_mission_request_int),
      make_handler(&MissionBase::handle_mission_count),
      make_handler(&MissionBase::handle_mission_ack),
    };

    // NOTE(vooon): those messages do not have mission_type and only needed for waypoint plugin
    if (mission_type == MTYPE::MISSION) {
      ret.push_back(make_handler(&MissionBase::handle_mission_current));
      ret.push_back(make_handler(&MissionBase::handle_mission_item_reached));
    }

    return ret;
  }

protected:
  using unique_lock = std::unique_lock<std::recursive_mutex>;
  using lock_guard = std::lock_guard<std::recursive_mutex>;

  std::recursive_mutex mutex;
  std::vector<MissionItem> waypoints;
  std::vector<MissionItem> send_waypoints;

  const MTYPE mission_type;
  const char * log_prefix;

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

  rclcpp::TimerBase::SharedPtr timeout_timer;
  rclcpp::TimerBase::SharedPtr schedule_timer;

  bool reschedule_pull;

  bool do_pull_after_gcs;
  bool enable_partial_push;
  bool use_mission_item_int;
  bool mission_item_int_support_confirmed;

  static constexpr int RETRIES_COUNT = 3;
  const std::chrono::nanoseconds BOOTUP_TIME;
  const std::chrono::nanoseconds LIST_TIMEOUT;
  const std::chrono::nanoseconds WP_TIMEOUT;
  const std::chrono::nanoseconds RESCHEDULE_TIME;

  /* -*- rx handlers -*- */

  /**
   * @brief filters messages not suitable for that plugin
   */
  template<class MsgT>
  bool filter_message(const MsgT & m)
  {
    return m.mission_type != enum_value(mission_type);
  }

  /**
   * @brief checks for a sequence mismatch between a
   * MISSION_REQUEST(_INT) sequence and the current
   * waypoint that should be sent.
   * @param seq	The seq member of a MISSION_REQUEST(_INT)
   * @return		True if there is a sequence mismatch
   */
  template<class MsgT>
  bool sequence_mismatch(const MsgT & m)
  {
    if (m.seq != wp_cur_id && m.seq != wp_cur_id + 1) {
      RCLCPP_WARN(
        get_logger(), "%s: Seq mismatch, dropping %s (%d != %zu)", log_prefix,
        m.get_name().c_str(), m.seq, wp_cur_id);
      return true;
    }

    return false;
  }

  /**
   * @brief handle MISSION_ITEM mavlink msg
   * handles and stores mission items when pulling waypoints
   * @param msg     Received Mavlink msg
   * @param wpi     WaypointItem from msg
   */
  virtual void handle_mission_item(
    const mavlink::mavlink_message_t * msg,
    MISSION_ITEM & wpi,
    MFilter filter);

  /**
   * @brief handle MISSION_ITEM_INT mavlink msg
   * handles and stores mission items when pulling waypoints
   * @param msg     Received Mavlink msg
   * @param wpi     WaypointItemInt from msg
   */
  virtual void handle_mission_item_int(
    const mavlink::mavlink_message_t * msg,
    MISSION_ITEM_INT & wpi,
    MFilter filter);

  /**
   * @brief handle MISSION_REQUEST mavlink msg
   * handles and acts on misison request from FCU
   * @param msg     Received Mavlink msg
   * @param mreq    MISSION_REQUEST from msg
   */
  virtual void handle_mission_request(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::MISSION_REQUEST & mreq,
    MFilter filter);

  /**
   * @brief handle MISSION_REQUEST_INT mavlink msg
   * handles and acts on misison request from FCU
   * @param msg     Received Mavlink msg
   * @param mreq    MISSION_REQUEST_INT from msg
   */
  virtual void handle_mission_request_int(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::MISSION_REQUEST_INT & mreq,
    MFilter filter);

  /**
   * @brief handle MISSION_COUNT mavlink msg
   * Handles a mission count from FCU in a Waypoint Pull
   * Triggers a pull GCS seems to be requesting mission
   * @param msg     Received Mavlink msg
   * @param mcnt    MISSION_COUNT from msg
   */
  virtual void handle_mission_count(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::MISSION_COUNT & mcnt,
    MFilter filter);

  /**
   * @brief handle MISSION_ACK mavlink msg
   * Handles a MISSION_ACK which marks the end of a push, or a failure
   * @param msg     Received Mavlink msg
   * @param mack    MISSION_ACK from msg
   */
  virtual void handle_mission_ack(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::MISSION_ACK & mack,
    MFilter filter);

  /**
   * @brief handle MISSION_CURRENT mavlink msg
   * This confirms a SET_CUR action
   * @param msg     Received Mavlink msg
   * @param mcur    MISSION_CURRENT from msg
   */
  virtual void handle_mission_current(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::MISSION_CURRENT & mcur,
    MFilter filter);

  /**
   * @brief handle MISSION_ITEM_REACHED mavlink msg
   * @param msg     Received Mavlink msg
   * @param mitr    MISSION_ITEM_REACHED from msg
   */
  virtual void handle_mission_item_reached(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::MISSION_ITEM_REACHED & mitr,
    MFilter filter);

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

    // run once
    schedule_timer->cancel();

    if (wp_state != WP::IDLE) {
      /* try later */
      RCLCPP_DEBUG(get_logger(), "%s: busy, reschedule pull", log_prefix);
      schedule_pull(RESCHEDULE_TIME);
      return;
    }

    RCLCPP_DEBUG(get_logger(), "%s: start scheduled pull", log_prefix);
    wp_state = WP::RXLIST;
    wp_count = 0;
    restart_timeout_timer();
    mission_request_list();
  }

  //! @brief Send ACK back to FCU after pull
  void request_mission_done(void)
  {
    /* possibly not needed if count == 0 (QGC impl) */
    mission_ack(MRES::ACCEPTED);

    go_idle();
    list_receiving.notify_all();
    RCLCPP_INFO(get_logger(), "%s: mission received", log_prefix);
  }

  void go_idle(void)
  {
    reschedule_pull = false;
    wp_state = WP::IDLE;
    timeout_timer->cancel();
  }

  void restart_timeout_timer(void)
  {
    wp_retries = RETRIES_COUNT;
    restart_timeout_timer_int();
  }

  void restart_timeout_timer_int(void)
  {
    is_timedout = false;
    timeout_timer->reset();
  }

  void schedule_pull(const std::chrono::nanoseconds & dt)
  {
    if (schedule_timer) {
      schedule_timer->cancel();
      schedule_timer.reset();
    }

    schedule_timer = node->create_wall_timer(dt, std::bind(&MissionBase::scheduled_pull_cb, this));
  }

  //! @brief send a single waypoint to FCU
  template<class MsgT>
  void send_waypoint(size_t seq)
  {
    static_assert(
      std::is_same<MsgT, MISSION_ITEM>::value || std::is_same<MsgT,
      MISSION_ITEM_INT>::value, "wrong type");

    if (seq < send_waypoints.size()) {
      MsgT wpi{};

      auto wp_msg = send_waypoints.at(seq);
      wp_msg.seq = seq;
      wp_msg.mission_type = enum_value(mission_type);
      wp_msg.to_msg(wpi);

      RCLCPP_DEBUG_STREAM(get_logger(), log_prefix << ": send item " << wp_msg);
      mission_send(wpi);
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
    size_t i = 0;
    for (auto & it : waypoints) {
      it.is_current = !!(i++ == seq);
    }
  }

  //! @brief publish the updated waypoint list after operation
  virtual void publish_waypoints() = 0;

  //! @brief publish mission item reached seq
  virtual void publish_reached(const uint16_t seq) = 0;

  /* -*- low-level send functions -*- */

  template<class MsgT>
  void mission_send(MsgT & wpi)
  {
    static_assert(
      std::is_same<MsgT, MISSION_ITEM>::value || std::is_same<MsgT,
      MISSION_ITEM_INT>::value, "wrong type");

    uas->msg_set_target(wpi);
    uas->send_message(wpi);
  }

  void mission_request(const uint16_t seq);
  void mission_request_int(const uint16_t seq);
  void mission_set_current(const uint16_t seq);
  void mission_request_list();
  void mission_count(const uint16_t cnt);
  void mission_write_partial_list(const uint16_t start_index, const uint16_t end_index);
  void mission_clear_all();
  void mission_ack(const MRES type);
};

}       // namespace plugin
}       // namespace mavros

#endif  // MAVROS__MISSION_PROTOCOL_BASE_HPP_
