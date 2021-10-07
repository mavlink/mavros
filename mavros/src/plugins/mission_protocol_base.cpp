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
 * @file mission_protocol_base.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Charlie Burge <charlieburge@yahoo.com>
 */

#include "mavros/mission_protocol_base.hpp"

using namespace mavros;          // NOLINT
using namespace mavros::plugin;  // NOLINT


std::ostream & mavros::plugin::operator<<(std::ostream & os, const MissionItem & mi)
{
  os << '#' << mi.seq << (mi.is_current ? '*' : ' ') << " F:" << +mi.frame << " C:" <<
    std::setw(3) << mi.command;
  os << std::setprecision(7) << " p: " << mi.param1 << ' ' << mi.param2 << ' ' << mi.param3 <<
    ' ' << mi.param4;
  os << std::setprecision(7) << " x: " << mi.x_lat << " y: " << mi.y_long << " z: " << mi.z_alt;
  return os;
}

void MissionBase::handle_mission_item(
  const mavlink::mavlink_message_t * msg [[maybe_unused]],
  MISSION_ITEM & wpi,
  MFilter filter [[maybe_unused]])
{
  unique_lock lock(mutex);

  if (filter_message(wpi)) {
    return;
  }

  // receive item only in RX state
  if (wp_state == WP::RXWP) {
    if (sequence_mismatch(wpi)) {
      return;
    }

    auto it = waypoints.emplace(waypoints.end(), wpi);
    RCLCPP_INFO_STREAM(get_logger(), log_prefix << ": item " << *it);

    if (++wp_cur_id < wp_count) {
      restart_timeout_timer();
      mission_request(wp_cur_id);
    } else {
      request_mission_done();
      lock.unlock();
      publish_waypoints();
    }
  } else {
    RCLCPP_DEBUG(
      get_logger(), "%s: rejecting item, wrong state %d", log_prefix, enum_value(
        wp_state));
    if (do_pull_after_gcs && reschedule_pull) {
      RCLCPP_DEBUG(get_logger(), "%s: reschedule pull", log_prefix);
      schedule_pull(WP_TIMEOUT);
    }
  }
}

void MissionBase::handle_mission_item_int(
  const mavlink::mavlink_message_t * msg [[maybe_unused]],
  MISSION_ITEM_INT & wpi,
  MFilter filter [[maybe_unused]])
{
  unique_lock lock(mutex);

  if (filter_message(wpi)) {
    return;
  }

  // receive item only in RX state
  if (wp_state == WP::RXWPINT) {
    if (sequence_mismatch(wpi)) {
      return;
    }

    auto it = waypoints.emplace(waypoints.end(), wpi);
    RCLCPP_INFO_STREAM(get_logger(), log_prefix << ": item " << *it);

    if (++wp_cur_id < wp_count) {
      restart_timeout_timer();
      mission_request_int(wp_cur_id);
    } else {
      request_mission_done();
      mission_item_int_support_confirmed = true;
      lock.unlock();
      publish_waypoints();
    }
  } else {
    RCLCPP_DEBUG(
      get_logger(), "%s: rejecting item, wrong state %d", log_prefix, enum_value(
        wp_state));
    if (do_pull_after_gcs && reschedule_pull) {
      RCLCPP_DEBUG(get_logger(), "%s: reschedule pull", log_prefix);
      schedule_pull(WP_TIMEOUT);
    }
  }
}

void MissionBase::handle_mission_request(
  const mavlink::mavlink_message_t * msg [[maybe_unused]],
  mavlink::common::msg::MISSION_REQUEST & mreq,
  MFilter filter [[maybe_unused]])
{
  lock_guard lock(mutex);

  if (filter_message(mreq)) {
    return;
  }

  if (
    (wp_state == WP::TXLIST && mreq.seq == 0) ||
    (wp_state == WP::TXPARTIAL && mreq.seq == wp_start_id) || (wp_state == WP::TXWP) ||
    (wp_state == WP::TXWPINT))
  {
    if (sequence_mismatch(mreq)) {
      return;
    }

    restart_timeout_timer();
    if (mreq.seq < wp_end_id) {
      RCLCPP_DEBUG(
        get_logger(), "%s: FCU requested MISSION_ITEM waypoint %d", log_prefix,
        mreq.seq);
      wp_cur_id = mreq.seq;
      if (use_mission_item_int) {
        RCLCPP_DEBUG(get_logger(), "%s: Trying to send a MISSION_ITEM_INT instead", log_prefix);
        wp_state = WP::TXWPINT;
        send_waypoint<MISSION_ITEM_INT>(wp_cur_id);
      } else {
        wp_state = WP::TXWP;
        send_waypoint<MISSION_ITEM>(wp_cur_id);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "%s: FCU require seq out of range", log_prefix);
    }
  } else {
    RCLCPP_DEBUG(
      get_logger(), "%s: rejecting request, wrong state %d", log_prefix,
      enum_value(wp_state));
  }
}

void MissionBase::handle_mission_request_int(
  const mavlink::mavlink_message_t * msg [[maybe_unused]],
  mavlink::common::msg::MISSION_REQUEST_INT & mreq,
  MFilter filter [[maybe_unused]])
{
  lock_guard lock(mutex);

  if (filter_message(mreq)) {
    return;
  }

  if (
    (wp_state == WP::TXLIST && mreq.seq == 0) ||
    (wp_state == WP::TXPARTIAL && mreq.seq == wp_start_id) || (wp_state == WP::TXWPINT))
  {
    if (sequence_mismatch(mreq)) {
      return;
    }

    if (!use_mission_item_int) {
      use_mission_item_int = true;
    }
    if (!mission_item_int_support_confirmed) {
      mission_item_int_support_confirmed = true;
    }

    restart_timeout_timer();
    if (mreq.seq < wp_end_id) {
      RCLCPP_DEBUG(
        get_logger(), "%s: FCU reqested MISSION_ITEM_INT waypoint %d", log_prefix, mreq.seq);
      wp_state = WP::TXWPINT;
      wp_cur_id = mreq.seq;
      send_waypoint<MISSION_ITEM_INT>(wp_cur_id);
    } else {
      RCLCPP_ERROR(get_logger(), "%s: FCU require seq out of range", log_prefix);
    }
  } else {
    RCLCPP_DEBUG(
      get_logger(), "%s: rejecting request, wrong state %d", log_prefix,
      enum_value(wp_state));
  }
}

void MissionBase::handle_mission_count(
  const mavlink::mavlink_message_t * msg [[maybe_unused]],
  mavlink::common::msg::MISSION_COUNT & mcnt,
  MFilter filter [[maybe_unused]])
{
  unique_lock lock(mutex);

  if (filter_message(mcnt)) {
    return;
  }

  if (wp_state == WP::RXLIST) {
    // FCU report of MISSION_REQUEST_LIST
    RCLCPP_DEBUG(get_logger(), "%s: count %d", log_prefix, mcnt.count);

    wp_count = mcnt.count;
    wp_cur_id = 0;

    waypoints.clear();
    waypoints.reserve(wp_count);

    if (wp_count > 0) {
      if (use_mission_item_int) {
        wp_state = WP::RXWPINT;
        restart_timeout_timer();
        mission_request_int(wp_cur_id);
      } else {
        wp_state = WP::RXWP;
        restart_timeout_timer();
        mission_request(wp_cur_id);
      }
    } else {
      request_mission_done();
      lock.unlock();
      publish_waypoints();
    }
  } else {
    RCLCPP_INFO(get_logger(), "%s: seems GCS requesting mission", log_prefix);
    // schedule pull after GCS done
    if (do_pull_after_gcs) {
      RCLCPP_INFO(get_logger(), "%s: scheduling pull after GCS is done", log_prefix);
      reschedule_pull = true;
      schedule_pull(RESCHEDULE_TIME);
    }
  }
}

void MissionBase::handle_mission_ack(
  const mavlink::mavlink_message_t * msg [[maybe_unused]],
  mavlink::common::msg::MISSION_ACK & mack,
  MFilter filter [[maybe_unused]])
{
  unique_lock lock(mutex);

  auto ack_type = static_cast<MRES>(mack.type);

  if (filter_message(mack)) {
    return;
  }

  auto is_tx_done = [&]() -> bool {
      return
        (
        wp_state == WP::TXLIST ||
        wp_state == WP::TXPARTIAL ||
        wp_state == WP::TXWP ||
        wp_state == WP::TXWPINT
        ) &&
        (wp_cur_id == wp_end_id - 1) &&
        (ack_type == MRES::ACCEPTED)
      ;
    };

  auto is_tx_seq_error = [&]() -> bool {
      return
        (
        wp_state == WP::TXWP ||
        wp_state == WP::TXWPINT
        ) &&
        ack_type == MRES::INVALID_SEQUENCE
      ;
    };

  auto is_tx_failed = [&]() -> bool {
      return
        wp_state == WP::TXLIST ||
        wp_state == WP::TXPARTIAL ||
        wp_state == WP::TXWP ||
        wp_state == WP::TXWPINT
      ;
    };

  if (is_tx_done()) {
    go_idle();
    waypoints = send_waypoints;
    send_waypoints.clear();

    if (wp_state == WP::TXWPINT) {
      mission_item_int_support_confirmed = true;
    }

    lock.unlock();
    list_sending.notify_all();
    publish_waypoints();

    RCLCPP_INFO(get_logger(), "%s: mission sended", log_prefix);
  } else if (is_tx_seq_error()) {
    // Mission Ack: INVALID_SEQUENCE received during TXWP
    // This happens when waypoint N was received by autopilot,
    // but the request for waypoint N+1 failed.
    // This causes seq mismatch, ignore and eventually the request for n+1
    // will get to us and seq will sync up.
    RCLCPP_DEBUG(get_logger(), "%s: Received INVALID_SEQUENCE ack", log_prefix);
  } else if (is_tx_failed()) {
    go_idle();
    // use this flag for failure report
    is_timedout = true;
    lock.unlock();
    list_sending.notify_all();

    RCLCPP_ERROR_STREAM(
      get_logger(), log_prefix << ": upload failed: " << utils::to_string(
        ack_type));
  } else if (wp_state == WP::CLEAR) {
    go_idle();
    if (ack_type != MRES::ACCEPTED) {
      is_timedout = true;
      lock.unlock();
      RCLCPP_ERROR_STREAM(
        get_logger(),
        log_prefix << ": clear failed: " << utils::to_string(ack_type));
    } else {
      waypoints.clear();
      lock.unlock();
      publish_waypoints();
      RCLCPP_INFO(get_logger(), "%s: mission cleared", log_prefix);
    }

    list_sending.notify_all();
  } else {
    RCLCPP_DEBUG(get_logger(), "%s: not planned ACK, type: %d", log_prefix, mack.type);
  }
}

void MissionBase::handle_mission_current(
  const mavlink::mavlink_message_t * msg [[maybe_unused]],
  mavlink::common::msg::MISSION_CURRENT & mcur,
  MFilter filter [[maybe_unused]])
{
  unique_lock lock(mutex);

  // NOTE(vooon): this message does not have mission_type
  // if (filter_message(mcur)) {
  //   return;
  // }

  if (wp_state == WP::SET_CUR) {
    /* MISSION_SET_CURRENT ACK */
    RCLCPP_DEBUG(get_logger(), "%s: set current #%d done", log_prefix, mcur.seq);
    go_idle();
    wp_cur_active = mcur.seq;
    set_current_waypoint(wp_cur_active);

    lock.unlock();
    list_sending.notify_all();
    publish_waypoints();
  } else if (wp_state == WP::IDLE && wp_cur_active != mcur.seq) {
    /* update active */
    RCLCPP_DEBUG(get_logger(), "%s: update current #%d", log_prefix, mcur.seq);
    wp_cur_active = mcur.seq;
    set_current_waypoint(wp_cur_active);

    lock.unlock();
    publish_waypoints();
  }
}

void MissionBase::handle_mission_item_reached(
  const mavlink::mavlink_message_t * msg [[maybe_unused]],
  mavlink::common::msg::MISSION_ITEM_REACHED & mitr,
  MFilter filter [[maybe_unused]])
{
  // NOTE(vooon): this message does not have mission_type
  // if (filter_message(mitr)) {
  //   return;
  // }

  // in QGC used as informational message
  RCLCPP_INFO(get_logger(), "%s: reached #%d", log_prefix, mitr.seq);
  publish_reached(mitr.seq);
}

void MissionBase::timeout_cb()
{
  unique_lock lock(mutex);

  // run once
  timeout_timer->cancel();

  if (wp_retries > 0) {
    wp_retries--;
    RCLCPP_WARN(get_logger(), "%s: timeout, retries left %zu", log_prefix, wp_retries);

    switch (wp_state) {
      case WP::RXLIST:
        mission_request_list();
        break;
      case WP::RXWP:
        mission_request(wp_cur_id);
        break;
      case WP::RXWPINT:
        mission_request(wp_cur_id);
        break;
      case WP::TXLIST:
        mission_count(wp_count);
        break;
      case WP::TXPARTIAL:
        mission_write_partial_list(wp_start_id, wp_end_id);
        break;
      case WP::TXWP:
        send_waypoint<MISSION_ITEM>(wp_cur_id);
        break;
      case WP::TXWPINT:
        send_waypoint<MISSION_ITEM_INT>(wp_cur_id);
        break;
      case WP::CLEAR:
        mission_clear_all();
        break;
      case WP::SET_CUR:
        mission_set_current(wp_set_active);
        break;

      case WP::IDLE:
        break;
    }

    restart_timeout_timer_int();
    return;
  }

  auto use_int = use_mission_item_int && !mission_item_int_support_confirmed;

  if (wp_state == WP::TXWPINT && use_int) {
    RCLCPP_ERROR(
      get_logger(), "%s: mission_item_int timed out, falling back to mission_item.", log_prefix);
    use_mission_item_int = false;

    wp_state = WP::TXWP;
    restart_timeout_timer();
    send_waypoint<MISSION_ITEM>(wp_cur_id);
  } else if (wp_state == WP::RXWPINT && use_int) {
    RCLCPP_ERROR(
      get_logger(), "%s: mission_item_int timed out, falling back to mission_item.", log_prefix);
    use_mission_item_int = false;

    wp_state = WP::RXWP;
    restart_timeout_timer();
    mission_request(wp_cur_id);
  } else {
    RCLCPP_ERROR(get_logger(), "%s: timed out.", log_prefix);
    go_idle();
    is_timedout = true;
    // prevent waiting cond var timeout
    lock.unlock();
    list_receiving.notify_all();
    list_sending.notify_all();
  }
}

void MissionBase::mission_request(const uint16_t seq)
{
  RCLCPP_DEBUG(get_logger(), "%s:m: request #%u", log_prefix, seq);

  mavlink::common::msg::MISSION_REQUEST mrq {};
  uas->msg_set_target(mrq);
  mrq.seq = seq;
  mrq.mission_type = enum_value(mission_type);

  uas->send_message(mrq);
}

void MissionBase::mission_request_int(const uint16_t seq)
{
  RCLCPP_DEBUG(get_logger(), "%s:m: request_int #%u", log_prefix, seq);

  mavlink::common::msg::MISSION_REQUEST_INT mrq {};
  uas->msg_set_target(mrq);
  mrq.seq = seq;
  mrq.mission_type = enum_value(mission_type);

  uas->send_message(mrq);
}

void MissionBase::mission_set_current(const uint16_t seq)
{
  RCLCPP_DEBUG(get_logger(), "%s:m: set current #%u", log_prefix, seq);

  mavlink::common::msg::MISSION_SET_CURRENT msc {};
  uas->msg_set_target(msc);
  msc.seq = seq;
  // msc.mission_type = enum_value(mission_type);

  uas->send_message(msc);
}

void MissionBase::mission_request_list()
{
  RCLCPP_DEBUG(get_logger(), "%s:m: request list", log_prefix);

  mavlink::common::msg::MISSION_REQUEST_LIST mrl {};
  uas->msg_set_target(mrl);
  mrl.mission_type = enum_value(mission_type);

  uas->send_message(mrl);
}

void MissionBase::mission_count(const uint16_t cnt)
{
  RCLCPP_DEBUG(get_logger(), "%s:m: count %u", log_prefix, cnt);

  mavlink::common::msg::MISSION_COUNT mcnt {};
  uas->msg_set_target(mcnt);
  mcnt.count = cnt;
  mcnt.mission_type = enum_value(mission_type);

  uas->send_message(mcnt);
}

void MissionBase::mission_write_partial_list(const uint16_t start_index, const uint16_t end_index)
{
  RCLCPP_DEBUG(
    get_logger(), "%s:m: write partial list %u - %u", log_prefix,
    start_index, end_index);

  mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST mwpl {};
  uas->msg_set_target(mwpl);
  mwpl.start_index = start_index;
  mwpl.end_index = end_index;
  mwpl.mission_type = enum_value(mission_type);

  uas->send_message(mwpl);
}

void MissionBase::mission_clear_all()
{
  RCLCPP_DEBUG(get_logger(), "%s:m: clear all", log_prefix);

  mavlink::common::msg::MISSION_CLEAR_ALL mclr {};
  uas->msg_set_target(mclr);
  mclr.mission_type = enum_value(mission_type);

  uas->send_message(mclr);
}

void MissionBase::mission_ack(const MRES type)
{
  RCLCPP_DEBUG(get_logger(), "%s:m: ACK %u", log_prefix, enum_value(type));

  mavlink::common::msg::MISSION_ACK mack {};
  uas->msg_set_target(mack);
  mack.type = enum_value(type);
  mack.mission_type = enum_value(mission_type);

  uas->send_message(mack);
}
