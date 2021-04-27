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

#include "mavros/mission_protocol_base.h"

using namespace mavros;          // NOLINT
using namespace mavros::plugin;  // NOLINT

void MissionBase::handle_mission_item_int(
  const mavlink::mavlink_message_t * msg,
  WP_ITEM_INT & wpi,
  Filter filter)
{
  unique_lock lock(mutex);

  // Only interested in the specific msg type
  if (wpi.mission_type != enum_value(wp_type)) {
    return;
  }
  // receive item only in RX state
  else if (wp_state == WP::RXWPINT) {
    if (wpi.seq != wp_cur_id) {
      RCLCPP_WARN(get_logger(), "MP: Seq mismatch, dropping item (%d != %zu)", wpi.seq, wp_cur_id);
      return;
    }

    auto it = waypoints.emplace(waypoints.end(), wpi);
    RCLCPP_INFO_STREAM(get_logger(), "MP: item " << it->to_string());

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
    RCLCPP_DEBUG(get_logger(), "MP: rejecting item, wrong state %d", enum_value(wp_state));
    if (do_pull_after_gcs && reschedule_pull) {
      RCLCPP_DEBUG(get_logger(), "MP: reschedule pull");
      schedule_pull(WP_TIMEOUT);
    }
  }
}

void MissionBase::handle_mission_item(
  const mavlink::mavlink_message_t * msg,
  WP_ITEM & wpi,
  Filter filter)
{
  unique_lock lock(mutex);

  // Only interested in the specific msg type
  if (wpi.mission_type != enum_value(wp_type)) {
    return;
  }
  // receive item only in RX state
  else if (wp_state == WP::RXWP) {
    if (wpi.seq != wp_cur_id) {
      RCLCPP_WARN(get_logger(), "MP: Seq mismatch, dropping item (%d != %zu)", wpi.seq, wp_cur_id);
      return;
    }

    auto it = waypoints.emplace(waypoints.end(), wpi);
    RCLCPP_INFO_STREAM(get_logger(), "MP: item " << it->to_string());

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
      get_logger(),
      "MP: rejecting item, wrong state %d", enum_value(wp_state));
    if (do_pull_after_gcs && reschedule_pull) {
      RCLCPP_DEBUG(get_logger(), "MP: reschedule pull");
      schedule_pull(WP_TIMEOUT);
    }
  }
}

bool MissionBase::sequence_mismatch(const uint16_t & seq)
{
  if (seq != wp_cur_id && seq != wp_cur_id + 1) {
    RCLCPP_WARN(get_logger(), "MP: Seq mismatch, dropping request (%d != %zu)", seq, wp_cur_id);
    return true;
  }

  return false;
}

void MissionBase::handle_mission_request(
  const mavlink::mavlink_message_t * msg,
  mavlink::common::msg::MISSION_REQUEST & mreq,
  Filter filter)
{
  lock_guard lock(mutex);

  // Only interested in the specific msg type
  if (mreq.mission_type != enum_value(wp_type)) {
    return;
  }

  if (
    (wp_state == WP::TXLIST && mreq.seq == 0) ||
    (wp_state == WP::TXPARTIAL && mreq.seq == wp_start_id) || (wp_state == WP::TXWP) ||
    (wp_state == WP::TXWPINT))
  {
    if (sequence_mismatch(mreq.seq)) {
      return;
    }

    restart_timeout_timer();
    if (mreq.seq < wp_end_id) {
      RCLCPP_DEBUG(get_logger(), "MP: FCU requested MISSION_ITEM waypoint %d", mreq.seq);
      wp_cur_id = mreq.seq;
      if (use_mission_item_int) {
        RCLCPP_DEBUG(get_logger(), "MP: Trying to send a MISSION_ITEM_INT instead");
        wp_state = WP::TXWPINT;
        send_waypoint<WP_ITEM_INT>(wp_cur_id);
      } else {
        wp_state = WP::TXWP;
        send_waypoint<WP_ITEM>(wp_cur_id);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "MP: FCU require seq out of range");
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "MP: rejecting request, wrong state %d", enum_value(wp_state));
  }
}

void MissionBase::handle_mission_request_int(
  const mavlink::mavlink_message_t * msg,
  mavlink::common::msg::MISSION_REQUEST_INT & mreq,
  Filter filter)
{
  lock_guard lock(mutex);

  // Only interested in the specific msg type
  if (mreq.mission_type != enum_value(wp_type)) {
    return;
  }

  if (
    (wp_state == WP::TXLIST && mreq.seq == 0) ||
    (wp_state == WP::TXPARTIAL && mreq.seq == wp_start_id) || (wp_state == WP::TXWPINT))
  {
    if (sequence_mismatch(mreq.seq)) {
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
      RCLCPP_DEBUG(get_logger(), "MP: FCU reqested MISSION_ITEM_INT waypoint %d", mreq.seq);
      wp_state = WP::TXWPINT;
      wp_cur_id = mreq.seq;
      send_waypoint<WP_ITEM_INT>(wp_cur_id);
    } else {
      RCLCPP_ERROR(get_logger(), "MP: FCU require seq out of range");
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "MP: rejecting request, wrong state %d", enum_value(wp_state));
  }
}

void MissionBase::handle_mission_count(
  const mavlink::mavlink_message_t * msg,
  mavlink::common::msg::MISSION_COUNT & mcnt,
  Filter filter)
{
  unique_lock lock(mutex);

  // Only interested in the specific msg type
  if (mcnt.mission_type != enum_value(wp_type)) {
    return;
  }

  if (wp_state == WP::RXLIST) {
    // FCU report of MISSION_REQUEST_LIST
    RCLCPP_DEBUG(get_logger(), "MP: count %d", mcnt.count);

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
    RCLCPP_INFO(get_logger(), "MP: seems GCS requesting mission");
    // schedule pull after GCS done
    if (do_pull_after_gcs) {
      RCLCPP_INFO(get_logger(), "MP: scheduling pull after GCS is done");
      reschedule_pull = true;
      schedule_pull(RESCHEDULE_TIME);
    }
  }
}

void MissionBase::handle_mission_ack(
  const mavlink::mavlink_message_t * msg,
  mavlink::common::msg::MISSION_ACK & mack,
  Filter filter)
{
  unique_lock lock(mutex);

  auto ack_type = static_cast<MRES>(mack.type);

  // Only interested in the specific msg type
  if (mack.mission_type != enum_value(wp_type)) {
    return;
  }

  if (
    (wp_state == WP::TXLIST || wp_state == WP::TXPARTIAL || wp_state == WP::TXWP ||
    wp_state == WP::TXWPINT) &&
    (wp_cur_id == wp_end_id - 1) && (ack_type == MRES::ACCEPTED))
  {
    go_idle();
    waypoints = send_waypoints;
    send_waypoints.clear();
    if (wp_state == WP::TXWPINT) {
      mission_item_int_support_confirmed = true;
    }
    lock.unlock();
    list_sending.notify_all();
    publish_waypoints();
    RCLCPP_INFO(get_logger(), "MP: mission sended");
  } else if (
    (wp_state == WP::TXWP || wp_state == WP::TXWPINT) && ack_type == MRES::INVALID_SEQUENCE)
  {
    // Mission Ack: INVALID_SEQUENCE received during TXWP
    // This happens when waypoint N was received by autopilot, but the request for waypoint N+1 failed.
    // This causes seq mismatch, ignore and eventually the request for n+1 will get to us and seq will sync up.
    RCLCPP_DEBUG(get_logger(), "MP: Received INVALID_SEQUENCE ack");
  } else if (
    wp_state == WP::TXLIST || wp_state == WP::TXPARTIAL || wp_state == WP::TXWP ||
    wp_state == WP::TXWPINT)
  {
    go_idle();
    // use this flag for failure report
    is_timedout = true;
    lock.unlock();
    list_sending.notify_all();

    RCLCPP_ERROR_STREAM(get_logger(), "MP: upload failed: " << utils::to_string(ack_type));
  } else if (wp_state == WP::CLEAR) {
    go_idle();
    if (ack_type != MRES::ACCEPTED) {
      is_timedout = true;
      lock.unlock();
      RCLCPP_ERROR_STREAM(get_logger(), "MP: clear failed: " << utils::to_string(ack_type));
    } else {
      waypoints.clear();
      lock.unlock();
      publish_waypoints();
      RCLCPP_INFO(get_logger(), "MP: mission cleared");
    }

    list_sending.notify_all();
  } else {
    RCLCPP_DEBUG(get_logger(), "MP: not planned ACK, type: %d", mack.type);
  }
}

void MissionBase::timeout_cb()
{
  unique_lock lock(mutex);

  // run once
  timeout_timer->cancel();

  if (wp_retries > 0) {
    wp_retries--;
    RCLCPP_WARN(get_logger(), "MP: timeout, retries left %zu", wp_retries);

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
        send_waypoint<WP_ITEM>(wp_cur_id);
        break;
      case WP::TXWPINT:
        send_waypoint<WP_ITEM_INT>(wp_cur_id);
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
  } else {
    if (wp_state == WP::TXWPINT && use_mission_item_int && !mission_item_int_support_confirmed) {
      RCLCPP_ERROR(get_logger(), "MP: mission_item_int timed out, falling back to mission_item.");
      use_mission_item_int = false;

      wp_state = WP::TXWP;
      restart_timeout_timer();
      send_waypoint<WP_ITEM>(wp_cur_id);
    } else if (
      wp_state == WP::RXWPINT && use_mission_item_int && !mission_item_int_support_confirmed)
    {
      RCLCPP_ERROR(
        get_logger(), "MP: mission_item_int timed out, falling back to mission_item.");
      use_mission_item_int = false;

      wp_state = WP::RXWP;
      restart_timeout_timer();
      mission_request(wp_cur_id);
    } else {
      RCLCPP_ERROR(get_logger(), "MP: timed out.");
      go_idle();
      is_timedout = true;
      // prevent waiting cond var timeout
      lock.unlock();
      list_receiving.notify_all();
      list_sending.notify_all();
    }
  }
}
