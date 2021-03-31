/**
 * @brief Mission base plugin
 * @file mission_protocol_base.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Charlie Burge <charlieburge@yahoo.com>
 *
 */
/*
 * Copyright 2014,2015,2016,2017,2018 Vladimir Ermakov.
 * Copyright 2021 Charlie Burge.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mission_protocol_base.h>


namespace mavros {
namespace plugin {
void MissionBase::handle_mission_item_int(const mavlink::mavlink_message_t *msg, WP_ITEM_INT &wpi)
{
	unique_lock lock(mutex);

	/* Only interested in the specific msg type */
	if (wpi.mission_type != enum_value(wp_type)) {
		return;
	}
	/* receive item only in RX state */
	else if (wp_state == WP::RXWPINT) {
		if (wpi.seq != wp_cur_id) {
			ROS_WARN_NAMED(log_ns, "%s: Seq mismatch, dropping item (%d != %zu)",
				log_ns.c_str(), wpi.seq, wp_cur_id);
			return;
		}

		ROS_INFO_STREAM_NAMED(log_ns, log_ns << ": item " << waypoint_to_string<WP_ITEM_INT>(wpi));

		waypoints.push_back(mav_to_msg<WP_ITEM_INT>(wpi));
		if (++wp_cur_id < wp_count) {
			restart_timeout_timer();
			mission_request_int(wp_cur_id);
		}
		else {
			request_mission_done();
			mission_item_int_support_confirmed = true;
			lock.unlock();
			publish_waypoints();
		}
	}
	else {
		ROS_DEBUG_NAMED(log_ns, "%s: rejecting item, wrong state %d", log_ns.c_str(), enum_value(wp_state));
		if (do_pull_after_gcs && reschedule_pull) {
			ROS_DEBUG_NAMED(log_ns, "%s: reschedule pull", log_ns.c_str());
			schedule_pull(WP_TIMEOUT_DT);
		}
	}
}


void MissionBase::handle_mission_item(const mavlink::mavlink_message_t *msg, WP_ITEM &wpi)
{
	unique_lock lock(mutex);

	/* Only interested in the specific msg type */
	if (wpi.mission_type != enum_value(wp_type)) {
		return;
	}
	/* receive item only in RX state */
	else if (wp_state == WP::RXWP) {
		if (wpi.seq != wp_cur_id) {
			ROS_WARN_NAMED(log_ns, "%s: Seq mismatch, dropping item (%d != %zu)",
				log_ns.c_str(), wpi.seq, wp_cur_id);
			return;
		}

		ROS_INFO_STREAM_NAMED(log_ns, log_ns << ": item " << waypoint_to_string<WP_ITEM>(wpi));

		waypoints.push_back(mav_to_msg<WP_ITEM>(wpi));
		if (++wp_cur_id < wp_count) {
			restart_timeout_timer();
			mission_request(wp_cur_id);
		}
		else {
			request_mission_done();
			lock.unlock();
			publish_waypoints();
		}
	}
	else {
		ROS_DEBUG_NAMED(log_ns, "%s: rejecting item, wrong state %d", log_ns.c_str(), enum_value(wp_state));
		if (do_pull_after_gcs && reschedule_pull) {
			ROS_DEBUG_NAMED(log_ns, "%s: reschedule pull", log_ns.c_str());
			schedule_pull(WP_TIMEOUT_DT);
		}
	}
}


bool MissionBase::sequence_mismatch(const uint16_t &seq)
{
	if (seq != wp_cur_id && seq != wp_cur_id + 1) {
		ROS_WARN_NAMED(log_ns, "%s: Seq mismatch, dropping request (%d != %zu)",
			log_ns.c_str(), seq, wp_cur_id);
		return true;
	} else {
		return false;
	}
}


void MissionBase::handle_mission_request(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_REQUEST &mreq)
{
	lock_guard lock(mutex);

	/* Only interested in the specific msg type */
	if (mreq.mission_type != enum_value(wp_type)) {
		return;
	}
	else if ((wp_state == WP::TXLIST && mreq.seq == 0) || (wp_state == WP::TXPARTIAL && mreq.seq == wp_start_id) || (wp_state == WP::TXWP) || (wp_state == WP::TXWPINT)) {
		if (sequence_mismatch(mreq.seq)) {
			return;
		}

		restart_timeout_timer();
		if (mreq.seq < wp_end_id) {
			ROS_DEBUG_NAMED(log_ns, "%s: FCU requested MISSION_ITEM waypoint %d", log_ns.c_str(), mreq.seq);
			wp_cur_id = mreq.seq;
			if (use_mission_item_int) {
				ROS_DEBUG_NAMED(log_ns, "%s: Trying to send a MISSION_ITEM_INT instead", log_ns.c_str());
				wp_state = WP::TXWPINT;
				send_waypoint<WP_ITEM_INT>(wp_cur_id);
			} else {
				wp_state = WP::TXWP;
				send_waypoint<WP_ITEM>(wp_cur_id);
			}
		}
		else
			ROS_ERROR_NAMED(log_ns, "%s: FCU require seq out of range", log_ns.c_str());
	}
	else
		ROS_DEBUG_NAMED(log_ns, "%s: rejecting request, wrong state %d", log_ns.c_str(), enum_value(wp_state));
}


void MissionBase::handle_mission_request_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_REQUEST_INT &mreq)
{
	lock_guard lock(mutex);

	/* Only interested in the specific msg type */
	if (mreq.mission_type != enum_value(wp_type)) {
		return;
	}
	else if ((wp_state == WP::TXLIST && mreq.seq == 0) || (wp_state == WP::TXPARTIAL && mreq.seq == wp_start_id) || (wp_state == WP::TXWPINT)) {
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
			ROS_DEBUG_NAMED(log_ns, "%s: FCU reqested MISSION_ITEM_INT waypoint %d", log_ns.c_str(), mreq.seq);
			wp_state = WP::TXWPINT;
			wp_cur_id = mreq.seq;
			send_waypoint<WP_ITEM_INT>(wp_cur_id);
		}
		else
			ROS_ERROR_NAMED(log_ns, "%s: FCU require seq out of range", log_ns.c_str());
	}
	else
		ROS_DEBUG_NAMED(log_ns, "%s: rejecting request, wrong state %d", log_ns.c_str(), enum_value(wp_state));
}


void MissionBase::handle_mission_count(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_COUNT &mcnt)
{
	unique_lock lock(mutex);

	/* Only interested in the specific msg type */
	if (mcnt.mission_type != enum_value(wp_type)) {
		return;
	}
	else if (wp_state == WP::RXLIST) {
		/* FCU report of MISSION_REQUEST_LIST */
		ROS_DEBUG_NAMED(log_ns, "%s: count %d", log_ns.c_str(), mcnt.count);

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
		}
		else {
			request_mission_done();
			lock.unlock();
			publish_waypoints();
		}
	}
	else {
		ROS_INFO_NAMED(log_ns, "%s: seems GCS requesting mission", log_ns.c_str());
		/* schedule pull after GCS done */
		if (do_pull_after_gcs) {
			ROS_INFO_NAMED(log_ns, "%s: scheduling pull after GCS is done", log_ns.c_str());
			reschedule_pull = true;
			schedule_pull(RESCHEDULE_DT);
		}
	}
}


void MissionBase::handle_mission_ack(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_ACK &mack)
{
	unique_lock lock(mutex);

	auto ack_type = static_cast<MRES>(mack.type);

	/* Only interested in the specific msg type */
	if (mack.mission_type != enum_value(wp_type)) {
		return;
	}
	else if ((wp_state == WP::TXLIST || wp_state == WP::TXPARTIAL || wp_state == WP::TXWP || wp_state == WP::TXWPINT)
		&& (wp_cur_id == wp_end_id - 1)
		&& (ack_type == MRES::ACCEPTED)) {
		go_idle();
		waypoints = send_waypoints;
		send_waypoints.clear();
		if (wp_state == WP::TXWPINT) mission_item_int_support_confirmed = true;
		lock.unlock();
		list_sending.notify_all();
		publish_waypoints();
		ROS_INFO_NAMED(log_ns, "%s: mission sended", log_ns.c_str());
	}
	else if ((wp_state == WP::TXWP || wp_state == WP::TXWPINT) && ack_type == MRES::INVALID_SEQUENCE) {
		// Mission Ack: INVALID_SEQUENCE received during TXWP
		// This happens when waypoint N was received by autopilot, but the request for waypoint N+1 failed.
		// This causes seq mismatch, ignore and eventually the request for n+1 will get to us and seq will sync up.
		ROS_DEBUG_NAMED(log_ns, "%s: Received INVALID_SEQUENCE ack", log_ns.c_str());
	}
	else if (wp_state == WP::TXLIST || wp_state == WP::TXPARTIAL || wp_state == WP::TXWP || wp_state == WP::TXWPINT) {
		go_idle();
		/* use this flag for failure report */
		is_timedout = true;
		lock.unlock();
		list_sending.notify_all();

		ROS_ERROR_STREAM_NAMED(log_ns, log_ns << ": upload failed: " << utils::to_string(ack_type));
	}
	else if (wp_state == WP::CLEAR) {
		go_idle();
		if (ack_type != MRES::ACCEPTED) {
			is_timedout = true;
			lock.unlock();
			ROS_ERROR_STREAM_NAMED(log_ns, log_ns << ": clear failed: " << utils::to_string(ack_type));
		}
		else {
			waypoints.clear();
			lock.unlock();
			publish_waypoints();
			ROS_INFO_NAMED(log_ns, "%s: mission cleared", log_ns.c_str());
		}

		list_sending.notify_all();
	}
	else
		ROS_DEBUG_NAMED(log_ns, "%s: not planned ACK, type: %d", log_ns.c_str(), mack.type);
}


void MissionBase::timeout_cb(const ros::TimerEvent &event)
{
	unique_lock lock(mutex);
	if (wp_retries > 0) {
		wp_retries--;
		ROS_WARN_NAMED(log_ns, "%s: timeout, retries left %zu", log_ns.c_str(), wp_retries);

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
	}
	else {
		if (wp_state == WP::TXWPINT && use_mission_item_int && !mission_item_int_support_confirmed) {
			ROS_ERROR_NAMED(log_ns, "%s: mission_item_int timed out, falling back to mission_item.", log_ns.c_str());
			use_mission_item_int = false;

			wp_state = WP::TXWP;
			restart_timeout_timer();
			send_waypoint<WP_ITEM>(wp_cur_id);
		} else if (wp_state == WP::RXWPINT && use_mission_item_int && !mission_item_int_support_confirmed) {
			ROS_ERROR_NAMED(log_ns, "%s: mission_item_int timed out, falling back to mission_item.", log_ns.c_str());
			use_mission_item_int = false;

			wp_state = WP::RXWP;
			restart_timeout_timer();
			mission_request(wp_cur_id);
		} else {
			ROS_ERROR_NAMED(log_ns, "%s: timed out.", log_ns.c_str());
			go_idle();
			is_timedout = true;
			/* prevent waiting cond var timeout */
			lock.unlock();
			list_receiving.notify_all();
			list_sending.notify_all();
		}
	}
}
}	// namespace plugin
}	// namespace mavros