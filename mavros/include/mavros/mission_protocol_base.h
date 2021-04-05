/**
 * @brief Mission base plugin
 * @file mission_protocol_base.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Charlie Burge <charlieburge@yahoo.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016,2017,2018 Vladimir Ermakov.
 * Copyright 2021 Charlie Burge.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <chrono>
#include <sstream>
#include <iomanip>
#include <string>

#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <mavros/mavros_uas.h>

#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>

namespace mavros {
namespace plugin {
using mavlink::common::MAV_CMD;
using mavlink::common::MAV_FRAME;
using MRES = mavlink::common::MAV_MISSION_RESULT;
using utils::enum_value;
using mavlink::common::MAV_FRAME;
using WP_ITEM = mavlink::common::msg::MISSION_ITEM;
using WP_ITEM_INT = mavlink::common::msg::MISSION_ITEM_INT;
using WP_TYPE = mavlink::common::MAV_MISSION_TYPE;

static double waypoint_encode_factor( const uint8_t &frame ){
	switch (frame) {
	// [[[cog:
	// from pymavlink.dialects.v20 import common
	// e = common.enums['MAV_FRAME']
	// all_names = [ee.name[len('MAV_FRAME_'):] for ee in e.values()]
	// all_names.pop() # remove ENUM_END
	// global_names = [v for v in all_names if v.startswith('GLOBAL')]
	// local_names = [v for v in all_names if v.startswith(('LOCAL', 'BODY', 'MOCAP', 'VISION', 'ESTIM'))]
	// other_names = ['MISSION']
	//
	// for names, factor in [(global_names, 10000000), (local_names, 10000), (other_names, 1)]:
	// 	for name in names:
	// 		cog.outl(f"case enum_value(MAV_FRAME::{name}):")
	// 	cog.outl(f"\treturn {factor};")
	//
	// cog.outl("default:\n\treturn 1;")
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
	// [[[end]]] (checksum: 152fba25e8f422b878caf990a551a6fd)
	}
}

template <class ITEM>
mavros_msgs::Waypoint mav_to_msg(const ITEM &mav_msg)
{
	mavros_msgs::Waypoint ret;

	// [[[cog:
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
	// for a, b in waypoint_item_msg + waypoint_coords:
	//     cog.outl(f"ret.{a} = mav_msg.{b};")
	// ]]]
	ret.frame = mav_msg.frame;
	ret.command = mav_msg.command;
	ret.is_current = mav_msg.current;
	ret.autocontinue = mav_msg.autocontinue;
	ret.param1 = mav_msg.param1;
	ret.param2 = mav_msg.param2;
	ret.param3 = mav_msg.param3;
	ret.param4 = mav_msg.param4;
	ret.x_lat = mav_msg.x;
	ret.y_long = mav_msg.y;
	ret.z_alt = mav_msg.z;
	// [[[end]]] (checksum: 27badd1a5facc63f38cdd7aad3be9816)

	return ret;
}


template<>
inline mavros_msgs::Waypoint mav_to_msg(const WP_ITEM_INT &mav_msg){
	mavros_msgs::Waypoint ret;

	// [[[cog:
	// for a, b in waypoint_item_msg + waypoint_coords:
	//     if a.startswith(('x', 'y')):
	//         cog.outl(f"ret.{a} = mav_msg.{b} / waypoint_encode_factor(mav_msg.frame);")
	//     else:
	//         cog.outl(f"ret.{a} = mav_msg.{b};")
	// ]]]
	ret.frame = mav_msg.frame;
	ret.command = mav_msg.command;
	ret.is_current = mav_msg.current;
	ret.autocontinue = mav_msg.autocontinue;
	ret.param1 = mav_msg.param1;
	ret.param2 = mav_msg.param2;
	ret.param3 = mav_msg.param3;
	ret.param4 = mav_msg.param4;
	ret.x_lat = mav_msg.x / waypoint_encode_factor(mav_msg.frame);
	ret.y_long = mav_msg.y / waypoint_encode_factor(mav_msg.frame);
	ret.z_alt = mav_msg.z;
	// [[[end]]] (checksum: 6c82a18990af7aeeb1db9211e9b1bbf1)

	return ret;
}


template <class ITEM>
ITEM mav_from_msg(const mavros_msgs::Waypoint &wp, const uint16_t seq, WP_TYPE type){
	ITEM ret{};

	// [[[cog:
	// for a, b in waypoint_item_msg + waypoint_coords:
	//     cog.outl(f"ret.{b} = wp.{a};")
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
	// [[[end]]] (checksum: c1b08cda34f1c4dc94129bda4743aaec)

	ret.seq = seq;

	// defaults to 0 -> MAV_MISSION_TYPE_MISSION
	ret.mission_type = enum_value(type);

	return ret;
}

template <>
inline WP_ITEM_INT mav_from_msg(const mavros_msgs::Waypoint &wp, const uint16_t seq, WP_TYPE type){
	WP_ITEM_INT ret{};

	// [[[cog:
	// for a, b in waypoint_item_msg + waypoint_coords:
	//     if b.startswith(('x', 'y')):
	//         cog.outl(f"ret.{b} = int32_t(wp.{a} * waypoint_encode_factor(wp.frame));")
	//     else:
	//         cog.outl(f"ret.{b} = wp.{a};")
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
	// [[[end]]] (checksum: 6315c451fe834dbf20a43ee112b8b5fe)

	ret.seq = seq;

	// defaults to 0 -> MAV_MISSION_TYPE_MISSION
	ret.mission_type = enum_value(type);

	return ret;
}

template <class ITEM>
std::string waypoint_to_string(const ITEM &wp)
{
	std::stringstream ss;
	ss.precision(7);
	ss << '#' << wp.seq << (wp.current ? '*' : ' ') << " F:" << wp.frame << " C:" << std::setw(3) << wp.command;
	ss << " p: " <<   wp.param1 << ' ' << wp.param2 << ' ' << wp.param3 << ' ' << wp.param4 << " x: " << wp.x << " y: " << wp.y << " z: " << wp.z;
	return ss.str();
}


/**
 * @brief Mission base plugin
 */
class MissionBase : public plugin::PluginBase {
public:
	MissionBase(std::string _name) :
		PluginBase(),
		log_ns(_name),
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
		BOOTUP_TIME_DT(BOOTUP_TIME_MS / 1000.0),
		LIST_TIMEOUT_DT(LIST_TIMEOUT_MS / 1000.0),
		WP_TIMEOUT_DT(WP_TIMEOUT_MS / 1000.0),
		RESCHEDULE_DT(RESCHEDULE_MS / 1000.0)
	{ }

	virtual void initialize(ros::NodeHandle *_wp_nh)
	{
		wp_timer = _wp_nh->createTimer(WP_TIMEOUT_DT, &MissionBase::timeout_cb, this, true);
		wp_timer.stop();

		schedule_timer = _wp_nh->createTimer(BOOTUP_TIME_DT, &MissionBase::scheduled_pull_cb, this, true);
		schedule_timer.stop();
	}

protected:
	using unique_lock = std::unique_lock<std::recursive_mutex>;
	using lock_guard = std::lock_guard<std::recursive_mutex>;

	std::string log_ns;

	std::recursive_mutex mutex;

	std::vector<mavros_msgs::Waypoint> waypoints;
	std::vector<mavros_msgs::Waypoint> send_waypoints;

	enum class WP {
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

	ros::Timer wp_timer;
	ros::Timer schedule_timer;
	bool do_pull_after_gcs;
	bool enable_partial_push;

	bool reschedule_pull;

	bool use_mission_item_int;
	bool mission_item_int_support_confirmed;

	static constexpr int BOOTUP_TIME_MS = 15000;	//! system startup delay before start pull
	static constexpr int LIST_TIMEOUT_MS = 30000;	//! Timeout for pull/push operations
	static constexpr int WP_TIMEOUT_MS = 1000;
	static constexpr int RESCHEDULE_MS = 5000;
	static constexpr int RETRIES_COUNT = 3;
	static constexpr unsigned int MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4;

	const ros::Duration BOOTUP_TIME_DT;
	const ros::Duration LIST_TIMEOUT_DT;
	const ros::Duration WP_TIMEOUT_DT;
	const ros::Duration RESCHEDULE_DT;

	/* -*- rx handlers -*- */

	/**
	 * @brief handle MISSION_ITEM_INT mavlink msg
	 * handles and stores mission items when pulling waypoints
	 * @param msg		Received Mavlink msg
	 * @param wpi		WaypointItemInt from msg
	 */
	void handle_mission_item_int(const mavlink::mavlink_message_t *msg, WP_ITEM_INT &wpi);


	/**
	 * @brief handle MISSION_ITEM mavlink msg
	 * handles and stores mission items when pulling waypoints
	 * @param msg		Received Mavlink msg
	 * @param wpi		WaypointItem from msg
	 */
	void handle_mission_item(const mavlink::mavlink_message_t *msg, WP_ITEM &wpi);

	/**
	 * @brief checks for a sequence mismatch between a
	 * MISSION_REQUEST(_INT) sequence and the current
	 * waypoint that should be sent.
	 * @param seq	The seq member of a MISSION_REQUEST(_INT)
	 * @return		True if there is a sequence mismatch
	 */
	bool sequence_mismatch(const uint16_t &seq);

	/**
	 * @brief handle MISSION_REQUEST mavlink msg
	 * handles and acts on misison request from FCU
	 * @param msg		Received Mavlink msg
	 * @param mreq		MISSION_REQUEST from msg
	 */
	void handle_mission_request(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_REQUEST &mreq);

	/**
	 * @brief handle MISSION_REQUEST_INT mavlink msg
	 * handles and acts on misison request from FCU
	 * @param msg		Received Mavlink msg
	 * @param mreq		MISSION_REQUEST_INT from msg
	 */
	void handle_mission_request_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_REQUEST_INT &mreq);

	/**
	 * @brief handle MISSION_COUNT mavlink msg
	 * Handles a mission count from FCU in a Waypoint Pull
	 * Triggers a pull GCS seems to be requesting mission
	 * @param msg		Received Mavlink msg
	 * @param mcnt		MISSION_COUNT from msg
	 */
	void handle_mission_count(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_COUNT &mcnt);

	/**
	 * @brief handle MISSION_ACK mavlink msg
	 * Handles a MISSION_ACK which marks the end of a push, or a failure
	 * @param msg		Received Mavlink msg
	 * @param mack		MISSION_ACK from msg
	 */
	void handle_mission_ack(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_ACK &mack);

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Act on a timeout
	 * Resend the message that may have been lost
	 */
	void timeout_cb(const ros::TimerEvent &event);

	//! @brief Callback for scheduled waypoint pull
	void scheduled_pull_cb(const ros::TimerEvent &event)
	{
		lock_guard lock(mutex);
		if (wp_state != WP::IDLE) {
			/* try later */
			ROS_DEBUG_NAMED(log_ns, "%s: busy, reschedule pull", log_ns.c_str());
			schedule_pull(RESCHEDULE_DT);
			return;
		}

		ROS_DEBUG_NAMED(log_ns, "%s: start scheduled pull", log_ns.c_str());
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
		ROS_INFO_NAMED(log_ns, "%s: mission received", log_ns.c_str());
	}

	void go_idle(void)
	{
		reschedule_pull = false;
		wp_state = WP::IDLE;
		wp_timer.stop();
	}

	void restart_timeout_timer(void)
	{
		wp_retries = RETRIES_COUNT;
		restart_timeout_timer_int();
	}

	void restart_timeout_timer_int(void)
	{
		is_timedout = false;
		wp_timer.stop();
		wp_timer.start();
	}

	void schedule_pull(const ros::Duration &dt)
	{
		schedule_timer.stop();
		schedule_timer.setPeriod(dt);
		schedule_timer.start();
	}

	//! @brief send a single waypoint to FCU
	template <class ITEM>
	void send_waypoint(size_t seq){
		if (seq < send_waypoints.size()) {
			auto wp_msg = send_waypoints.at(seq);
			auto wpi = mav_from_msg<ITEM>(wp_msg, seq, wp_type);
			mission_send(wpi);
			ROS_DEBUG_STREAM_NAMED(log_ns, log_ns << ": send item " << waypoint_to_string<ITEM>(wpi));
		}
	}

	/**
	 * @brief wait until a waypoint pull is complete.
	 * Pull happens asynchronously, this function blocks until it is done.
	 */
	bool wait_fetch_all()
	{
		std::unique_lock<std::mutex> lock(recv_cond_mutex);
		return list_receiving.wait_for(lock, std::chrono::nanoseconds(LIST_TIMEOUT_DT.toNSec()))
		       == std::cv_status::no_timeout
		       && !is_timedout;
	}

	/**
	 * @brief wait until a waypoint push is complete.
	 * Push happens asynchronously, this function blocks until it is done.
	 */
	bool wait_push_all()
	{
		std::unique_lock<std::mutex> lock(send_cond_mutex);

		return list_sending.wait_for(lock, std::chrono::nanoseconds(LIST_TIMEOUT_DT.toNSec()))
		       == std::cv_status::no_timeout
		       && !is_timedout;
	}

	//! @brief set the FCU current waypoint
	void set_current_waypoint(size_t seq)
	{
		auto i = 0;
		for (auto &it : waypoints) {
			it.is_current = (i == seq) ? true : false;
			i++;
		}
	}

	//! @brief publish the updated waypoint list after operation
	virtual void publish_waypoints() = 0;

	/* -*- low-level send functions -*- */

	template<class ITEM>
	void mission_send(ITEM &wp)
	{
		auto wpi = wp;
		m_uas->msg_set_target(wpi);
		UAS_FCU(m_uas)->send_message_ignore_drop(wpi);
	}

	void mission_request(uint16_t seq)
	{
		ROS_DEBUG_NAMED(log_ns, "%s:m: request #%u", log_ns.c_str(), seq);

		mavlink::common::msg::MISSION_REQUEST mrq {};
		mrq.mission_type = enum_value(wp_type);
		m_uas->msg_set_target(mrq);
		mrq.seq = seq;

		UAS_FCU(m_uas)->send_message_ignore_drop(mrq);
	}

	void mission_request_int(uint16_t seq)
	{
		ROS_DEBUG_NAMED(log_ns, "%s:m: request_int #%u", log_ns.c_str(), seq);

		mavlink::common::msg::MISSION_REQUEST_INT mrq {};
		mrq.mission_type = enum_value(wp_type);
		m_uas->msg_set_target(mrq);
		mrq.seq = seq;

		UAS_FCU(m_uas)->send_message_ignore_drop(mrq);
	}

	void mission_set_current(uint16_t seq)
	{
		ROS_DEBUG_NAMED(log_ns, "%s:m: set current #%u", log_ns.c_str(), seq);

		mavlink::common::msg::MISSION_SET_CURRENT msc {};
		m_uas->msg_set_target(msc);
		msc.seq = seq;

		UAS_FCU(m_uas)->send_message_ignore_drop(msc);
	}

	void mission_request_list()
	{
		ROS_DEBUG_NAMED(log_ns, "%s:m: request list", log_ns.c_str());

		mavlink::common::msg::MISSION_REQUEST_LIST mrl {};
		mrl.mission_type = enum_value(wp_type);
		m_uas->msg_set_target(mrl);

		UAS_FCU(m_uas)->send_message_ignore_drop(mrl);
	}

	void mission_count(uint16_t cnt)
	{
		ROS_INFO_NAMED(log_ns, "%s:m: count %u", log_ns.c_str(), cnt);

		mavlink::common::msg::MISSION_COUNT mcnt {};
		mcnt.mission_type = enum_value(wp_type);
		mcnt.count = cnt;
		m_uas->msg_set_target(mcnt);

		UAS_FCU(m_uas)->send_message_ignore_drop(mcnt);
	}

	void mission_write_partial_list(uint16_t start_index, uint16_t end_index)
	{
		ROS_DEBUG_NAMED(log_ns, "%s:m: write partial list %u - %u",
			log_ns.c_str(),start_index, end_index);

		mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST mwpl {};
		mwpl.start_index = start_index;
		mwpl.end_index = end_index;
		mwpl.mission_type = enum_value(wp_type);
		m_uas->msg_set_target(mwpl);

		UAS_FCU(m_uas)->send_message_ignore_drop(mwpl);
	}

	void mission_clear_all()
	{
		ROS_DEBUG_NAMED(log_ns, "%s:m: clear all", log_ns.c_str());

		mavlink::common::msg::MISSION_CLEAR_ALL mclr {};
		mclr.mission_type = enum_value(wp_type);
		m_uas->msg_set_target(mclr);

		UAS_FCU(m_uas)->send_message_ignore_drop(mclr);
	}

	void mission_ack(MRES type) {
		ROS_DEBUG_NAMED(log_ns, "%s:m: ACK %u", log_ns.c_str(), enum_value(type));

		mavlink::common::msg::MISSION_ACK mack {};
		mack.type = enum_value(type);
		mack.mission_type = enum_value(wp_type);
		m_uas->msg_set_target(mack);

		UAS_FCU(m_uas)->send_message_ignore_drop(mack);
	}
};
}	// namespace plugin
}	// namespace mavros
