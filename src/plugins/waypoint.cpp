/**
 * @brief Waypoint plugin
 * @file waypoint.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <mavros/WaypointList.h>
#include <mavros/WaypointSetCurrent.h>
#include <mavros/WaypointClear.h>
#include <mavros/WaypointPull.h>
#include <mavros/WaypointPush.h>

namespace mavplugin {

class WaypointItem {
public:
	uint16_t seq;
	enum MAV_FRAME frame;
	enum MAV_CMD command;
	bool current;
	bool autocontinue;
	float param1;
	float param2;
	float param3;
	float param4;
	double x_lat;
	double y_long;
	double z_alt;

	static mavros::Waypoint to_msg(WaypointItem &wp) {
		mavros::Waypoint ret;

		ret.frame = static_cast<uint8_t>(wp.frame);
		ret.command = static_cast<uint16_t>(wp.command);
		ret.is_current = wp.current;
		ret.autocontinue = wp.autocontinue;
		ret.param1 = wp.param1;
		ret.param2 = wp.param1;
		ret.param3 = wp.param1;
		ret.param4 = wp.param1;
		ret.x_lat = wp.x_lat;
		ret.y_long = wp.y_long;
		ret.z_alt = wp.z_alt;

		return ret;
	}

	static WaypointItem from_msg(mavros::Waypoint &wp, uint16_t seq) {
		WaypointItem ret;

		ret.seq = seq;
		ret.frame = static_cast<enum MAV_FRAME>(wp.frame);
		ret.command = static_cast<enum MAV_CMD>(wp.command);
		ret.current = wp.is_current;
		ret.autocontinue = wp.autocontinue;
		ret.param1 = wp.param1;
		ret.param2 = wp.param1;
		ret.param3 = wp.param1;
		ret.param4 = wp.param1;
		ret.x_lat = wp.x_lat;
		ret.y_long = wp.y_long;
		ret.z_alt = wp.z_alt;

		return ret;
	}

	static WaypointItem from_mission_item(mavlink_mission_item_t &mit) {
		WaypointItem ret;

		ret.seq = mit.seq;
		ret.frame = static_cast<enum MAV_FRAME>(mit.frame);
		ret.command = static_cast<enum MAV_CMD>(mit.command);
		ret.current = mit.current;
		ret.autocontinue = mit.autocontinue;
		ret.param1 = mit.param1;
		ret.param2 = mit.param1;
		ret.param3 = mit.param1;
		ret.param4 = mit.param1;
		ret.x_lat = mit.x;
		ret.y_long = mit.y;
		ret.z_alt = mit.z;

		return ret;
	}
};

class WaypointPlugin : public MavRosPlugin {
public:
	WaypointPlugin() {
	};

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;
		wp_state = WP_IDLE;

		wp_nh = ros::NodeHandle(nh, "mission");

		wp_list_pub = wp_nh.advertise<mavros::WaypointList>("waypoints", 2, true);
		pull_srv = wp_nh.advertiseService("pull", &WaypointPlugin::pull_cb, this);
		push_srv = wp_nh.advertiseService("push", &WaypointPlugin::push_cb, this);
		clear_srv = wp_nh.advertiseService("clear", &WaypointPlugin::clear_cb, this);
		set_cur_srv = wp_nh.advertiseService("set_current", &WaypointPlugin::set_cur_cb, this);
	};

	std::string get_name() {
		return "Waypoint";
	};

	std::vector<uint8_t> get_supported_messages() {
		return {
			MAVLINK_MSG_ID_MISSION_ITEM,
			MAVLINK_MSG_ID_MISSION_REQUEST,
			MAVLINK_MSG_ID_MISSION_CURRENT,
			MAVLINK_MSG_ID_MISSION_COUNT,
			MAVLINK_MSG_ID_MISSION_ITEM_REACHED,
			MAVLINK_MSG_ID_MISSION_ACK
		};
	};

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_MISSION_ITEM:
			mavlink_mission_item_t mit;
			mavlink_msg_mission_item_decode(msg, &mit);
			handle_mission_item(mit);
			break;

		case MAVLINK_MSG_ID_MISSION_REQUEST:
			mavlink_mission_request_t mreq;
			mavlink_msg_mission_request_decode(msg, &mreq);
			handle_mission_request(mreq);
			break;

		case MAVLINK_MSG_ID_MISSION_CURRENT:
			mavlink_mission_current_t mcur;
			mavlink_msg_mission_current_decode(msg, &mcur);
			handle_mission_current(mcur);
			break;

		case MAVLINK_MSG_ID_MISSION_COUNT:
			mavlink_mission_count_t mcnt;
			mavlink_msg_mission_count_decode(msg, &mcnt);
			handle_mission_count(mcnt);
			break;

		case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
			mavlink_mission_item_reached_t mitr;
			mavlink_msg_mission_item_reached_decode(msg, &mitr);
			handle_mission_item_reached(mitr);
			break;

		case MAVLINK_MSG_ID_MISSION_ACK:
			mavlink_mission_ack_t mack;
			mavlink_msg_mission_ack_decode(msg, &mack);
			handle_mission_ack(mack);
			break;
		}
	}

private:
	boost::recursive_mutex mutex;
	UAS *uas;

	ros::NodeHandle wp_nh;
	ros::Publisher wp_list_pub;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer clear_srv;
	ros::ServiceServer set_cur_srv;

	std::vector<WaypointItem> waypoints;
	enum {
		WP_IDLE,
		WP_RXLIST,
		WP_TXLIST,
		WP_CLEAR,
		WP_SET_CUR
	} wp_state = WP_IDLE;
	size_t wp_count;
	size_t wp_current;
	boost::condition_variable list_receiving;

	const int LIST_TIMEOUT_MS = 30000;
	const int WP_TIMEOUT_MS = 1000;

	/* -*- rx handlers -*- */

	void handle_mission_item(mavlink_mission_item_t &mit) {
		WaypointItem wpi = WaypointItem::from_mission_item(mit);

		boost::recursive_mutex::scoped_lock lock(mutex);

		/* receive item only in RX state */
		if (wp_state == WP_RXLIST) {
			ROS_INFO_NAMED("wp", "WP: item #%d frame:%d cmd:%d", wpi.seq, wpi.frame, wpi.command);
			waypoints.push_back(wpi); /* todo replace with */
			request_next_mission_item();
		}
	}

	void handle_mission_request(mavlink_mission_request_t &mreq) {
		ROS_WARN_NAMED("wp", "WP: FCU requests #%d", mreq.seq);
	}

	void handle_mission_current(mavlink_mission_current_t &mcur) {
		ROS_DEBUG_COND_NAMED(wp_current != mcur.seq, "wp", "WP: current #%d", mcur.seq);
		wp_current = mcur.seq;
		/* TODO: fix current flag in IDLE */
	}

	void handle_mission_count(mavlink_mission_count_t &mcnt) {
		ROS_DEBUG_NAMED("wp", "WP: count %d", mcnt.count);

		boost::recursive_mutex::scoped_lock lock(mutex);

		if (wp_state == WP_RXLIST) {
			/* FCU report of MISSION_REQUEST_LIST */
			wp_count = mcnt.count;
			waypoints.clear();
			waypoints.reserve(wp_count);
			request_next_mission_item();
		}
	}

	void handle_mission_item_reached(mavlink_mission_item_reached_t &mitr) {
		/* TODO check APM support */
		ROS_INFO_NAMED("wp", "WP: reached #%d", mitr.seq);
	}

	void handle_mission_ack(mavlink_mission_ack_t &mack) {
		ROS_DEBUG_NAMED("wp", "WP: ACK %d", mack.type);
	}

	/* -*- low-level send functions -*- */

	void mission_item(WaypointItem &wp) {
		mavlink_message_t msg;

		mavlink_msg_mission_item_pack(0, 0, &msg,
				uas->get_tgt_system(),
				uas->get_tgt_component(),
				wp.seq,
				wp.frame,
				wp.command,
				wp.current,
				wp.autocontinue,
				wp.param1,
				wp.param2,
				wp.param3,
				wp.param4,
				wp.x_lat,
				wp.y_long,
				wp.z_alt
				);
		uas->mav_link->send_message(&msg);
	}

	void mission_request(uint16_t seq) {
		mavlink_message_t msg;

		mavlink_msg_mission_request_pack(0, 0, &msg,
				uas->get_tgt_system(),
				uas->get_tgt_component(),
				seq
				);
		uas->mav_link->send_message(&msg);
	}

	void mission_set_current(uint16_t seq) {
		mavlink_message_t msg;

		mavlink_msg_mission_set_current_pack(0, 0, &msg,
				uas->get_tgt_system(),
				uas->get_tgt_component(),
				seq
				);
		uas->mav_link->send_message(&msg);
	}

	void mission_request_list() {
		mavlink_message_t msg;

		mavlink_msg_mission_request_list_pack(0, 0, &msg,
				uas->get_tgt_system(),
				uas->get_tgt_component()
				);
		uas->mav_link->send_message(&msg);
	}

	void mission_count(uint16_t cnt) {
		mavlink_message_t msg;

		mavlink_msg_mission_count_pack(0, 0, &msg,
				uas->get_tgt_system(),
				uas->get_tgt_component(),
				cnt
				);
		uas->mav_link->send_message(&msg);
	}

	void mission_clear_all() {
		mavlink_message_t msg;

		mavlink_msg_mission_clear_all_pack(0, 0, &msg,
				uas->get_tgt_system(),
				uas->get_tgt_component()
				);
		uas->mav_link->send_message(&msg);
	}

	void mission_ack(enum MAV_MISSION_RESULT type) {
		mavlink_message_t msg;

		mavlink_msg_mission_ack_pack(0, 0, &msg,
				uas->get_tgt_system(),
				uas->get_tgt_component(),
				type
				);
		uas->mav_link->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	void request_next_mission_item(void) {
		size_t seq = waypoints.size();

		if (seq < wp_count) {
			mission_request(seq);
			/* TODO start timeout */
		}
		else {
			/* retriving list done */
			mission_ack(MAV_MISSION_ACCEPTED);
			list_receiving.notify_all();
			ROS_DEBUG_NAMED("wp", "WP: mission received");
		}
	}

	bool wait_fetch_all() {
		boost::mutex cond_mutex;
		boost::unique_lock<boost::mutex> lock(cond_mutex);

		return list_receiving.timed_wait(lock, boost::posix_time::milliseconds(LIST_TIMEOUT_MS));
	}

	void publish_waypoints() {
		mavros::WaypointListPtr wpl(new mavros::WaypointList);
		boost::recursive_mutex::scoped_lock lock(mutex);

		wpl->waypoints.clear();
		wpl->waypoints.reserve(waypoints.size());
		for (auto it = waypoints.begin();
				it != waypoints.end();
				++it) {
			wpl->waypoints.push_back(WaypointItem::to_msg(*it));
		}

		lock.unlock();
		wp_list_pub.publish(wpl);
	}

	/* -*- ROS callbacks -*- */

	bool pull_cb(mavros::WaypointPull::Request &req,
			mavros::WaypointPull::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (wp_state != WP_IDLE)
			/* Wrong initial state, other operation in progress? */
			return false;

		wp_state = WP_RXLIST;
		wp_count = 0;

		lock.unlock();
		mission_request_list();
		res.success = wait_fetch_all();
		lock.lock();

		res.wp_received = waypoints.size();
		wp_state = WP_IDLE;

		lock.unlock();
		publish_waypoints();
		return true;
	}

	bool push_cb(mavros::WaypointPush::Request &req,
			mavros::WaypointPush::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);
	}

	bool clear_cb(mavros::WaypointClear::Request &req,
			mavros::WaypointClear::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);
	}

	bool set_cur_cb(mavros::WaypointSetCurrent::Request &req,
			mavros::WaypointSetCurrent::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::WaypointPlugin, mavplugin::MavRosPlugin)

