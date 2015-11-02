/**
 * @brief Waypoint plugin
 * @file waypoint.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>

namespace mavplugin {
class WaypointItem {
public:
	uint16_t seq;
	enum MAV_FRAME frame;
	enum MAV_CMD command;
	uint8_t current;/* APM use some magical numbers */
	bool autocontinue;
	float param1;
	float param2;
	float param3;
	float param4;
	double x_lat;
	double y_long;
	double z_alt;

	static mavros_msgs::Waypoint to_msg(WaypointItem &wp) {
		mavros_msgs::Waypoint ret;

		ret.frame = static_cast<uint8_t>(wp.frame);
		ret.command = static_cast<uint16_t>(wp.command);
		ret.is_current = !!wp.current;
		ret.autocontinue = wp.autocontinue;
		ret.param1 = wp.param1;
		ret.param2 = wp.param2;
		ret.param3 = wp.param3;
		ret.param4 = wp.param4;
		ret.x_lat = wp.x_lat;
		ret.y_long = wp.y_long;
		ret.z_alt = wp.z_alt;

		return ret;
	}

	static WaypointItem from_msg(mavros_msgs::Waypoint &wp, uint16_t seq) {
		WaypointItem ret;

		ret.seq = seq;
		ret.frame = static_cast<enum MAV_FRAME>(wp.frame);
		ret.command = static_cast<enum MAV_CMD>(wp.command);
		ret.current = wp.is_current;
		ret.autocontinue = wp.autocontinue;
		ret.param1 = wp.param1;
		ret.param2 = wp.param2;
		ret.param3 = wp.param3;
		ret.param4 = wp.param4;
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
		ret.param2 = mit.param2;
		ret.param3 = mit.param3;
		ret.param4 = mit.param4;
		ret.x_lat = mit.x;
		ret.y_long = mit.y;
		ret.z_alt = mit.z;

		return ret;
	}

	static std::string to_string_frame(WaypointItem &wpi) {
		switch (wpi.frame) {
		case MAV_FRAME_GLOBAL:		return "GAA";
		case MAV_FRAME_LOCAL_NED:	return "LNED";
		case MAV_FRAME_MISSION:		return "MIS";
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:	return "GRA";
		case MAV_FRAME_LOCAL_ENU:	return "LENU";
		default:
			std::ostringstream unk;
			unk << "UNK " << (int)wpi.frame;
			return unk.str();
		}
	}

	static std::string to_string_command(WaypointItem &wpi) {
		switch (wpi.command) {
		case MAV_CMD_NAV_WAYPOINT:	return "WAYPOINT";
		case MAV_CMD_NAV_LOITER_UNLIM:	return "LOITER UNLIM";
		case MAV_CMD_NAV_LOITER_TURNS:	return "LOITER TURNS";
		case MAV_CMD_NAV_LOITER_TIME:	return "LOITER TIME";
		case MAV_CMD_NAV_RETURN_TO_LAUNCH:	return "RTL";
		case MAV_CMD_NAV_LAND:		return "LAND";
		case MAV_CMD_NAV_TAKEOFF:	return "TAKEOFF";
		case MAV_CMD_NAV_ROI:		return "ROI";
		case MAV_CMD_NAV_PATHPLANNING:	return "PATH PLANNING";
		default:
			std::ostringstream unk;
			unk << "UNK " << (int)wpi.command;
			return unk.str();
		}
	}
};


/**
 * @brief Mission manupulation plugin
 */
class WaypointPlugin : public MavRosPlugin {
public:
	WaypointPlugin() :
		wp_nh("~mission"),
		uas(nullptr),
		wp_state(WP_IDLE),
		wp_count(0),
		wp_retries(RETRIES_COUNT),
		wp_cur_id(0),
		wp_cur_active(0),
		wp_set_active(0),
		is_timedout(false),
		do_pull_after_gcs(false),
		reshedule_pull(false),
		BOOTUP_TIME_DT(BOOTUP_TIME_MS / 1000.0),
		LIST_TIMEOUT_DT(LIST_TIMEOUT_MS / 1000.0),
		WP_TIMEOUT_DT(WP_TIMEOUT_MS / 1000.0),
		RESHEDULE_DT(RESHEDULE_MS / 1000.0)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;
		wp_state = WP_IDLE;

		wp_nh.param("pull_after_gcs", do_pull_after_gcs, false);

		wp_list_pub = wp_nh.advertise<mavros_msgs::WaypointList>("waypoints", 2, true);
		pull_srv = wp_nh.advertiseService("pull", &WaypointPlugin::pull_cb, this);
		push_srv = wp_nh.advertiseService("push", &WaypointPlugin::push_cb, this);
		clear_srv = wp_nh.advertiseService("clear", &WaypointPlugin::clear_cb, this);
		set_cur_srv = wp_nh.advertiseService("set_current", &WaypointPlugin::set_cur_cb, this);

		wp_timer = wp_nh.createTimer(WP_TIMEOUT_DT, &WaypointPlugin::timeout_cb, this, true);
		wp_timer.stop();
		shedule_timer = wp_nh.createTimer(BOOTUP_TIME_DT, &WaypointPlugin::sheduled_pull_cb, this, true);
		shedule_timer.stop();
		uas->sig_connection_changed.connect(boost::bind(&WaypointPlugin::connection_cb, this, _1));
	};

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_MISSION_ITEM, &WaypointPlugin::handle_mission_item),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_MISSION_REQUEST, &WaypointPlugin::handle_mission_request),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_MISSION_CURRENT, &WaypointPlugin::handle_mission_current),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_MISSION_COUNT, &WaypointPlugin::handle_mission_count),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_MISSION_ITEM_REACHED, &WaypointPlugin::handle_mission_item_reached),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_MISSION_ACK, &WaypointPlugin::handle_mission_ack),
		};
	}

private:
	std::recursive_mutex mutex;
	ros::NodeHandle wp_nh;
	UAS *uas;

	ros::Publisher wp_list_pub;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer clear_srv;
	ros::ServiceServer set_cur_srv;

	std::vector<WaypointItem> waypoints;
	std::vector<WaypointItem> send_waypoints;
	enum {
		WP_IDLE,
		WP_RXLIST,
		WP_RXWP,
		WP_TXLIST,
		WP_TXWP,
		WP_CLEAR,
		WP_SET_CUR
	} wp_state;

	size_t wp_count;
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
	ros::Timer shedule_timer;
	bool do_pull_after_gcs;
	bool reshedule_pull;

	static constexpr int BOOTUP_TIME_MS = 15000;	//! system startup delay before start pull
	static constexpr int LIST_TIMEOUT_MS = 30000;	//! Timeout for pull/push operations
	static constexpr int WP_TIMEOUT_MS = 1000;
	static constexpr int RESHEDULE_MS = 5000;
	static constexpr int RETRIES_COUNT = 3;

	const ros::Duration BOOTUP_TIME_DT;
	const ros::Duration LIST_TIMEOUT_DT;
	const ros::Duration WP_TIMEOUT_DT;
	const ros::Duration RESHEDULE_DT;

	/* -*- rx handlers -*- */

	void handle_mission_item(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_mission_item_t mit;
		mavlink_msg_mission_item_decode(msg, &mit);
		WaypointItem wpi = WaypointItem::from_mission_item(mit);
		unique_lock lock(mutex);

		/* receive item only in RX state */
		if (wp_state == WP_RXWP) {
			if (mit.seq != wp_cur_id) {
				ROS_WARN_NAMED("wp", "WP: Seq mismatch, dropping item (%d != %zu)",
						mit.seq, wp_cur_id);
				return;
			}

			ROS_INFO_STREAM_NAMED("wp", "WP: item #" << wpi.seq <<
					" " << WaypointItem::to_string_frame(wpi) <<
					" " << WaypointItem::to_string_command(wpi) <<
					((wpi.current) ? " CUR" : "    ") <<
					" params: " << wpi.param1 <<
					", " << wpi.param2 <<
					", " << wpi.param3 <<
					", " << wpi.param4 <<
					" x: " << wpi.x_lat <<
					" y: " << wpi.y_long <<
					" z: " << wpi.z_alt);

			waypoints.push_back(wpi);
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
			ROS_DEBUG_NAMED("wp", "WP: rejecting item, wrong state %d", wp_state);
			if (do_pull_after_gcs && reshedule_pull) {
				ROS_DEBUG_NAMED("wp", "WP: reshedule pull");
				shedule_pull(WP_TIMEOUT_DT);
			}
		}
	}

	void handle_mission_request(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_mission_request_t mreq;
		mavlink_msg_mission_request_decode(msg, &mreq);
		lock_guard lock(mutex);

		if ((wp_state == WP_TXLIST && mreq.seq == 0) || (wp_state == WP_TXWP)) {
			if (mreq.seq != wp_cur_id && mreq.seq != wp_cur_id + 1) {
				ROS_WARN_NAMED("wp", "WP: Seq mismatch, dropping request (%d != %zu)",
						mreq.seq, wp_cur_id);
				return;
			}

			restart_timeout_timer();
			if (mreq.seq < send_waypoints.size()) {
				wp_state = WP_TXWP;
				wp_cur_id = mreq.seq;
				send_waypoint(wp_cur_id);
			}
			else
				ROS_ERROR_NAMED("wp", "WP: FCU require seq out of range");
		}
		else
			ROS_DEBUG_NAMED("wp", "WP: rejecting request, wrong state %d", wp_state);
	}

	void handle_mission_current(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_mission_current_t mcur;
		mavlink_msg_mission_current_decode(msg, &mcur);
		unique_lock lock(mutex);

		if (wp_state == WP_SET_CUR) {
			/* MISSION_SET_CURRENT ACK */
			ROS_DEBUG_NAMED("wp", "WP: set current #%d done", mcur.seq);
			go_idle();
			wp_cur_active = mcur.seq;
			set_current_waypoint(wp_cur_active);

			lock.unlock();
			list_sending.notify_all();
			publish_waypoints();
		}
		else if (wp_state == WP_IDLE && wp_cur_active != mcur.seq) {
			/* update active */
			ROS_DEBUG_NAMED("wp", "WP: update current #%d", mcur.seq);
			wp_cur_active = mcur.seq;
			set_current_waypoint(wp_cur_active);

			lock.unlock();
			publish_waypoints();
		}
	}

	void handle_mission_count(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_mission_count_t mcnt;
		mavlink_msg_mission_count_decode(msg, &mcnt);
		unique_lock lock(mutex);

		if (wp_state == WP_RXLIST) {
			/* FCU report of MISSION_REQUEST_LIST */
			ROS_DEBUG_NAMED("wp", "WP: count %d", mcnt.count);

			wp_count = mcnt.count;
			wp_cur_id = 0;

			waypoints.clear();
			waypoints.reserve(wp_count);

			if (wp_count > 0) {
				wp_state = WP_RXWP;
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
			ROS_INFO_NAMED("wp", "WP: seems GCS requesting mission");
			/* shedule pull after GCS done */
			if (do_pull_after_gcs) {
				ROS_INFO_NAMED("wp", "WP: sheduling pull after GCS is done");
				reshedule_pull = true;
				shedule_pull(RESHEDULE_DT);
			}
		}
	}

	void handle_mission_item_reached(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_mission_item_reached_t mitr;
		mavlink_msg_mission_item_reached_decode(msg, &mitr);

		/* in QGC used as informational message */
		ROS_INFO_NAMED("wp", "WP: reached #%d", mitr.seq);
	}

	void handle_mission_ack(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_mission_ack_t mack;
		mavlink_msg_mission_ack_decode(msg, &mack);
		unique_lock lock(mutex);

		if ((wp_state == WP_TXLIST || wp_state == WP_TXWP)
				&& (wp_cur_id == send_waypoints.size() - 1)
				&& (mack.type == MAV_MISSION_ACCEPTED)) {
			go_idle();
			waypoints = send_waypoints;
			send_waypoints.clear();

			lock.unlock();
			list_sending.notify_all();
			publish_waypoints();
			ROS_INFO_NAMED("wp", "WP: mission sended");
		}
		else if (wp_state == WP_TXLIST || wp_state == WP_TXWP) {
			go_idle();
			/* use this flag for failure report */
			is_timedout = true;
			lock.unlock();
			list_sending.notify_all();

			switch (mack.type) {
			case MAV_MISSION_ERROR:
				ROS_ERROR_NAMED("wp", "WP: upload failed: general error");
				break;

			case MAV_MISSION_UNSUPPORTED_FRAME:
				ROS_ERROR_NAMED("wp", "WP: upload failed: unsupported frame");
				break;

			case MAV_MISSION_UNSUPPORTED:
				ROS_ERROR_NAMED("wp", "WP: upload failed: command unsupported");
				break;

			case MAV_MISSION_NO_SPACE:
				ROS_ERROR_NAMED("wp", "WP: upload failed: no space left on mission storage");
				break;

			case MAV_MISSION_DENIED:
				ROS_ERROR_NAMED("wp", "WP: upload failed: denied");
				break;

			default:
				ROS_ERROR_NAMED("wp", "WP: upload failed: error #%d", mack.type);
				break;
			}
		}
		else if (wp_state == WP_CLEAR) {
			go_idle();
			if (mack.type != MAV_MISSION_ACCEPTED) {
				is_timedout = true;
				lock.unlock();
				ROS_ERROR_NAMED("wp", "WP: clear failed: error #%d", mack.type);
			}
			else {
				waypoints.clear();
				lock.unlock();
				publish_waypoints();
				ROS_INFO_NAMED("wp", "WP: mission cleared");
			}

			list_sending.notify_all();
		}
		else
			ROS_DEBUG_NAMED("wp", "WP: not planned ACK, type: %d", mack.type);
	}

	/* -*- mid-level helpers -*- */

	void timeout_cb(const ros::TimerEvent &event) {
		unique_lock lock(mutex);
		if (wp_retries > 0) {
			wp_retries--;
			ROS_WARN_NAMED("wp", "WP: timeout, retries left %zu", wp_retries);

			switch (wp_state) {
			case WP_RXLIST:
				mission_request_list();
				break;
			case WP_RXWP:
				mission_request(wp_cur_id);
				break;
			case WP_TXLIST:
				mission_count(wp_count);
				break;
			case WP_TXWP:
				send_waypoint(wp_cur_id);
				break;
			case WP_CLEAR:
				mission_clear_all();
				break;
			case WP_SET_CUR:
				mission_set_current(wp_set_active);
				break;

			case WP_IDLE:
				break;
			}

			restart_timeout_timer_int();
		}
		else {
			ROS_ERROR_NAMED("wp", "WP: timed out.");
			go_idle();
			is_timedout = true;
			/* prevent waiting cond var timeout */
			lock.unlock();
			list_receiving.notify_all();
			list_sending.notify_all();
		}
	}

	void connection_cb(bool connected) {
		lock_guard lock(mutex);
		if (connected)
			shedule_pull(BOOTUP_TIME_DT);
		else
			shedule_timer.stop();
	}

	void sheduled_pull_cb(const ros::TimerEvent &event) {
		lock_guard lock(mutex);
		if (wp_state != WP_IDLE) {
			/* try later */
			ROS_DEBUG_NAMED("wp", "WP: busy, reshedule pull");
			shedule_pull(RESHEDULE_DT);
			return;
		}

		ROS_DEBUG_NAMED("wp", "WP: start sheduled pull");
		wp_state = WP_RXLIST;
		wp_count = 0;
		restart_timeout_timer();
		mission_request_list();
	}

	void request_mission_done(void) {
		/* possibly not needed if count == 0 (QGC impl) */
		mission_ack(MAV_MISSION_ACCEPTED);

		go_idle();
		list_receiving.notify_all();
		ROS_INFO_NAMED("wp", "WP: mission received");
	}

	void go_idle(void) {
		reshedule_pull = false;
		wp_state = WP_IDLE;
		wp_timer.stop();
	}

	void restart_timeout_timer(void) {
		wp_retries = RETRIES_COUNT;
		restart_timeout_timer_int();
	}

	void restart_timeout_timer_int(void) {
		is_timedout = false;
		wp_timer.stop();
		wp_timer.start();
	}

	void shedule_pull(const ros::Duration &dt) {
		shedule_timer.stop();
		shedule_timer.setPeriod(dt);
		shedule_timer.start();
	}

	void send_waypoint(size_t seq) {
		if (seq < send_waypoints.size()) {
			WaypointItem wpi = send_waypoints.at(seq);
			mission_item(wpi);

			ROS_DEBUG_STREAM_NAMED("wp", "WP: send item #" << wpi.seq <<
					" " << WaypointItem::to_string_frame(wpi) <<
					" " << WaypointItem::to_string_command(wpi) <<
					((wpi.current) ? " CUR" : "    ") <<
					" params: " << wpi.param1 <<
					", " << wpi.param2 <<
					", " << wpi.param3 <<
					", " << wpi.param4 <<
					" x: " << wpi.x_lat <<
					" y: " << wpi.y_long <<
					" z: " << wpi.z_alt);
		}
	}

	bool wait_fetch_all() {
		std::unique_lock<std::mutex> lock(recv_cond_mutex);

		return list_receiving.wait_for(lock, std::chrono::nanoseconds(LIST_TIMEOUT_DT.toNSec()))
		       == std::cv_status::no_timeout
		       && !is_timedout;
	}

	bool wait_push_all() {
		std::unique_lock<std::mutex> lock(send_cond_mutex);

		return list_sending.wait_for(lock, std::chrono::nanoseconds(LIST_TIMEOUT_DT.toNSec()))
		       == std::cv_status::no_timeout
		       && !is_timedout;
	}

	void set_current_waypoint(size_t seq) {
		for (auto &it : waypoints)
			it.current = (it.seq == seq) ? true : false;
	}

	void publish_waypoints() {
		auto wpl = boost::make_shared<mavros_msgs::WaypointList>();
		unique_lock lock(mutex);

		wpl->waypoints.clear();
		wpl->waypoints.reserve(waypoints.size());
		for (auto &it : waypoints) {
			wpl->waypoints.push_back(WaypointItem::to_msg(it));
		}

		lock.unlock();
		wp_list_pub.publish(wpl);
	}

	/* -*- low-level send functions -*- */

	void mission_item(WaypointItem &wp) {
		mavlink_message_t msg;

		mavlink_msg_mission_item_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
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
		UAS_FCU(uas)->send_message(&msg);
	}

	void mission_request(uint16_t seq) {
		mavlink_message_t msg;

		ROS_DEBUG_NAMED("wp", "WP:m: request #%u", seq);
		mavlink_msg_mission_request_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				seq
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	void mission_set_current(uint16_t seq) {
		mavlink_message_t msg;

		ROS_DEBUG_NAMED("wp", "WP:m: set current #%u", seq);
		mavlink_msg_mission_set_current_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				seq
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	void mission_request_list() {
		mavlink_message_t msg;

		ROS_DEBUG_NAMED("wp", "WP:m: request list");
		mavlink_msg_mission_request_list_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas)
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	void mission_count(uint16_t cnt) {
		mavlink_message_t msg;

		ROS_DEBUG_NAMED("wp", "WP:m: count %u", cnt);
		mavlink_msg_mission_count_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				cnt
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	void mission_clear_all() {
		mavlink_message_t msg;

		ROS_DEBUG_NAMED("wp", "WP:m: clear all");
		mavlink_msg_mission_clear_all_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas)
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	void mission_ack(enum MAV_MISSION_RESULT type) {
		mavlink_message_t msg;

		ROS_DEBUG_NAMED("wp", "WP:m: ACK %u", type);
		mavlink_msg_mission_ack_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				type
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- ROS callbacks -*- */

	bool pull_cb(mavros_msgs::WaypointPull::Request &req,
			mavros_msgs::WaypointPull::Response &res) {
		unique_lock lock(mutex);

		if (wp_state != WP_IDLE)
			// Wrong initial state, other operation in progress?
			return false;

		wp_state = WP_RXLIST;
		wp_count = 0;
		restart_timeout_timer();

		lock.unlock();
		mission_request_list();
		res.success = wait_fetch_all();
		lock.lock();

		res.wp_received = waypoints.size();
		go_idle();	// not nessessary, but prevents from blocking
		return true;
	}

	bool push_cb(mavros_msgs::WaypointPush::Request &req,
			mavros_msgs::WaypointPush::Response &res) {
		unique_lock lock(mutex);

		if (wp_state != WP_IDLE)
			// Wrong initial state, other operation in progress?
			return false;

		wp_state = WP_TXLIST;

		send_waypoints.clear();
		send_waypoints.reserve(req.waypoints.size());
		uint16_t seq = 0;
		for (auto &it : req.waypoints) {
			send_waypoints.push_back(WaypointItem::from_msg(it, seq++));
		}

		wp_count = send_waypoints.size();
		wp_cur_id = 0;
		restart_timeout_timer();

		lock.unlock();
		mission_count(wp_count);
		res.success = wait_push_all();
		lock.lock();

		res.wp_transfered = wp_cur_id + 1;
		go_idle();	// same as in pull_cb
		return true;
	}

	bool clear_cb(mavros_msgs::WaypointClear::Request &req,
			mavros_msgs::WaypointClear::Response &res) {
		unique_lock lock(mutex);

		if (wp_state != WP_IDLE)
			return false;

		wp_state = WP_CLEAR;
		restart_timeout_timer();

		lock.unlock();
		mission_clear_all();
		res.success = wait_push_all();

		lock.lock();
		go_idle();	// same as in pull_cb
		return true;
	}

	bool set_cur_cb(mavros_msgs::WaypointSetCurrent::Request &req,
			mavros_msgs::WaypointSetCurrent::Response &res) {
		unique_lock lock(mutex);

		if (wp_state != WP_IDLE)
			return false;

		wp_state = WP_SET_CUR;
		wp_set_active = req.wp_seq;
		restart_timeout_timer();

		lock.unlock();
		mission_set_current(wp_set_active);
		res.success = wait_push_all();

		lock.lock();
		go_idle();	// same as in pull_cb
		return true;
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::WaypointPlugin, mavplugin::MavRosPlugin)

