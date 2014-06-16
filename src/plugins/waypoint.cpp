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
#include <mavros/WaypointGOTO.h>

namespace mavplugin {

class WaypointItem {
public:
	uint16_t seq;
	enum MAV_FRAME frame;
	enum MAV_CMD command;
	uint8_t current; /* APM use some magical numbers */
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
		ret.is_current = !!wp.current;
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

class WaypointPlugin : public MavRosPlugin {
public:
	WaypointPlugin() :
		wp_state(WP_IDLE),
		wp_retries(RETRIES_COUNT),
		do_pull_after_gcs(false),
		reshedule_pull(false),
		BOOT_TIME_MS(15000),
		LIST_TIMEOUT_MS(30000),
		WP_TIMEOUT_MS(1000),
		RESHEDULE_MS(5000),
		RETRIES_COUNT(3)
	{
	};

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;
		wp_state = WP_IDLE;

		wp_nh = ros::NodeHandle(nh, "mission");

		wp_nh.param("pull_after_gcs", do_pull_after_gcs, false);

		wp_list_pub = wp_nh.advertise<mavros::WaypointList>("waypoints", 2, true);
		pull_srv = wp_nh.advertiseService("pull", &WaypointPlugin::pull_cb, this);
		push_srv = wp_nh.advertiseService("push", &WaypointPlugin::push_cb, this);
		clear_srv = wp_nh.advertiseService("clear", &WaypointPlugin::clear_cb, this);
		set_cur_srv = wp_nh.advertiseService("set_current", &WaypointPlugin::set_cur_cb, this);
		goto_srv = wp_nh.advertiseService("goto", &WaypointPlugin::goto_cb, this);

		wp_timer.reset(new boost::asio::deadline_timer(uas->timer_service));
		shedule_timer.reset(new boost::asio::deadline_timer(uas->timer_service));
		uas->sig_connection_changed.connect(boost::bind(&WaypointPlugin::connection_cb, this, _1));
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
	ros::ServiceServer goto_srv;

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
	boost::condition_variable list_receiving;
	boost::condition_variable list_sending;

	std::unique_ptr<boost::asio::deadline_timer> wp_timer;
	std::unique_ptr<boost::asio::deadline_timer> shedule_timer;
	bool do_pull_after_gcs;
	bool reshedule_pull;
	const int BOOT_TIME_MS;		// = 15000;
	const int LIST_TIMEOUT_MS;	// = 30000;
	const int WP_TIMEOUT_MS;	// = 1000;
	const int RESHEDULE_MS;		// = 5000;
	const int RETRIES_COUNT;	// = 3;

	/* -*- rx handlers -*- */

	void handle_mission_item(mavlink_mission_item_t &mit) {
		WaypointItem wpi = WaypointItem::from_mission_item(mit);
		boost::recursive_mutex::scoped_lock lock(mutex);

		/* receive item only in RX state */
		if (wp_state == WP_RXWP) {
			if (mit.seq != wp_cur_id) {
				ROS_WARN_NAMED("wp", "WP: Seq mismatch, dropping packet");
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
			ROS_DEBUG_NAMED("wp", "WP: rejecting message, wrong state");
			if (do_pull_after_gcs && reshedule_pull) {
				ROS_DEBUG_NAMED("wp", "WP: reshedule pull");
				shedule_pull(WP_TIMEOUT_MS);
			}
		}
	}

	void handle_mission_request(mavlink_mission_request_t &mreq) {
		boost::recursive_mutex::scoped_lock lock(mutex);
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

	void handle_mission_current(mavlink_mission_current_t &mcur) {
		boost::recursive_mutex::scoped_lock lock(mutex);

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

	void handle_mission_count(mavlink_mission_count_t &mcnt) {
		boost::recursive_mutex::scoped_lock lock(mutex);

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
				shedule_pull(RESHEDULE_MS);
			}
		}
	}

	void handle_mission_item_reached(mavlink_mission_item_reached_t &mitr) {
		/* in QGC used as informational message */
		ROS_INFO_NAMED("wp", "WP: reached #%d", mitr.seq);
	}

	void handle_mission_ack(mavlink_mission_ack_t &mack) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if ((wp_state == WP_TXLIST || wp_state == WP_TXWP)
				&& (wp_cur_id == send_waypoints.size() - 1)
				&& (mack.type == MAV_MISSION_ACCEPTED)) {

			go_idle();
			waypoints = send_waypoints;
			send_waypoints.clear();

			lock.unlock();
			list_sending.notify_all();
			publish_waypoints();
			ROS_DEBUG_NAMED("wp", "WP: mission sended");
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
				ROS_DEBUG_NAMED("wp", "WP: mission cleared");
			}

			list_sending.notify_all();
		}
		else
			ROS_DEBUG_NAMED("wp", "WP: not planned ACK, type: %d", mack.type);
	}

	/* -*- mid-level helpers -*- */

	void timeout_cb(boost::system::error_code error) {
		if (error)
			return;

		boost::recursive_mutex::scoped_lock lock(mutex);
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
			}

			restart_timeout_timer_int();
		}
		else {
			ROS_ERROR_NAMED("wp", "WP: timed out.");
			wp_state = WP_IDLE; // go_idle()
			is_timedout = true;
			/* prevent waiting cond var timeout */
			list_receiving.notify_all();
			list_sending.notify_all();
		}
	}

	void connection_cb(bool connected) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		if (connected)
			shedule_pull(BOOT_TIME_MS);
		else
			shedule_timer->cancel();
	}

	void sheduled_pull_cb(boost::system::error_code error) {
		if (error)
			return;

		boost::recursive_mutex::scoped_lock lock(mutex);
		if (wp_state != WP_IDLE) {
			/* try later */
			ROS_DEBUG_NAMED("wp", "WP: busy, reshedule pull");
			shedule_pull(RESHEDULE_MS);
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
		ROS_DEBUG_NAMED("wp", "WP: mission received");
	}

	void go_idle(void) {
		reshedule_pull = false;
		wp_state = WP_IDLE;
		wp_timer->cancel();
	}

	void restart_timeout_timer(void) {
		wp_retries = RETRIES_COUNT;
		restart_timeout_timer_int();
	}

	void restart_timeout_timer_int(void) {
		is_timedout = false;
		wp_timer->cancel();
		wp_timer->expires_from_now(boost::posix_time::milliseconds(WP_TIMEOUT_MS));
		wp_timer->async_wait(boost::bind(&WaypointPlugin::timeout_cb, this, _1));
	}

	void shedule_pull(int millis) {
		shedule_timer->cancel();
		shedule_timer->expires_from_now(boost::posix_time::milliseconds(millis));
		shedule_timer->async_wait(boost::bind(&WaypointPlugin::sheduled_pull_cb, this, _1));
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
		boost::mutex cond_mutex;
		boost::unique_lock<boost::mutex> lock(cond_mutex);

		return list_receiving.timed_wait(lock, boost::posix_time::milliseconds(LIST_TIMEOUT_MS))
			&& !is_timedout;
	}

	bool wait_push_all() {
		boost::mutex cond_mutex;
		boost::unique_lock<boost::mutex> lock(cond_mutex);

		return list_sending.timed_wait(lock, boost::posix_time::milliseconds(LIST_TIMEOUT_MS))
			&& !is_timedout;
	}

	void set_current_waypoint(size_t seq) {
		for (auto it = waypoints.begin();
				it != waypoints.end();
				++it)
			it->current = (it->seq == seq) ? true : false;
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

	/* -*- ROS callbacks -*- */

	bool pull_cb(mavros::WaypointPull::Request &req,
			mavros::WaypointPull::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);

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
		go_idle(); // not nessessary, but prevents from blocking
		return true;
	}

	bool push_cb(mavros::WaypointPush::Request &req,
			mavros::WaypointPush::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (wp_state != WP_IDLE)
			// Wrong initial state, other operation in progress?
			return false;

		wp_state = WP_TXLIST;

		send_waypoints.clear();
		send_waypoints.reserve(req.waypoints.size());
		uint16_t seq = 0;
		for (auto it = req.waypoints.begin();
				it != req.waypoints.end();
				++it, ++seq) {
			send_waypoints.push_back(WaypointItem::from_msg(*it, seq));
		}

		wp_count = send_waypoints.size();
		wp_cur_id = 0;
		restart_timeout_timer();

		lock.unlock();
		mission_count(wp_count);
		res.success = wait_push_all();
		lock.lock();

		res.wp_transfered = wp_cur_id + 1;
		go_idle(); // same as in pull_cb
		return true;
	}

	bool clear_cb(mavros::WaypointClear::Request &req,
			mavros::WaypointClear::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (wp_state != WP_IDLE)
			return false;

		wp_state = WP_CLEAR;
		restart_timeout_timer();

		lock.unlock();
		mission_clear_all();
		res.success = wait_push_all();

		lock.lock();
		go_idle(); // same as in pull_cb
		return true;
	}

	bool set_cur_cb(mavros::WaypointSetCurrent::Request &req,
			mavros::WaypointSetCurrent::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (wp_state != WP_IDLE)
			return false;

		wp_state = WP_SET_CUR;
		wp_set_active = req.wp_seq;
		restart_timeout_timer();

		lock.unlock();
		mission_set_current(wp_set_active);
		res.success = wait_push_all();

		lock.lock();
		go_idle(); // same as in pull_cb
		return true;
	}

	bool goto_cb(mavros::WaypointGOTO::Request &req,
			mavros::WaypointGOTO::Response &res) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (wp_state != WP_IDLE)
			return false;

		if (!uas->is_ardupilotmega()) {
			ROS_ERROR_NAMED("wp", "WP: FCU not support GOTO command");
			return false;
		}

		wp_state = WP_TXWP;

		WaypointItem wpi = WaypointItem::from_msg(req.waypoint, 0);
		wpi.current = 2; /* APM's magic */

		send_waypoints.clear();
		send_waypoints.push_back(wpi);

		wp_count = 1;
		wp_cur_id = 0;
		restart_timeout_timer();

		lock.unlock();
		send_waypoint(wp_cur_id);
		res.success = wait_push_all();
		lock.lock();

		go_idle(); // same as in pull_cb
		return true;
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::WaypointPlugin, mavplugin::MavRosPlugin)

