/**
 * @brief Waypoint plugin
 * @file waypoint.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>

#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>

namespace mavros {
namespace std_plugins {
using utils::enum_value;
using mavlink::common::MAV_CMD;
using mavlink::common::MAV_FRAME;
using MRES = mavlink::common::MAV_MISSION_RESULT;


class WaypointItem : public mavlink::common::msg::MISSION_ITEM {
public:
	double x_lat;
	double y_long;
	double z_alt;

	mavros_msgs::Waypoint to_msg()
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
		//     'x_lat',
		//     'y_long',
		//     'z_alt',
		// )]
		// for a, b in waypoint_item_msg:
		//     cog.outl("ret.%s = %s;" % (a, b))
		// ]]]
		ret.frame = frame;
		ret.command = command;
		ret.is_current = current;
		ret.autocontinue = autocontinue;
		ret.param1 = param1;
		ret.param2 = param2;
		ret.param3 = param3;
		ret.param4 = param4;
		ret.x_lat = x_lat;
		ret.y_long = y_long;
		ret.z_alt = z_alt;
		// [[[end]]] (checksum: 371710cb8984352c8cc1b93eb8b04a2b)

		return ret;
	}

	static WaypointItem from_msg(mavros_msgs::Waypoint &wp, uint16_t seq)
	{
		WaypointItem ret;

		// [[[cog:
		// for a, b in waypoint_item_msg:
		//     cog.outl("ret.%s = wp.%s;" % (b, a))
		// ]]]
		ret.frame = wp.frame;
		ret.command = wp.command;
		ret.current = wp.is_current;
		ret.autocontinue = wp.autocontinue;
		ret.param1 = wp.param1;
		ret.param2 = wp.param2;
		ret.param3 = wp.param3;
		ret.param4 = wp.param4;
		ret.x_lat = wp.x_lat;
		ret.y_long = wp.y_long;
		ret.z_alt = wp.z_alt;
		// [[[end]]] (checksum: 14cc0f2fc12a4f95f6ea200e41005e3b)

		ret.seq = seq;
		ret.x = ret.x_lat;
		ret.y = ret.y_long;
		ret.z = ret.z_alt;

		return ret;
	}

	std::string to_string()
	{
		//return to_yaml();

		return utils::format("#%u%1s F:%u C:%3u p: %f %f %f %f x: %f y: %f z: %f",
				seq,
				(current) ? "*" : "",
				frame, command,
				param1, param2, param3, param4,
				x_lat, y_long, z_alt);
	}
};


/**
 * @brief Mission manupulation plugin
 */
class WaypointPlugin : public plugin::PluginBase {
public:
	WaypointPlugin() : PluginBase(),
		wp_nh("~mission"),
		wp_state(WP::IDLE),
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
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		wp_state = WP::IDLE;

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
		enable_connection_cb();
	}

	Subscriptions get_subscriptions() {
		return {
		       make_handler(&WaypointPlugin::handle_mission_item),
		       make_handler(&WaypointPlugin::handle_mission_request),
		       make_handler(&WaypointPlugin::handle_mission_current),
		       make_handler(&WaypointPlugin::handle_mission_count),
		       make_handler(&WaypointPlugin::handle_mission_item_reached),
		       make_handler(&WaypointPlugin::handle_mission_ack),
		};
	}

private:
	using unique_lock = std::unique_lock<std::recursive_mutex>;
	using lock_guard = std::lock_guard<std::recursive_mutex>;

	std::recursive_mutex mutex;
	ros::NodeHandle wp_nh;

	ros::Publisher wp_list_pub;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer clear_srv;
	ros::ServiceServer set_cur_srv;

	std::vector<WaypointItem> waypoints;
	std::vector<WaypointItem> send_waypoints;
	enum class WP {
		IDLE,
		RXLIST,
		RXWP,
		TXLIST,
		TXWP,
		CLEAR,
		SET_CUR
	};
	WP wp_state;

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

	// WaypointItem is MISSION_ITEM child
	void handle_mission_item(const mavlink::mavlink_message_t *msg, WaypointItem &wpi)
	{
		unique_lock lock(mutex);

		// WaypointItem has wider fields for Lat/Long/Alt, set it
		// [[[cog:
		// waypoint_mission_item = (('x_lat', 'x'), ('y_long', 'y'), ('z_alt', 'z'))
		// for a, b in waypoint_mission_item:
		//     cog.outl("wpi.%s = wpi.%s;" % (a, b))
		// ]]]
		wpi.x_lat = wpi.x;
		wpi.y_long = wpi.y;
		wpi.z_alt = wpi.z;
		// [[[end]]] (checksum: b8f95ce9c7c9dbd4eb493bf1227f273f)

		/* receive item only in RX state */
		if (wp_state == WP::RXWP) {
			if (wpi.seq != wp_cur_id) {
				ROS_WARN_NAMED("wp", "WP: Seq mismatch, dropping item (%d != %zu)",
						wpi.seq, wp_cur_id);
				return;
			}

			ROS_INFO_STREAM_NAMED("wp", "WP: item " << wpi.to_string());

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
			ROS_DEBUG_NAMED("wp", "WP: rejecting item, wrong state %d", enum_value(wp_state));
			if (do_pull_after_gcs && reshedule_pull) {
				ROS_DEBUG_NAMED("wp", "WP: reshedule pull");
				shedule_pull(WP_TIMEOUT_DT);
			}
		}
	}

	void handle_mission_request(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_REQUEST &mreq)
	{
		lock_guard lock(mutex);

		if ((wp_state == WP::TXLIST && mreq.seq == 0) || (wp_state == WP::TXWP)) {
			if (mreq.seq != wp_cur_id && mreq.seq != wp_cur_id + 1) {
				ROS_WARN_NAMED("wp", "WP: Seq mismatch, dropping request (%d != %zu)",
						mreq.seq, wp_cur_id);
				return;
			}

			restart_timeout_timer();
			if (mreq.seq < send_waypoints.size()) {
				wp_state = WP::TXWP;
				wp_cur_id = mreq.seq;
				send_waypoint(wp_cur_id);
			}
			else
				ROS_ERROR_NAMED("wp", "WP: FCU require seq out of range");
		}
		else
			ROS_DEBUG_NAMED("wp", "WP: rejecting request, wrong state %d", enum_value(wp_state));
	}

	void handle_mission_current(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_CURRENT &mcur)
	{
		unique_lock lock(mutex);

		if (wp_state == WP::SET_CUR) {
			/* MISSION_SET_CURRENT ACK */
			ROS_DEBUG_NAMED("wp", "WP: set current #%d done", mcur.seq);
			go_idle();
			wp_cur_active = mcur.seq;
			set_current_waypoint(wp_cur_active);

			lock.unlock();
			list_sending.notify_all();
			publish_waypoints();
		}
		else if (wp_state == WP::IDLE && wp_cur_active != mcur.seq) {
			/* update active */
			ROS_DEBUG_NAMED("wp", "WP: update current #%d", mcur.seq);
			wp_cur_active = mcur.seq;
			set_current_waypoint(wp_cur_active);

			lock.unlock();
			publish_waypoints();
		}
	}

	void handle_mission_count(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_COUNT &mcnt)
	{
		unique_lock lock(mutex);

		if (wp_state == WP::RXLIST) {
			/* FCU report of MISSION_REQUEST_LIST */
			ROS_DEBUG_NAMED("wp", "WP: count %d", mcnt.count);

			wp_count = mcnt.count;
			wp_cur_id = 0;

			waypoints.clear();
			waypoints.reserve(wp_count);

			if (wp_count > 0) {
				wp_state = WP::RXWP;
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

	void handle_mission_item_reached(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_ITEM_REACHED &mitr)
	{
		/* in QGC used as informational message */
		ROS_INFO_NAMED("wp", "WP: reached #%d", mitr.seq);
	}

	void handle_mission_ack(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_ACK &mack)
	{
		unique_lock lock(mutex);

		if ((wp_state == WP::TXLIST || wp_state == WP::TXWP)
				&& (wp_cur_id == send_waypoints.size() - 1)
				&& (mack.type == enum_value(MRES::ACCEPTED))) {
			go_idle();
			waypoints = send_waypoints;
			send_waypoints.clear();

			lock.unlock();
			list_sending.notify_all();
			publish_waypoints();
			ROS_INFO_NAMED("wp", "WP: mission sended");
		}
		else if (wp_state == WP::TXLIST || wp_state == WP::TXWP) {
			go_idle();
			/* use this flag for failure report */
			is_timedout = true;
			lock.unlock();
			list_sending.notify_all();

			switch (mack.type) {
			// XXX add to_string for enum
			// [[[cog:
			// mission_ack_log = (
			//     ('ERROR', "general error"),
			//     ('UNSUPPORTED_FRAME', "unsupported frame"),
			//     ('UNSUPPORTED', "command unsupported"),
			//     ('NO_SPACE', "no space left on mission storage"),
			//     ('DENIED', "denied"),
			// )
			// for k, v in mission_ack_log:
			//     cog.outl("case enum_value(MRES::%s):" % k)
			//     cog.outl("""\tROS_ERROR_NAMED("wp", "WP: upload failed: %s");""" % v)
			//     cog.outl("\tbreak;")
			// ]]]
			case enum_value(MRES::ERROR):
				ROS_ERROR_NAMED("wp", "WP: upload failed: general error");
				break;
			case enum_value(MRES::UNSUPPORTED_FRAME):
				ROS_ERROR_NAMED("wp", "WP: upload failed: unsupported frame");
				break;
			case enum_value(MRES::UNSUPPORTED):
				ROS_ERROR_NAMED("wp", "WP: upload failed: command unsupported");
				break;
			case enum_value(MRES::NO_SPACE):
				ROS_ERROR_NAMED("wp", "WP: upload failed: no space left on mission storage");
				break;
			case enum_value(MRES::DENIED):
				ROS_ERROR_NAMED("wp", "WP: upload failed: denied");
				break;
			// [[[end]]] (checksum: b85e3c2b5705b79375cefa3ff3fa108b)

			default:
				ROS_ERROR_NAMED("wp", "WP: upload failed: error #%d", mack.type);
				break;
			}
		}
		else if (wp_state == WP::CLEAR) {
			go_idle();
			if (mack.type != enum_value(MRES::ACCEPTED)) {
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

	void timeout_cb(const ros::TimerEvent &event)
	{
		unique_lock lock(mutex);
		if (wp_retries > 0) {
			wp_retries--;
			ROS_WARN_NAMED("wp", "WP: timeout, retries left %zu", wp_retries);

			switch (wp_state) {
			case WP::RXLIST:
				mission_request_list();
				break;
			case WP::RXWP:
				mission_request(wp_cur_id);
				break;
			case WP::TXLIST:
				mission_count(wp_count);
				break;
			case WP::TXWP:
				send_waypoint(wp_cur_id);
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
			ROS_ERROR_NAMED("wp", "WP: timed out.");
			go_idle();
			is_timedout = true;
			/* prevent waiting cond var timeout */
			lock.unlock();
			list_receiving.notify_all();
			list_sending.notify_all();
		}
	}

	void connection_cb(bool connected) override
	{
		lock_guard lock(mutex);
		if (connected)
			shedule_pull(BOOTUP_TIME_DT);
		else
			shedule_timer.stop();
	}

	void sheduled_pull_cb(const ros::TimerEvent &event)
	{
		lock_guard lock(mutex);
		if (wp_state != WP::IDLE) {
			/* try later */
			ROS_DEBUG_NAMED("wp", "WP: busy, reshedule pull");
			shedule_pull(RESHEDULE_DT);
			return;
		}

		ROS_DEBUG_NAMED("wp", "WP: start sheduled pull");
		wp_state = WP::RXLIST;
		wp_count = 0;
		restart_timeout_timer();
		mission_request_list();
	}

	void request_mission_done(void)
	{
		/* possibly not needed if count == 0 (QGC impl) */
		mission_ack(MRES::ACCEPTED);

		go_idle();
		list_receiving.notify_all();
		ROS_INFO_NAMED("wp", "WP: mission received");
	}

	void go_idle(void)
	{
		reshedule_pull = false;
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

	void shedule_pull(const ros::Duration &dt)
	{
		shedule_timer.stop();
		shedule_timer.setPeriod(dt);
		shedule_timer.start();
	}

	void send_waypoint(size_t seq)
	{
		if (seq < send_waypoints.size()) {
			auto wpi = send_waypoints.at(seq);
			mission_item(wpi);

			ROS_DEBUG_STREAM_NAMED("wp", "WP: send item " << wpi.to_string());
		}
	}

	bool wait_fetch_all()
	{
		std::unique_lock<std::mutex> lock(recv_cond_mutex);

		return list_receiving.wait_for(lock, std::chrono::nanoseconds(LIST_TIMEOUT_DT.toNSec()))
		       == std::cv_status::no_timeout
		       && !is_timedout;
	}

	bool wait_push_all()
	{
		std::unique_lock<std::mutex> lock(send_cond_mutex);

		return list_sending.wait_for(lock, std::chrono::nanoseconds(LIST_TIMEOUT_DT.toNSec()))
		       == std::cv_status::no_timeout
		       && !is_timedout;
	}

	void set_current_waypoint(size_t seq)
	{
		for (auto &it : waypoints)
			it.current = (it.seq == seq) ? true : false;
	}

	void publish_waypoints()
	{
		auto wpl = boost::make_shared<mavros_msgs::WaypointList>();
		unique_lock lock(mutex);

		wpl->waypoints.clear();
		wpl->waypoints.reserve(waypoints.size());
		for (auto &it : waypoints) {
			wpl->waypoints.push_back(it.to_msg());
		}

		lock.unlock();
		wp_list_pub.publish(wpl);
	}

	/* -*- low-level send functions -*- */

	void mission_item(WaypointItem &wp)
	{
		m_uas->msg_set_target(wp);

		// WaypointItem may be sent as MISSION_ITEM
		UAS_FCU(m_uas)->send_message_ignore_drop(wp);
	}

	void mission_request(uint16_t seq)
	{
		ROS_DEBUG_NAMED("wp", "WP:m: request #%u", seq);

		mavlink::common::msg::MISSION_REQUEST mrq{};
		m_uas->msg_set_target(mrq);
		mrq.seq = seq;

		UAS_FCU(m_uas)->send_message_ignore_drop(mrq);
	}

	void mission_set_current(uint16_t seq)
	{
		ROS_DEBUG_NAMED("wp", "WP:m: set current #%u", seq);

		mavlink::common::msg::MISSION_SET_CURRENT msc{};
		m_uas->msg_set_target(msc);
		msc.seq = seq;

		UAS_FCU(m_uas)->send_message_ignore_drop(msc);
	}

	void mission_request_list()
	{
		ROS_DEBUG_NAMED("wp", "WP:m: request list");

		mavlink::common::msg::MISSION_REQUEST_LIST mrl{};
		m_uas->msg_set_target(mrl);

		UAS_FCU(m_uas)->send_message_ignore_drop(mrl);
	}

	void mission_count(uint16_t cnt)
	{
		ROS_DEBUG_NAMED("wp", "WP:m: count %u", cnt);

		mavlink::common::msg::MISSION_COUNT mcnt{};
		m_uas->msg_set_target(mcnt);
		mcnt.count = cnt;

		UAS_FCU(m_uas)->send_message_ignore_drop(mcnt);
	}

	void mission_clear_all()
	{
		ROS_DEBUG_NAMED("wp", "WP:m: clear all");

		mavlink::common::msg::MISSION_CLEAR_ALL mclr{};
		m_uas->msg_set_target(mclr);

		UAS_FCU(m_uas)->send_message_ignore_drop(mclr);
	}

	void mission_ack(MRES type) {
		ROS_DEBUG_NAMED("wp", "WP:m: ACK %u", enum_value(type));

		mavlink::common::msg::MISSION_ACK mack{};
		m_uas->msg_set_target(mack);
		mack.type = enum_value(type);

		UAS_FCU(m_uas)->send_message_ignore_drop(mack);
	}

	/* -*- ROS callbacks -*- */

	bool pull_cb(mavros_msgs::WaypointPull::Request &req,
			mavros_msgs::WaypointPull::Response &res)
	{
		unique_lock lock(mutex);

		if (wp_state != WP::IDLE)
			// Wrong initial state, other operation in progress?
			return false;

		wp_state = WP::RXLIST;
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
			mavros_msgs::WaypointPush::Response &res)
	{
		unique_lock lock(mutex);

		if (wp_state != WP::IDLE)
			// Wrong initial state, other operation in progress?
			return false;

		wp_state = WP::TXLIST;

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
			mavros_msgs::WaypointClear::Response &res)
	{
		unique_lock lock(mutex);

		if (wp_state != WP::IDLE)
			return false;

		wp_state = WP::CLEAR;
		restart_timeout_timer();

		lock.unlock();
		mission_clear_all();
		res.success = wait_push_all();

		lock.lock();
		go_idle();	// same as in pull_cb
		return true;
	}

	bool set_cur_cb(mavros_msgs::WaypointSetCurrent::Request &req,
			mavros_msgs::WaypointSetCurrent::Response &res)
	{
		unique_lock lock(mutex);

		if (wp_state != WP::IDLE)
			return false;

		wp_state = WP::SET_CUR;
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
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WaypointPlugin, mavros::plugin::PluginBase)
