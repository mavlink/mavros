/**
 * @brief Waypoint plugin
 * @file waypoint.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016,2017,2018 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mission_protocol_base.h>

#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/WaypointReached.h>


namespace mavros {
namespace std_plugins {
/**
 * @brief Mission manupulation plugin
 */
class WaypointPlugin : public plugin::MissionBase {
public:
	WaypointPlugin() :
		MissionBase("WP"),
		wp_nh("~mission")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		MissionBase::initialize(&wp_nh);

		wp_state = WP::IDLE;
		wp_type = plugin::WP_TYPE::MISSION;

		wp_nh.param("pull_after_gcs", do_pull_after_gcs, true);
		wp_nh.param("use_mission_item_int", use_mission_item_int, false);

		wp_list_pub = wp_nh.advertise<mavros_msgs::WaypointList>("waypoints", 2, true);
		pull_srv = wp_nh.advertiseService("pull", &WaypointPlugin::pull_cb, this);
		push_srv = wp_nh.advertiseService("push", &WaypointPlugin::push_cb, this);
		clear_srv = wp_nh.advertiseService("clear", &WaypointPlugin::clear_cb, this);

		wp_reached_pub = wp_nh.advertise<mavros_msgs::WaypointReached>("reached", 10, true);
		set_cur_srv = wp_nh.advertiseService("set_current", &WaypointPlugin::set_cur_cb, this);

		enable_connection_cb();
		enable_capabilities_cb();
	}

	Subscriptions get_subscriptions() override {
		return {
			       make_handler(&WaypointPlugin::handle_mission_item),
			       make_handler(&WaypointPlugin::handle_mission_item_int),
			       make_handler(&WaypointPlugin::handle_mission_request),
			       make_handler(&WaypointPlugin::handle_mission_request_int),
			       make_handler(&WaypointPlugin::handle_mission_count),
			       make_handler(&WaypointPlugin::handle_mission_ack),
			       make_handler(&WaypointPlugin::handle_mission_current),
			       make_handler(&WaypointPlugin::handle_mission_item_reached),
		};
	}

private:
	ros::NodeHandle wp_nh;

	ros::Publisher wp_list_pub;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer clear_srv;

	ros::Publisher wp_reached_pub;
	ros::ServiceServer set_cur_srv;

	/* -*- rx handlers -*- */

	/**
	 * @brief handle MISSION_CURRENT mavlink msg
	 * This confirms a SET_CUR action
	 * @param msg		Received Mavlink msg
	 * @param mcur		MISSION_CURRENT from msg
	 */
	void handle_mission_current(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_CURRENT &mcur)
	{
		unique_lock lock(mutex);

		if (wp_state == WP::SET_CUR) {
			/* MISSION_SET_CURRENT ACK */
			ROS_DEBUG_NAMED(log_ns, "%s: set current #%d done", log_ns.c_str(), mcur.seq);
			go_idle();
			wp_cur_active = mcur.seq;
			set_current_waypoint(wp_cur_active);

			lock.unlock();
			list_sending.notify_all();
			publish_waypoints();
		}
		else if (wp_state == WP::IDLE && wp_cur_active != mcur.seq) {
			/* update active */
			ROS_DEBUG_NAMED(log_ns, "%s: update current #%d", log_ns.c_str(), mcur.seq);
			wp_cur_active = mcur.seq;
			set_current_waypoint(wp_cur_active);

			lock.unlock();
			publish_waypoints();
		}
	}

	/**
	 * @brief handle MISSION_ITEM_REACHED mavlink msg
	 * @param msg		Received Mavlink msg
	 * @param mitr		MISSION_ITEM_REACHED from msg
	 */
	void handle_mission_item_reached(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_ITEM_REACHED &mitr)
	{
		/* in QGC used as informational message */
		ROS_INFO_NAMED(log_ns, "%s: reached #%d", log_ns.c_str(), mitr.seq);

		auto wpr = boost::make_shared<mavros_msgs::WaypointReached>();

		wpr->header.stamp = ros::Time::now();
		wpr->wp_seq = mitr.seq;

		wp_reached_pub.publish(wpr);
	}

	/* -*- mid-level helpers -*- */

	// Acts when capabilities of the fcu are changed
	void capabilities_cb(UAS::MAV_CAP capabilities) override {
		lock_guard lock(mutex);
		if (m_uas->has_capability(UAS::MAV_CAP::MISSION_INT)) {
			use_mission_item_int = true;
			mission_item_int_support_confirmed = true;
			ROS_INFO_NAMED(log_ns, "%s: Using MISSION_ITEM_INT", log_ns.c_str());
		} else {
			use_mission_item_int = false;
			mission_item_int_support_confirmed = false;
			ROS_WARN_NAMED(log_ns, "%s: Falling back to MISSION_ITEM", log_ns.c_str());
		}
	}

	// Act on first heartbeat from FCU
	void connection_cb(bool connected) override
	{
		lock_guard lock(mutex);
		if (connected) {
			schedule_pull(BOOTUP_TIME_DT);

			if (wp_nh.hasParam("enable_partial_push")) {
				wp_nh.getParam("enable_partial_push", enable_partial_push);
			}
			else {
				enable_partial_push = m_uas->is_ardupilotmega();
			}
		}
		else {
			schedule_timer.stop();
		}
	}

	//! @brief publish the updated waypoint list after operation
	void publish_waypoints() override
	{
		auto wpl = boost::make_shared<mavros_msgs::WaypointList>();
		unique_lock lock(mutex);

		wpl->current_seq = wp_cur_active;
		wpl->waypoints.clear();
		wpl->waypoints.reserve(waypoints.size());
		for (auto &it : waypoints) {
			wpl->waypoints.push_back(it);
		}

		lock.unlock();
		wp_list_pub.publish(wpl);
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

		if (req.start_index) {
			// Partial Waypoint update

			if (!enable_partial_push) {
				ROS_WARN_NAMED(log_ns, "%s: Partial Push not enabled. (Only supported on APM)", log_ns.c_str());
				res.success = false;
				res.wp_transfered = 0;
				return true;
			}

			if (waypoints.size() < req.start_index + req.waypoints.size()) {
				ROS_WARN_NAMED(log_ns, "%s: Partial push out of range rejected.", log_ns.c_str());
				res.success = false;
				res.wp_transfered = 0;
				return true;
			}

			wp_state = WP::TXPARTIAL;
			send_waypoints = waypoints;

			uint16_t seq = req.start_index;
			for (auto &it : req.waypoints) {
				send_waypoints[seq] = it;
				seq++;
			}

			wp_count = req.waypoints.size();
			wp_start_id = req.start_index;
			wp_end_id = req.start_index + wp_count;
			wp_cur_id = req.start_index;
			restart_timeout_timer();

			lock.unlock();
			mission_write_partial_list(wp_start_id, wp_end_id);
			res.success = wait_push_all();
			lock.lock();

			res.wp_transfered = wp_cur_id - wp_start_id + 1;
		}
		else {
			// Full waypoint update
			wp_state = WP::TXLIST;

			send_waypoints.clear();
			send_waypoints.reserve(req.waypoints.size());
			send_waypoints = req.waypoints;

			wp_count = send_waypoints.size();
			wp_end_id = wp_count;
			wp_cur_id = 0;
			restart_timeout_timer();

			lock.unlock();
			mission_count(wp_count);
			res.success = wait_push_all();
			lock.lock();

			res.wp_transfered = wp_cur_id + 1;
		}

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