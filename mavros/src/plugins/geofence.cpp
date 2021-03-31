/**
 * @brief Geofence plugin
 * @file geofence.cpp
 * @author Charlie Burge <charlieburge@yahoo.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Charlie Burge.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mission_protocol_base.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Geofence manipulation plugin
 */
class GeofencePlugin : public plugin::MissionBase {
public:
	GeofencePlugin() :
		MissionBase("GF"),
		gf_nh("~geofence")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		MissionBase::initialize(&gf_nh);

		wp_state = WP::IDLE;
		wp_type = plugin::WP_TYPE::FENCE;

		gf_nh.param("pull_after_gcs", do_pull_after_gcs, true);
		gf_nh.param("use_mission_item_int", use_mission_item_int, false);

		gf_list_pub = gf_nh.advertise<mavros_msgs::WaypointList>("waypoints", 2, true);
		pull_srv = gf_nh.advertiseService("pull", &GeofencePlugin::pull_cb, this);
		push_srv = gf_nh.advertiseService("push", &GeofencePlugin::push_cb, this);
		clear_srv = gf_nh.advertiseService("clear", &GeofencePlugin::clear_cb, this);

		enable_connection_cb();
		enable_capabilities_cb();
	}

	Subscriptions get_subscriptions() override {
		return {
			       make_handler(&GeofencePlugin::handle_mission_item),
			       make_handler(&GeofencePlugin::handle_mission_item_int),
			       make_handler(&GeofencePlugin::handle_mission_request),
			       make_handler(&GeofencePlugin::handle_mission_request_int),
			       make_handler(&GeofencePlugin::handle_mission_count),
			       make_handler(&GeofencePlugin::handle_mission_ack),
		};
	}

private:
	ros::NodeHandle gf_nh;

	ros::Publisher gf_list_pub;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer clear_srv;

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

			if (gf_nh.hasParam("enable_partial_push")) {
				gf_nh.getParam("enable_partial_push", enable_partial_push);
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
		gf_list_pub.publish(wpl);
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
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::GeofencePlugin, mavros::plugin::PluginBase)