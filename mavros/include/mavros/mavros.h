/**
 * @brief MavRos node implementation class
 * @file mavros.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 *  @brief MAVROS node implementation
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <array>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <mavconn/interface.h>
#include <mavros/mavros_plugin.h>
#include <mavros/mavlink_diag.h>
#include <mavros/utils.h>

namespace mavros {
/**
 * @brief MAVROS node class
 *
 * This class implement mavros_node
 */
class MavRos
{
public:
	MavRos();
	~MavRos() {};

	void spin();

private:
	ros::NodeHandle mavlink_nh;
	// fcu_link stored in mav_uas
	mavconn::MAVConnInterface::Ptr gcs_link;
	bool gcs_quiet_mode;
	ros::Time last_message_received_from_gcs;
	ros::Duration conn_timeout;

	ros::Publisher mavlink_pub;
	ros::Subscriber mavlink_sub;

	diagnostic_updater::Updater gcs_diag_updater;
	MavlinkDiag fcu_link_diag;
	MavlinkDiag gcs_link_diag;

	pluginlib::ClassLoader<plugin::PluginBase> plugin_loader;
	std::vector<plugin::PluginBase::Ptr> loaded_plugins;

	//! FCU link -> router -> plugin handler
	std::unordered_map<mavlink::msgid_t, plugin::PluginBase::Subscriptions> plugin_subscriptions;

	//! UAS object passed to all plugins
	UAS mav_uas;

	//! fcu link -> ros
	void mavlink_pub_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing);
	//! ros -> fcu link
	void mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg);

	//! message router
	void plugin_route_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing);

	//! load plugin
	void add_plugin(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist);

	//! start mavlink app on USB
	void startup_px4_usb_quirk();
	void log_connect_change(bool connected);
};
}	// namespace mavros

