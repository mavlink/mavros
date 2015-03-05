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

#include <mavros/Mavlink.h>

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

	ros::Publisher mavlink_pub;
	ros::Subscriber mavlink_sub;

	MavlinkDiag fcu_link_diag;
	MavlinkDiag gcs_link_diag;

	pluginlib::ClassLoader<mavplugin::MavRosPlugin> plugin_loader;
	std::vector<mavplugin::MavRosPlugin::Ptr> loaded_plugins;
	std::vector<std::string> plugin_blacklist;
	// link interface -> router -> plugin callback
	std::array<mavconn::MAVConnInterface::MessageSig, 256> message_route_table;
	UAS mav_uas;

	void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid);
	void mavlink_sub_cb(const Mavlink::ConstPtr &rmsg);
	void plugin_route_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid);
	bool check_in_blacklist(std::string &pl_name);
	void add_plugin(std::string &pl_name);
	void terminate_cb();
	void startup_px4_usb_quirk(void);
	void log_connect_change(bool connected);
};
};	// namespace mavros

