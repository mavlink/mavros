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

#pragma once

#include <array>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <mavros/mavconn_interface.h>
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
	explicit MavRos(const ros::NodeHandle &nh_);
	~MavRos() {};

	void spin();

private:
	ros::NodeHandle node_handle;
	ros::NodeHandle mavlink_node_handle;
	// fcu_link stored in mav_uas
	mavconn::MAVConnInterface::Ptr gcs_link;

	ros::Publisher mavlink_pub;
	ros::Subscriber mavlink_sub;

	diagnostic_updater::Updater diag_updater;
	MavlinkDiag fcu_link_diag;
	MavlinkDiag gcs_link_diag;

	pluginlib::ClassLoader<mavplugin::MavRosPlugin> plugin_loader;
	std::vector<mavplugin::MavRosPlugin::Ptr> loaded_plugins;
	std::vector<std::string> plugin_blacklist;
	std::array<mavconn::MAVConnInterface::MessageSig, 256>
		message_route_table; // link interface -> router -> plugin callback
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

}; // namespace mavros

