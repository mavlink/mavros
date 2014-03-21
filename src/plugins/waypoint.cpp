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

#include <mavros/WaypointList.h>
#include <mavros/WaypointSet.h>
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

	};

	std::string get_name() {
		return "Waypoint";
	};

	std::vector<uint8_t> get_supported_messages() {
		return {
			MAVLINK_MSG_ID_MISSION_ITEM,
			MAVLINK_MSG_ID_MISSION_CURRENT,
			MAVLINK_MSG_ID_MISSION_COUNT,
			MAVLINK_MSG_ID_MISSION_ITEM_REACHED,
			MAVLINK_MSG_ID_MISSION_ACK
		};
	};

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_MISSION_ITEM:
			ROS_INFO("WP: ITEM");
			break;
		case MAVLINK_MSG_ID_MISSION_CURRENT:
			ROS_INFO_THROTTLE(10, "WP: CURRENT");
			break;
		case MAVLINK_MSG_ID_MISSION_COUNT:
			ROS_INFO("WP: COUNT");
			break;
		case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
			ROS_INFO("WP: REACHED");
			break;
		case MAVLINK_MSG_ID_MISSION_ACK:
			ROS_INFO("WP: ACK");
			break;
		}
	};

private:
	UAS *uas;
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::WaypointPlugin, mavplugin::MavRosPlugin)

