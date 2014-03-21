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

	static mavros::WaypointPtr to_msgp(WaypointItem &wp) {
		mavros::WaypointPtr ret(new mavros::Waypoint);

		ret->frame = static_cast<uint8_t>(wp.frame);
		ret->command = static_cast<uint16_t>(wp.command);
		ret->is_current = wp.current;
		ret->autocontinue = wp.autocontinue;
		ret->param1 = wp.param1;
		ret->param2 = wp.param1;
		ret->param3 = wp.param1;
		ret->param4 = wp.param1;
		ret->x_lat = wp.x_lat;
		ret->y_long = wp.y_long;
		ret->z_alt = wp.z_alt;

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
			mavlink_mission_item_t mit;
			mavlink_msg_mission_item_decode(msg, &mit);
			handle_mission_item(mit);
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

	std::vector<WaypointItem> waypoints;
	bool is_list_receiving;

	/* -*- rx handlers -*- */

	void handle_mission_item(mavlink_mission_item_t &mit) {
		WaypointItem wpi = WaypointItem::from_mission_item(mit);
	}

	void handle_mission_current(mavlink_mission_current_t &mcur) {
	}

	void handle_mission_count(mavlink_mission_count_t &mcnt) {
	}

	void handle_mission_item_reached(mavlink_mission_item_reached_t &mitr) {
		/* TODO check APM support */
		ROS_INFO_NAMED("wp", "WP: Reached #%d", mitr.seq);
	}

	void handle_mission_ack(mavlink_mission_ack_t &mack) {
		/* TODO ACK */
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

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::WaypointPlugin, mavplugin::MavRosPlugin)

