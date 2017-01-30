/**
 * @brief HomePosition plugin 
 * @file home_position.cpp
 * @author Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 *
 * @Plugin to publish home position mavlink messages in ROS
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Thomas Stastny.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <list>

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/HomePosition.h>

namespace mavplugin {
/**
 * @brief home position plugin.
 *
 * Wrapper for home position MAVLink messages.
 */

class HomePositionPlugin : public MavRosPlugin {
public:
	HomePositionPlugin() :
		hp_nh("~"),
		nh(""),
		uas(nullptr)
    { }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		uas = &uas_;

        home_position_pub = hp_nh.advertise<mavros_msgs::HomePosition>("home_position",10,true);

	}

	/**
	 * This function returns message<->handler mapping
	 *
	 * Each entry defined by @a MESSAGE_HANDLER() macro
	 */
	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_HOME_POSITION, &HomePositionPlugin::handle_home_position),
    };
	}

private:
    ros::NodeHandle hp_nh;
    ros::NodeHandle nh;
    UAS *uas;

    ros::Publisher home_position_pub;

    void handle_home_position(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_home_position_t home_position;
        mavlink_msg_home_position_decode(msg, &home_position);

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        mavros_msgs::HomePosition homePositionMsg;

        homePositionMsg.header = header;

        homePositionMsg.latitude = home_position.latitude/1e7; // MAVLink msg contains degrees*1e7
        homePositionMsg.longitude = home_position.longitude/1e7; // MAVLink msg contains degrees*1e7
        homePositionMsg.altitude = home_position.altitude/1e3; // MAVLink msg contains meters*1e3 AMSL!! ROS standard would be WGS84!

        homePositionMsg.position.x = home_position.x;
        homePositionMsg.position.y = home_position.y;
        homePositionMsg.position.z = home_position.z;

        homePositionMsg.orientation.w = home_position.q[0];
        homePositionMsg.orientation.x = home_position.q[1];
        homePositionMsg.orientation.y = home_position.q[2];
        homePositionMsg.orientation.z = home_position.q[3];

        homePositionMsg.approach.x = home_position.approach_x;
        homePositionMsg.approach.y = home_position.approach_y;
        homePositionMsg.approach.z = home_position.approach_z;

        home_position_pub.publish(homePositionMsg);
    }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::HomePositionPlugin, mavplugin::MavRosPlugin)

