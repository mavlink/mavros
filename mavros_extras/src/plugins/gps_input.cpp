/**
 * @brief GPS_INPUT plugin
 * @file gps_input.cpp
 * @author Amilcar Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Amilcar Lucas.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include "mavros_msgs/GPSINPUT.h"

namespace mavros {
namespace extra_plugins {
using mavlink::common::GPS_FIX_TYPE;
using mavlink::common::GPS_INPUT_IGNORE_FLAGS;

/**
 * @brief GPS_INPUT GPS plugin.
 *
 * Sends <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">GPS_INPUT MAVLink messages</a>
 */
class GpsInputPlugin : public plugin::PluginBase {
public:
	GpsInputPlugin() : PluginBase(),
		gps_input_nh("~gps_input"),
		gps_rate(5.0)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		double _gps_rate;

		last_pos_time = ros::Time(0.0);
		gps_input_nh.param("gps_rate", _gps_rate, 5.0);		// GPS data rate of 5hz
		gps_rate = _gps_rate;

		gps_input_sub = gps_input_nh.subscribe("gps_input", 1, &GpsInputPlugin::send_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle gps_input_nh;
	ros::Subscriber gps_input_sub;

	ros::Rate gps_rate;
	ros::Time last_pos_time;

	/* -*- callbacks -*- */

	/**
	 * @brief Send GPS coordinates through GPS_INPUT Mavlink message
	 */
	void send_cb(const mavros_msgs::GPSINPUT::ConstPtr ros_msg) {
		// Throttle incoming messages to 5hz
		if ((ros::Time::now() - last_pos_time) < ros::Duration(gps_rate)) {
			return;
		}
		last_pos_time = ros::Time::now();

		/**
		 * @note: <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">GPS_INPUT MAVLink message</a>
		 * is currently only supported by Ardupilot firmware
		 */
		mavlink::common::msg::GPS_INPUT gps_input {};

		// Fill in and send message
		gps_input.time_usec          = ros_msg->header.stamp.toNSec() / 1000;
		gps_input.gps_id             = ros_msg->gps_id;
		gps_input.ignore_flags       = ros_msg->ignore_flags;
		gps_input.time_week_ms       = ros_msg->time_week_ms;
		gps_input.time_week          = ros_msg->time_week;
		gps_input.speed_accuracy     = ros_msg->speed_accuracy;
		gps_input.horiz_accuracy     = ros_msg->horiz_accuracy;
		gps_input.vert_accuracy      = ros_msg->vert_accuracy;
		gps_input.lat                = ros_msg->lat;
		gps_input.lon                = ros_msg->lon;
		gps_input.alt                = ros_msg->alt;
		gps_input.vn                 = ros_msg->vn;
		gps_input.ve                 = ros_msg->ve;
		gps_input.vd                 = ros_msg->vd;
		gps_input.hdop               = ros_msg->hdop;
		gps_input.vdop               = ros_msg->vdop;
		gps_input.fix_type           = ros_msg->fix_type;
		gps_input.satellites_visible = ros_msg->satellites_visible;

		UAS_FCU(m_uas)->send_message_ignore_drop(gps_input);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GpsInputPlugin, mavros::plugin::PluginBase)
