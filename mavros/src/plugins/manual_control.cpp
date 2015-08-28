/**
 * @brief ManualControls plugin
 * @file manual_controls.cpp
 * @author Matias Nitsche <mnitsche@dc.uba.ar>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * * Copyright 2015 Matias Nitsche.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/ManualControl.h>

namespace mavplugin {
/**
 * @brief Manual Control plugin
 */
class ManualControlPlugin : public MavRosPlugin {
public:
	ManualControlPlugin() :
		manual_control_nh("~manual_control"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		control_pub = manual_control_nh.advertise<mavros_msgs::ManualControl>("control", 10);
		//uas->sig_connection_changed.connect(boost::bind(&RCIOPlugin::connection_cb, this, _1));
	};

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_MANUAL_CONTROL, &ManualControlPlugin::handle_manual_control),
		};
	}

private:
	ros::NodeHandle manual_control_nh;
	UAS *uas;

	ros::Publisher control_pub;

	/* -*- rx handlers -*- */

	void handle_manual_control(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_manual_control_t manual_control;
		mavlink_msg_manual_control_decode(msg, &manual_control);

		mavros_msgs::ManualControl manual_control_ros;
		manual_control_ros.header.stamp = ros::Time::now();	// correct?
		manual_control_ros.x = (manual_control.x / 1000.0);
		manual_control_ros.y = (manual_control.y / 1000.0);
		manual_control_ros.z = (manual_control.z / 1000.0);
		manual_control_ros.r = (manual_control.r / 1000.0);
		manual_control_ros.buttons = manual_control.buttons;

		control_pub.publish(manual_control_ros);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ManualControlPlugin, mavplugin::MavRosPlugin)

