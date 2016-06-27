/**
 * @brief HilControls plugin
 * @file hil_controls.cpp
 * @author Pavel Vechersky <pvechersky@student.ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Pavel Vechersky.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/HilControls.h>

namespace mavplugin {
/**
 * @brief Hil Control plugin
 */
class HilControlsPlugin : public MavRosPlugin {
public:
	HilControlsPlugin() :
		hil_controls_nh("~hil_controls"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		hil_controls_pub = hil_controls_nh.advertise<mavros_msgs::HilControls>("hil_controls", 10);
	};

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_HIL_CONTROLS, &HilControlsPlugin::handle_hil_controls),
		};
	}

private:
	ros::NodeHandle hil_controls_nh;
	UAS *uas;

	ros::Publisher hil_controls_pub;

	/* -*- rx handlers -*- */

	void handle_hil_controls(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_hil_controls_t hil_controls;
		mavlink_msg_hil_controls_decode(msg, &hil_controls);

		auto hil_controls_msg = boost::make_shared<mavros_msgs::HilControls>();

		hil_controls_msg->header.stamp = uas->synchronise_stamp(hil_controls.time_usec);
		hil_controls_msg->roll_ailerons = hil_controls.roll_ailerons;
		hil_controls_msg->pitch_elevator = hil_controls.pitch_elevator;
		hil_controls_msg->yaw_rudder = hil_controls.yaw_rudder;
		hil_controls_msg->throttle = hil_controls.throttle;
		hil_controls_msg->aux1 = hil_controls.aux1;
		hil_controls_msg->aux2 = hil_controls.aux2;
		hil_controls_msg->aux3 = hil_controls.aux3;
		hil_controls_msg->aux4 = hil_controls.aux4;
		hil_controls_msg->mode = hil_controls.mode;
		hil_controls_msg->nav_mode = hil_controls.nav_mode;

		hil_controls_pub.publish(hil_controls_msg);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::HilControlsPlugin, mavplugin::MavRosPlugin)

