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

#include <mavros_msgs/HilControls.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Hil Control plugin
 */
class HilControlsPlugin : public plugin::PluginBase {
public:
	HilControlsPlugin() : PluginBase(),
		hil_controls_nh("~hil_controls")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		hil_controls_pub = hil_controls_nh.advertise<mavros_msgs::HilControls>("hil_controls", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&HilControlsPlugin::handle_hil_controls),
		};
	}

private:
	ros::NodeHandle hil_controls_nh;

	ros::Publisher hil_controls_pub;

	/* -*- rx handlers -*- */

	void handle_hil_controls(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIL_CONTROLS &hil_controls)
	{
		auto hil_controls_msg = boost::make_shared<mavros_msgs::HilControls>();

		hil_controls_msg->header.stamp = m_uas->synchronise_stamp(hil_controls.time_usec);
		// [[[cog:
		// for f in (
		//     'roll_ailerons', 'pitch_elevator', 'yaw_rudder', 'throttle',
		//     'aux1', 'aux2', 'aux3', 'aux4', 'mode', 'nav_mode'):
		//     cog.outl("hil_controls_msg->%s = hil_controls.%s;" % (f, f))
		// ]]]
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
		// [[[end]]] (checksum: a2c87ee8f36e7a32b08be5e0fe665b5a)

		hil_controls_pub.publish(hil_controls_msg);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilControlsPlugin, mavros::plugin::PluginBase)

