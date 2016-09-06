/**
 * @brief HilActuatorControls plugin
 * @file hil_actuator_controls.cpp
 * @author Pavel Vechersky <pvechersky@student.ethz.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Pavel Vechersky & Beat Küng.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/HilActuatorControls.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Hil Actuator Control plugin
 */
class HilActuatorControlsPlugin : public plugin::PluginBase {
public:
	HilActuatorControlsPlugin() : PluginBase(),
		hil_actuator_controls_nh("~")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		hil_actuator_controls_pub = hil_actuator_controls_nh.advertise<mavros_msgs::HilActuatorControls>("hil_actuator_controls", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&HilActuatorControlsPlugin::handle_hil_actuator_controls),
		};
	}

private:
	ros::NodeHandle hil_actuator_controls_nh;

	ros::Publisher hil_actuator_controls_pub;

	/* -*- rx handlers -*- */

	void handle_hil_actuator_controls(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HIL_ACTUATOR_CONTROLS &hil_actuator_controls)
	{
		auto hil_actuator_controls_msg = boost::make_shared<mavros_msgs::HilActuatorControls>();

		hil_actuator_controls_msg->header.stamp = m_uas->synchronise_stamp(hil_actuator_controls.time_usec);
		for (int i = 0; i < 16; ++i) {
			hil_actuator_controls_msg->controls[i] = hil_actuator_controls.controls[i];
		}
		hil_actuator_controls_msg->mode = hil_actuator_controls.mode;
		hil_actuator_controls_msg->flags = hil_actuator_controls.flags;

		hil_actuator_controls_pub.publish(hil_actuator_controls_msg);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HilActuatorControlsPlugin, mavros::plugin::PluginBase)

