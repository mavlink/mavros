/**
 * @brief ActuatorControl plugin
 * @file actuator_control.cpp
 * @author Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ActuatorControl.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief ActuatorControl plugin
 *
 * Sends actuator controls to FCU controller.
 */
class ActuatorControlPlugin : public plugin::PluginBase {
public:
	ActuatorControlPlugin() : PluginBase(),
		nh("~")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		actuator_control_sub = nh.subscribe("actuator_control", 10, &ActuatorControlPlugin::actuator_control_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber actuator_control_sub;

	/* -*- callbacks -*- */

	void actuator_control_cb(const mavros_msgs::ActuatorControl::ConstPtr &req) {
		//! about groups, mixing and channels: @p https://pixhawk.org/dev/mixing
		//! message definiton here: @p http://mavlink.org/messages/common#SET_ACTUATOR_CONTROL_TARGET
		mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET act{};
		act.time_usec = ros::Time::now().toNSec() / 1000;
		act.group_mlx = req->group_mix;
		act.target_system = m_uas->get_tgt_system();
		act.target_component = m_uas->get_tgt_component();
		std::copy(req->controls.begin(), req->controls.end(), act.controls.begin());	// std::array = boost::array

		UAS_FCU(m_uas)->send_message_ignore_drop(act);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ActuatorControlPlugin, mavros::plugin::PluginBase)
