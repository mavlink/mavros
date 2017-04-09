/**
 * @brief ActuatorControlTarget plugin
 * @file actuator_control_target.cpp
 * @author Mohamed Abdelkader Zahana <mohamedashraf123@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Mohamed Abdelkader Zahana.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ActuatorControlTarget.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Actuator Control Target plugin
 */
class ActuatorControlTargetPlugin : public plugin::PluginBase {
public:
	ActuatorControlTargetPlugin() : PluginBase(),
		actuator_control_target_nh("~")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		actuator_control_target_pub = actuator_control_target_nh.advertise<mavros_msgs::ActuatorControlTarget>("actuator_control_target", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&ActuatorControlTargetPlugin::handle_actuator_control_target),
		};
	}

private:
	ros::NodeHandle actuator_control_target_nh;

	ros::Publisher actuator_control_target_pub;

	/* -*- rx handlers -*- */

	void handle_actuator_control_target(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ACTUATOR_CONTROL_TARGET &actuator_control_target)
	{
		auto actuator_control_target_msg = boost::make_shared<mavros_msgs::ActuatorControlTarget>();

		actuator_control_target_msg->header.stamp = m_uas->synchronise_stamp(actuator_control_target.time_usec);

		actuator_control_target_msg->group_mlx = actuator_control_target.group_mlx;
		for(int i=0; i<8; i++){
			actuator_control_target_msg->controls[i]=actuator_control_target.controls[i];
		}

		actuator_control_target_pub.publish(actuator_control_target_msg);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ActuatorControlTargetPlugin, mavros::plugin::PluginBase)

