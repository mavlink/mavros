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
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/ActuatorControl.h>

namespace mavplugin {
/**
 * @brief ActuatorControl plugin
 *
 * Sends actuator controls to FCU controller.
 */
class ActuatorControlPlugin : public MavRosPlugin {
public:
	ActuatorControlPlugin() :
		nh("~"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		actuator_control_sub = nh.subscribe("actuator_control", 10, &ActuatorControlPlugin::actuator_control_cb, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle nh;
	UAS *uas;
	ros::Subscriber actuator_control_sub;

	/* -*- low-level send -*- */

	//! message definiton here: @p http://mavlink.org/messages/common#SET_ACTUATOR_CONTROL_TARGET
	void set_actuator_control_target(const uint64_t time_usec,
			const uint8_t group_mix,
			const float controls[8])
	{
		mavlink_message_t msg;

		mavlink_msg_set_actuator_control_target_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_usec,
				group_mix,
				UAS_PACK_TGT(uas),
				controls);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	void actuator_control_cb(const mavros_msgs::ActuatorControl::ConstPtr &req) {
		//! about groups, mixing and channels: @p https://pixhawk.org/dev/mixing
		set_actuator_control_target(ros::Time::now().toNSec() / 1000,
				req->group_mix,
				req->controls.data());
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ActuatorControlPlugin, mavplugin::MavRosPlugin)
