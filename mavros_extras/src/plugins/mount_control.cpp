/**
 * @brief Mouny Control plugin
 * @file mount_control.cpp
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/MountControl.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink enumerations
using mavlink::common::MAV_TYPE;
using mavlink::common::MAV_STATE;
using mavlink::common::MAV_COMPONENT;
using mavlink::common::MAV_MOUNT_MODE;
using utils::enum_value;

/**
 * @brief Mount Control plugin
 *
 * Publishes Mission commands to control the camera or antenna mount.
 * @see status_cb()
 */
class MountControlPlugin : public plugin::PluginBase {
public:
	MountControlPlugin() : PluginBase(),
	mount_nh("~mount_control")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		command_sub = mount_nh.subscribe("command", 10, &MountControlPlugin::command_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mount_nh;
	ros::Subscriber command_sub;

	/**
	 * @brief Send mount control commands to vehicle
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#DO_MOUNT_CONTROL
	 * @param req	received MountControl msg
	 */
	void command_cb(const mavros_msgs::MountControl::ConstPtr &req)
	{
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MountControlPlugin, mavros::plugin::PluginBase)
