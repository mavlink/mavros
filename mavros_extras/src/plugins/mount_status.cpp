/**
 * @brief Mouny Status plugin
 * @file mount_status.cpp
 * @author André Ferreira <andre.ferreira@beyond-vision.pt>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 André Ferreira.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/MountStatus.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Mount Status plugin
 *
 * Publish Mount status from FCU.
 * @see command_cb()
 */
class MountStatusPlugin : public plugin::PluginBase {
public:
	MountStatusPlugin() : PluginBase(),
		nh("~"),
		mount_nh("~mount_status")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		mount_status_pub = mount_nh.advertise<mavros_msgs::MountStatus>("orientation", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&MountStatusPlugin::handle_mount_status)
		};
	}

private:
	ros::NodeHandle nh;
	ros::NodeHandle mount_nh;
	ros::Publisher mount_status_pub;

	/**
	 * @brief Publish the mount status
	 *
	 * @param msg   the mavlink message
	 * @param ms	received MountStatus msg
	 */
	void handle_mount_status(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::MOUNT_STATUS &ms)
	{
		mavros_msgs::MountStatus publish_msg;

		publish_msg.header.stamp = ros::Time::now();

		publish_msg.target_system = ms.target_system;
		publish_msg.target_component = ms.target_component;

		publish_msg.pointing_a = ms.pointing_a;
		publish_msg.pointing_b = ms.pointing_b;
		publish_msg.pointing_c = ms.pointing_c;

		mount_status_pub.publish(publish_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MountStatusPlugin, mavros::plugin::PluginBase)
