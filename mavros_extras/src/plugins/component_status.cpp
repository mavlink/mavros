/**
 * @brief Component Status plugin
 * @file component_status.cpp
 * @author Tanja Baumann <tanja@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Tanja Baumann.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ComponentStatus.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink enumerations
using mavlink::common::MAV_TYPE;
using mavlink::common::MAV_STATE;
using mavlink::common::MAV_COMPONENT;
using utils::enum_value;

/**
 * @brief Component status plugin
 *
 * Publishes the status of components
 * @see status_cb()
 */
class ComponentStatusPlugin : public plugin::PluginBase {
public:
	ComponentStatusPlugin() : PluginBase(),
	status_nh("~component")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		status_sub = status_nh.subscribe("status", 10, &ComponentStatusPlugin::status_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle status_nh;
	ros::Subscriber status_sub;

	/**
	 * @brief Send component status to FCU and groundstation
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#COMPONENT_STATUS
	 * @param req	received ComponentStatus msg
	 */
	void status_cb(const mavros_msgs::ComponentStatus::ConstPtr &req)
	{
		mavlink::common::msg::COMPONENT_STATUS status {};

		status.cpu_usage = req->cpu_usage;
		status.temperature = req->temperature;
		status.ram_free = req->ram_free;
		status.ram_total = req->ram_total;
		status.storage_free = req->storage_free;
		status.storage_total = req->storage_total;
		status.network_send_rate = req->network_send_rate;
		status.network_receive_rate = req->network_receive_rate;
		status.time_boot_ms = req->time_boot_ms;

		UAS_FCU(m_uas)->send_message_ignore_drop(status);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ComponentStatusPlugin, mavros::plugin::PluginBase)
