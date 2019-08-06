/**
 * @brief Companion System Status plugin
 * @file companion_system_status.cpp
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

#include <mavros_msgs/CompanionSystemStatus.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief Companion system status plugin
 *
 * Publishes the status of the companion computer
 * @see status_cb()
 */
class CompanionSystemStatusPlugin : public plugin::PluginBase {
public:
	CompanionSystemStatusPlugin() : PluginBase(),
	status_nh("~companion_system")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		status_sub = status_nh.subscribe("status", 10, &CompanionSystemStatusPlugin::status_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle status_nh;
	ros::Subscriber status_sub;

	/**
	 * @brief Send companion system status to FCU and groundstation
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#COMPANION_SYSTEM_STATUS
	 * @param req	received CompanionSystemStatus msg
	 */
	void status_cb(const mavros_msgs::CompanionSystemStatus::ConstPtr &req)
	{
		mavlink::common::msg::COMPANION_SYSTEM_STATUS status {};

		status.cpu_usage = req->cpu_usage;
		status.temperature = req->temperature;
		status.ram_usage = req->ram_usage;
		status.ram_total = req->ram_total;
		status.storage_free = req->storage_free;
		status.storage_total = req->storage_total;
		status.network_send_rate = req->network_send_rate;
		status.network_receive_rate = req->network_receive_rate;
		status.time_usec = req->time_usec;
		status.uptime = req->uptime;

		UAS_FCU(m_uas)->send_message_ignore_drop(status, req->component);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CompanionSystemStatusPlugin, mavros::plugin::PluginBase)
