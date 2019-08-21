/**
 * @brief Onboard Computer Status plugin
 * @file onboard_computer_status.cpp
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

#include <mavros_msgs/OnboardComputerStatus.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Onboard Computer Status plugin
 *
 * Publishes the status of the onboard computer
 * @see status_cb()
 */
class OnboardComputerStatusPlugin : public plugin::PluginBase {
public:
	OnboardComputerStatusPlugin() : PluginBase(),
		status_nh("~onboard_computer")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		status_sub = status_nh.subscribe("status", 10, &OnboardComputerStatusPlugin::status_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle status_nh;
	ros::Subscriber status_sub;

	/**
	 * @brief Send onboard computer status to FCU and groundstation
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#ONBOARD_COMPUTER_STATUS
	 * @param req	received OnboardComputerStatus msg
	 */
	void status_cb(const mavros_msgs::OnboardComputerStatus::ConstPtr &req)
	{
		mavlink::common::msg::ONBOARD_COMPUTER_STATUS status {};
		status.time_usec = req->header.stamp.toNSec() / 1000;					//!< [microsecs]
		// [[[cog:
		// for f in ('uptime',
		//     'type',
		//     'temperature_board',
		//     'ram_usage',
		//     'ram_total'):
		//     cog.outl("status.%s = req->%s;" % (f, f))
		//
		// for f in ('cpu_cores',
		//     'cpu_combined',
		//     'gpu_cores',
		//     'gpu_combined',
		//     'temperature_core',
		//     'fan_speed',
		//     'storage_type',
		//     'storage_usage',
		//     'storage_total',
		//     'link_type',
		//     'link_tx_rate',
		//     'link_rx_rate',
		//     'link_tx_max',
		//     'link_rx_max'):
		//     cog.outl("std::copy(req->%s.cbegin(), req->%s.cend(), status.%s.begin());" % (f, f, f))
		// ]]]
		status.uptime = req->uptime;
		status.type = req->type;
		status.temperature_board = req->temperature_board;
		status.ram_usage = req->ram_usage;
		status.ram_total = req->ram_total;
		std::copy(req->cpu_cores.cbegin(), req->cpu_cores.cend(), status.cpu_cores.begin());
		std::copy(req->cpu_combined.cbegin(), req->cpu_combined.cend(), status.cpu_combined.begin());
		std::copy(req->gpu_cores.cbegin(), req->gpu_cores.cend(), status.gpu_cores.begin());
		std::copy(req->gpu_combined.cbegin(), req->gpu_combined.cend(), status.gpu_combined.begin());
		std::copy(req->temperature_core.cbegin(), req->temperature_core.cend(), status.temperature_core.begin());
		std::copy(req->fan_speed.cbegin(), req->fan_speed.cend(), status.fan_speed.begin());
		std::copy(req->storage_type.cbegin(), req->storage_type.cend(), status.storage_type.begin());
		std::copy(req->storage_usage.cbegin(), req->storage_usage.cend(), status.storage_usage.begin());
		std::copy(req->storage_total.cbegin(), req->storage_total.cend(), status.storage_total.begin());
		std::copy(req->link_type.cbegin(), req->link_type.cend(), status.link_type.begin());
		std::copy(req->link_tx_rate.cbegin(), req->link_tx_rate.cend(), status.link_tx_rate.begin());
		std::copy(req->link_rx_rate.cbegin(), req->link_rx_rate.cend(), status.link_rx_rate.begin());
		std::copy(req->link_tx_max.cbegin(), req->link_tx_max.cend(), status.link_tx_max.begin());
		std::copy(req->link_rx_max.cbegin(), req->link_rx_max.cend(), status.link_rx_max.begin());
		// [[[end]]] (checksum: 98538293a5932dfb5952d4badd311b39)

		std::cout << "timestamp: " << status.time_usec << "\n";

		UAS_FCU(m_uas)->send_message_ignore_drop(status, req->component);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OnboardComputerStatusPlugin, mavros::plugin::PluginBase)
