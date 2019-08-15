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

		status.time_usec = req->time_usec;
		status.uptime = req->uptime;
		status.type = req->type;

		const auto &cpu_cores = req->cpu_cores;
		std::copy(cpu_cores.cbegin(), cpu_cores.cend(), status.cpu_cores.begin());

		const auto &cpu_combined = req->cpu_combined;
		std::copy(cpu_combined.cbegin(), cpu_combined.cend(), status.cpu_combined.begin());

		const auto &gpu_cores = req->gpu_cores;
		std::copy(gpu_cores.cbegin(), gpu_cores.cend(), status.gpu_cores.begin());

		const auto &gpu_combined = req->gpu_combined;
		std::copy(gpu_combined.cbegin(), gpu_combined.cend(), status.gpu_combined.begin());

		status.temperature_board = req->temperature_board;

		const auto &temperature_core = req->temperature_core;
		std::copy(temperature_core.cbegin(), temperature_core.cend(), status.temperature_core.begin());

		const auto &fan_speed = req->fan_speed;
		std::copy(fan_speed.cbegin(), fan_speed.cend(), status.fan_speed.begin());

		status.ram_usage = req->ram_usage;
		status.ram_total = req->ram_total;

		const auto &storage_type = req->storage_type;
		std::copy(storage_type.cbegin(), storage_type.cend(), status.storage_type.begin());

		const auto &storage_usage = req->storage_usage;
		std::copy(storage_usage.cbegin(), storage_usage.cend(), status.storage_usage.begin());

		const auto &storage_total = req->storage_total;
		std::copy(storage_total.cbegin(), storage_total.cend(), status.storage_total.begin());

		const auto &link_type = req->link_type;
		std::copy(link_type.cbegin(), link_type.cend(), status.link_type.begin());

		const auto &link_tx_rate = req->link_tx_rate;
		std::copy(link_tx_rate.cbegin(), link_tx_rate.cend(), status.link_tx_rate.begin());

		const auto &link_rx_rate = req->link_rx_rate;
		std::copy(link_rx_rate.cbegin(), link_rx_rate.cend(), status.link_rx_rate.begin());

		const auto &link_tx_max = req->link_tx_max;
		std::copy(link_tx_max.cbegin(), link_tx_max.cend(), status.link_tx_max.begin());

		const auto &link_rx_max = req->link_rx_max;
		std::copy(link_rx_max.cbegin(), link_rx_max.cend(), status.link_rx_max.begin());


		UAS_FCU(m_uas)->send_message_ignore_drop(status, req->component);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OnboardComputerStatusPlugin, mavros::plugin::PluginBase)
