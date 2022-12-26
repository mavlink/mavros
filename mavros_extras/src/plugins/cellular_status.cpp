/**
 * @brief Cellular status plugin
 * @file cellular_status.cpp
 * @author Rui Mendes <rui.mendes@beyond-vision.pt>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/CellularStatus.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Cellular status plugin.
 *
 * Users must publish to the topic the CellularStatus message and it
 * will be relayed to the mavlink components.
 */
class CellularStatusPlugin : public plugin::PluginBase {
public:
	CellularStatusPlugin() : PluginBase(),
		cc_nh("~cellular_status")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		subCellularStatus = cc_nh.subscribe("status", 1, &CellularStatusPlugin::cellularStatusCb, this);
	}

	/**
	 * This function returns message subscriptions.
	 *
	 * Each subscription made by PluginBase::make_handler() template.
	 * Two variations:
	 *  - With automatic decoding and framing error filtering (see handle_heartbeat)
	 *  - Raw message with framig status (see handle_systemtext)
	 */
	Subscriptions get_subscriptions() {
		return {};
	}

private:
	ros::NodeHandle cc_nh;
	ros::Subscriber subCellularStatus;

	/**
	 * @brief Send Cellular Status messages to mavlink system
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#CELLULAR_STATUS
	 * @param msg	received CellularStatus msg
	 */
	void cellularStatusCb(const mavros_msgs::CellularStatus &msg)
	{
		mavlink::common::msg::CELLULAR_STATUS cs{};

		cs.status = msg.status;
		cs.failure_reason = msg.failure_reason;
		cs.type = msg.type;
		cs.quality = msg.quality;
		cs.mcc = msg.mcc;
		cs.mnc = msg.mnc;
		cs.lac = msg.lac;

		UAS_FCU(m_uas)->send_message_ignore_drop(cs);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CellularStatusPlugin, mavros::plugin::PluginBase)
