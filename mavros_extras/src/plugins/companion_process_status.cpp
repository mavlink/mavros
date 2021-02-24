/**
 * @brief Companion Status plugin
 * @file companion_status.cpp
 * @author Tanja Baumann <tanja@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Tanja Baumann.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CompanionProcessStatus.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink enumerations
using mavlink::minimal::MAV_TYPE;
using mavlink::minimal::MAV_STATE;
using mavlink::minimal::MAV_COMPONENT;
using mavlink::minimal::MAV_AUTOPILOT;
using mavlink::minimal::MAV_MODE_FLAG;
using utils::enum_value;

/**
 * @brief Obstacle companion process status plugin
 *
 * Publishes the status of components running on the companion computer
 * @see status_cb()
 */
class CompanionProcessStatusPlugin : public plugin::PluginBase {
public:
	CompanionProcessStatusPlugin() : PluginBase(),
	status_nh("~companion_process")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		status_sub = status_nh.subscribe("status", 10, &CompanionProcessStatusPlugin::status_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle status_nh;
	ros::Subscriber status_sub;

	/**
	 * @brief Send companion process status to FCU over a heartbeat message
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#HEARTBEAT
	 * @param req	received CompanionProcessStatus msg
	 */
	void status_cb(const mavros_msgs::CompanionProcessStatus::ConstPtr &req)
	{
		mavlink::minimal::msg::HEARTBEAT heartbeat {};

		heartbeat.type = enum_value(MAV_TYPE::ONBOARD_CONTROLLER);
		heartbeat.autopilot = enum_value(MAV_AUTOPILOT::PX4);
		heartbeat.base_mode	= enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
		heartbeat.system_status = req->state;	//enum="MAV_STATE" System status flag

		ROS_DEBUG_STREAM_NAMED("companion_process_status", "companion process component id: " <<
						utils::to_string_enum<MAV_COMPONENT>(req->component) << " companion process status: " <<
						utils::to_string_enum<MAV_STATE>(heartbeat.system_status) << std::endl << heartbeat.to_yaml());

		UAS_FCU(m_uas)->send_message_ignore_drop(heartbeat, req->component);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CompanionProcessStatusPlugin, mavros::plugin::PluginBase)
