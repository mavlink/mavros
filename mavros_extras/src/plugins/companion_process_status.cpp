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
 * https://github.com/mavlink/mavros/tree/master/LICENSE.mdA
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CompanionProcessStatus.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink enumerations
using mavlink::common::COMPANION_PROCESS_STATE;
using mavlink::common::COMPANION_PROCESS_TYPE;

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

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		status_sub = status_nh.subscribe("status", 10, &CompanionProcessStatusPlugin::status_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle status_nh;
	ros::Subscriber status_sub;

	/**
	 * @brief Send companion process status to FCU
	 *
	 * Message specification: http://mavlink.org/messages/common#COMPANION_PROCESS_STATUS
	 * @param req	received CompanionProcessStatus msg
	 */



	void status_cb(const mavros_msgs::CompanionProcessStatus::ConstPtr &req)
	{
		mavlink::common::msg::HEARTBEAT heartbeat {};

		heartbeat.type = 12;					//enum="MAV_TYPE" Type of the MAV (quadrotor, helicopter, etc.) missuse filed for submarine
		heartbeat.autopilot = 12;				//enum="MAV_AUTOPILOT" Autopilot type PX4
		heartbeat.base_mode	= 2;				// enum="MAV_MODE_FLAG" set to MAV_MODE_FLAG_TEST_ENABLED
		heartbeat.custom_mode = 0;				//A bitfield for use for autopilot-specific flags
		heartbeat.system_status = req->state;	//enum="MAV_STATE" System status flag

		ROS_DEBUG_STREAM_NAMED("companion_process_status", "Companion process status: " << utils::to_string_enum<COMPANION_PROCESS_STATE>(heartbeat.system_status)
				<< std::endl << heartbeat.to_yaml());

		UAS_FCU(m_uas)->send_message_ignore_drop(heartbeat);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CompanionProcessStatusPlugin, mavros::plugin::PluginBase)
