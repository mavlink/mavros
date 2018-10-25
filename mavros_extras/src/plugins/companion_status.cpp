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

#include <mavros_msgs/CompanionStatus.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink enumerations
using mavlink::common::COMPANION_STATE;
using mavlink::common::COMPANION_SOURCE;

/**
 * @brief Obstacle companion status plugin
 *
 * Publishes the status of components running on the companion computer
 * @see status_cb()
 */
class CompanionStatusPlugin : public plugin::PluginBase {
public:
	CompanionStatusPlugin() : PluginBase(),
	status_nh("~companion")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		status_sub = status_nh.subscribe("status", 10, &CompanionStatusPlugin::status_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle status_nh;
	ros::Subscriber status_sub;

	/**
	 * @brief Send companion status to FCU
	 *
	 * Message specification: http://mavlink.org/messages/common#COMPANION_STATUS
	 * @param req	received CompanionStatus msg
	 */
	void status_cb(const mavros_msgs::CompanionStatus::ConstPtr &req)
	{
		mavlink::common::msg::COMPANION_STATUS status_msg {};

		status_msg.time_usec = req->header.stamp.toNSec() / 1000;			//!< [milisecs]
		status_msg.state = req->state;									//!< status in [0,4] see Mavlink COMPANION_STATE enumeration
		status_msg.source = req->source;								//!< source in [0,2] see Mavlink COMPANION_SOURCE enumeration


		ROS_DEBUG_STREAM_NAMED("companion_status", "Companion status: " << utils::to_string_enum<COMPANION_STATE>(status_msg.state)
				<< " source: "<< utils::to_string_enum<COMPANION_SOURCE>(status_msg.source) << std::endl << status_msg.to_yaml());

		UAS_FCU(m_uas)->send_message_ignore_drop(status_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CompanionStatusPlugin, mavros::plugin::PluginBase)
