/**
 * @brief Avoidance Status plugin
 * @file avoidance_status.cpp
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

#include <mavros_msgs/AvoidanceStatus.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink OBS_AVOID_STATUS enumeration
using mavlink::common::OBS_AVOID_STATUS;

/**
 * @brief Obstacle avoidance status plugin
 *
 * Publishes the status of the obstacle avoidance algorithm
 * @see status_cb()
 */
class AvoidanceStatusPlugin : public plugin::PluginBase {
public:
	AvoidanceStatusPlugin() : PluginBase(),
	status_nh("~avoidance")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		status_sub = status_nh.subscribe("status", 10, &AvoidanceStatusPlugin::status_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle status_nh;
	ros::Subscriber status_sub;

	/**
	 * @brief Send avoidance status to FCU
	 *
	 * Message specification: http://mavlink.org/messages/common#AVOIDANCE_STATUS
	 * @param req	received AvoidanceStatus msg
	 */
	void status_cb(const mavros_msgs::AvoidanceStatus::ConstPtr &req)
	{
		mavlink::common::msg::AVOIDANCE_STATUS status_msg {};

		status_msg.time_usec = req->header.stamp.toNSec() / 1000;			//!< [milisecs]
		status_msg.status = req->status;									//!< status in [0,4] see Mavlink OBS_AVOID_STATUS enumeration


		ROS_DEBUG_STREAM_NAMED("avoidance_status", "Avoidance status: " << utils::to_string_enum<OBS_AVOID_STATUS>(status_msg.status)
				<< std::endl << status_msg.to_yaml());

		UAS_FCU(m_uas)->send_message_ignore_drop(status_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::AvoidanceStatusPlugin, mavros::plugin::PluginBase)
