/**
 * @brief Obstacle distance plugin
 * @file obstacle_distance.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.mdA
 */

#include <mavros/mavros_plugin.h>

#include <sensor_msgs/LaserScan.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::MAV_DISTANCE_SENSOR;
/**
 * @brief Obstacle distance plugin
 *
 * Publishes obstacle distance array to the FCU, in order to assist in an obstacle
 * avoidance flight.
 */
class ObstacleDistancePlugin : public plugin::PluginBase {
public:
	ObstacleDistancePlugin() : PluginBase(),
		obstacle_nh("~obstacle")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		obstacle_sub = obstacle_nh.subscribe("send", 10, &ObstacleDistancePlugin::obstacle_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle obstacle_nh;

	ros::Subscriber obstacle_sub;

	/**
	 * @brief Send obstacle distance array to the FCU.
	 * Message specification: @p https://pixhawk.ethz.ch/mavlink/#OBSTACLE_DISTANCE
	 * @param req	received ObstacleDistance msg
	 */
	void obstacle_cb(const sensor_msgs::LaserScan::ConstPtr &req)
	{
		mavlink::common::msg::OBSTACLE_DISTANCE obstacle {};

		constexpr size_t MAX_DISTCNT = 72;

		auto n = std::min(req->ranges.size(), MAX_DISTCNT);

		obstacle.time_usec = req->header.stamp.toNSec() / 1000;	// [milisecs]
		obstacle.estimator_type = utils::enum_value(MAV_DISTANCE_SENSOR::LASER);	// @todo: typo correction required on the Mavlink definition (should be sensor_type)
		std::copy(req->ranges.begin(), req->ranges.begin() + n, obstacle.distances.begin());
		std::fill(obstacle.distances.begin() + n, obstacle.distances.end(), UINT8_MAX);	// check https://github.com/mavlink/mavlink/pull/807#issuecomment-347885080
		obstacle.increment = req->angle_increment;		// [rads]

		ROS_DEBUG_STREAM_NAMED("obstacle_distance", "OBSDIST: sensor type: " << utils::to_string_enum<MAV_DISTANCE_SENSOR>(obstacle.estimator_type)
										     << " distances: [" << obstacle.distance << "] "
										     << " increment: " << obstacle.increment);

		UAS_FCU(m_uas)->send_message_ignore_drop(obstacle);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ObstacleDistancePlugin, mavros::plugin::PluginBase)
