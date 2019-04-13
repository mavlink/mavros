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
//! Radians to degrees
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
//! Mavlink MAV_DISTANCE_SENSOR enumeration
using mavlink::common::MAV_DISTANCE_SENSOR;

/**
 * @brief Obstacle distance plugin
 *
 * Publishes obstacle distance array to the FCU, in order to assist in an obstacle
 * avoidance flight.
 * @see obstacle_cb()
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
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
	 * @param req	received ObstacleDistance msg
	 */
	void obstacle_cb(const sensor_msgs::LaserScan::ConstPtr &req)
	{
		mavlink::common::msg::OBSTACLE_DISTANCE obstacle {};

		if (req->ranges.size() <= obstacle.distances.size()) {
			// all distances from sensor will fit in obstacle distance message
			Eigen::Map<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1> > map_distances(obstacle.distances.data(), req->ranges.size());
			auto cm_ranges = Eigen::Map<const Eigen::VectorXf>(req->ranges.data(), req->ranges.size()) * 1e2;
			map_distances = cm_ranges.cast<uint16_t>();							//!< [centimeters]
			std::fill(obstacle.distances.begin() + req->ranges.size(), obstacle.distances.end(), UINT16_MAX);    //!< fill the rest of the array values as "Unknown"
			obstacle.increment = req->angle_increment * RAD_TO_DEG;				//!< [degrees]
		} else {
			// all distances from sensor will not fit so we combine adjacent distances always taking the shortest distance
			size_t scale_factor = ceil(double(req->ranges.size()) / obstacle.distances.size());
			for (size_t i = 0; i < obstacle.distances.size(); i++) {
				obstacle.distances[i] = UINT16_MAX;
				for (size_t j = 0; j < scale_factor; j++) {
					size_t req_index = i * scale_factor + j;
					if (req_index < req->ranges.size()) {
						const float dist_m = req->ranges[req_index];
						if (!std::isnan(dist_m)) {
							obstacle.distances[i] = std::min(obstacle.distances[i], (uint16_t)(dist_m * 1e2));
						}
					}
				}
			}
			obstacle.increment = ceil(req->angle_increment * RAD_TO_DEG * scale_factor);   //!< [degrees]
		}

		obstacle.time_usec = req->header.stamp.toNSec() / 1000;					//!< [microsecs]
		obstacle.sensor_type = utils::enum_value(MAV_DISTANCE_SENSOR::LASER);	//!< defaults is laser type (depth sensor, Lidar)
		obstacle.min_distance = req->range_min * 1e2;							//!< [centimeters]
		obstacle.max_distance = req->range_max * 1e2;							//!< [centimeters]

		ROS_DEBUG_STREAM_NAMED("obstacle_distance", "OBSDIST: sensor type: " << utils::to_string_enum<MAV_DISTANCE_SENSOR>(obstacle.sensor_type)
				<< std::endl << obstacle.to_yaml());

		UAS_FCU(m_uas)->send_message_ignore_drop(obstacle);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ObstacleDistancePlugin, mavros::plugin::PluginBase)
