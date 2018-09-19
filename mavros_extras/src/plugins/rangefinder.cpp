/**
 * @brief Rangefinder plugin
 * @file rangefinder.cpp
 * @author Pierre Kancir <khancyr@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Ardupilot.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <sensor_msgs/Range.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Ardupilot Rangefinder plugin.
 *
 * This plugin allows publishing rangefinder sensor data from Ardupilot FCU to ROS.
 *
 */
class RangefinderPlugin : public plugin::PluginBase {
public:
	RangefinderPlugin() : PluginBase(),
		rangefinder_nh("~rangefinder")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		rangefinder_pub = rangefinder_nh.advertise<sensor_msgs::Range>("rangefinder", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&RangefinderPlugin::handle_rangefinder)
		};
	}

private:
	ros::NodeHandle rangefinder_nh;

	ros::Publisher rangefinder_pub;

	void handle_rangefinder(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::RANGEFINDER &rangefinder) {
		auto rangefinder_msg = boost::make_shared<sensor_msgs::Range>();
		rangefinder_msg->header.stamp = ros::Time::now();
		rangefinder_msg->header.frame_id = "/rangefinder";
		rangefinder_msg->radiation_type = sensor_msgs::Range::INFRARED;
		rangefinder_msg->field_of_view = 0;
		rangefinder_msg->min_range = 0;
		rangefinder_msg->max_range = 1000;
		rangefinder_msg->range = rangefinder.distance;

		rangefinder_pub.publish(rangefinder_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RangefinderPlugin, mavros::plugin::PluginBase)
