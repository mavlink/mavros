/**
 * @brief Terrain plugin
 * @file terrain.cpp
 * @author Matt Anderson <anderson_rayner@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Ardupilot.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/Terrain.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Terrain height plugin.
 *
 * This plugin allows publishing of terrain height estimate from FCU to ROS.
 *
 */
class TerrainPlugin : public plugin::PluginBase {
public:
	TerrainPlugin() : PluginBase(),
		terrain_nh("~terrain")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		terrain_pub = terrain_nh.advertise<mavros_msgs::Terrain>("in", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&TerrainPlugin::handle_terrain_report)
		};
	}

private:
	ros::NodeHandle terrain_nh;

	ros::Publisher terrain_pub;

	void handle_terrain_report(const mavlink::mavlink_message_t *msg, mavlink::common::msg::TERRAIN_REPORT &terrain) {
		auto terrain_msg = boost::make_shared<mavros_msgs::Terrain>();

		terrain_msg->header.stamp = ros::Time::now();
		terrain_msg->header.frame_id = "terrain";

		terrain_msg->latitude = (double) terrain.lat / 1e7;
		terrain_msg->longitude = (double) terrain.lon / 1e7;
		terrain_msg->spacing = terrain.spacing;
		terrain_msg->terrain_height = terrain.terrain_height;
		terrain_msg->current_height = terrain.current_height;
		terrain_msg->pending = terrain.pending;
		terrain_msg->loaded = terrain.loaded;

		terrain_pub.publish(terrain_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TerrainPlugin, mavros::plugin::PluginBase)
