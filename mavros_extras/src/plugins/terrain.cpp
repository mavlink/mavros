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
#include <mavros_msgs/TerrainReport.h>

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

		terrain_report_pub = terrain_nh.advertise<mavros_msgs::TerrainReport>("report", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&TerrainPlugin::handle_terrain_report)
		};
	}

private:
	ros::NodeHandle terrain_nh;

	ros::Publisher terrain_report_pub;

	void handle_terrain_report(const mavlink::mavlink_message_t *msg, mavlink::common::msg::TERRAIN_REPORT &report) {
		auto terrain_report_msg = boost::make_shared<mavros_msgs::TerrainReport>();

		terrain_report_msg->header.stamp = ros::Time::now();
		terrain_report_msg->header.frame_id = "terrain";

		terrain_report_msg->latitude = (double) report.lat / 1e7;
		terrain_report_msg->longitude = (double) report.lon / 1e7;
		terrain_report_msg->spacing = report.spacing;
		terrain_report_msg->terrain_height = report.terrain_height;
		terrain_report_msg->current_height = report.current_height;
		terrain_report_msg->pending = report.pending;
		terrain_report_msg->loaded = report.loaded;

		terrain_report_pub.publish(terrain_report_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TerrainPlugin, mavros::plugin::PluginBase)
