/**
 * @brief visualization plugin
 * @file visualization.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/Marker.h>

namespace mavplugin {
/**
 * @brief RViz Visualiser plugin.
 * Display track of MAV's path in RViz and current position target.
 */
class VisualizationPlugin : public MavRosPlugin {
public:
	VisualizationPlugin() :
		viz_nh("~visualization"),
		uas(nullptr),
		marker_scale(0),
		marker_id(0)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		viz_nh.param<std::string>("fixed_frame_id", fixed_frame_id, "local_origin");
		viz_nh.param<std::string>("child_frame_id", child_frame_id, "fcu");
		viz_nh.param<double>("marker_scale", marker_scale, 2.0);

		track_marker = viz_nh.advertise<visualization_msgs::Marker>("track_markers", 10);
		vehicle_marker = viz_nh.advertise<visualization_msgs::Marker>("vehicle_marker", 10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_LOCAL_POSITION_NED, &VisualizationPlugin::handle_local_position_ned)
				// XXX Add handling of position target.
		};
	}

private:
	ros::NodeHandle viz_nh;
	UAS *uas;

	ros::Publisher track_marker;
	ros::Publisher vehicle_marker;

	std::string fixed_frame_id;
	std::string child_frame_id;	// frame for visualization markers

	double marker_scale;
	int marker_id;

	void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(msg, &pos_ned);

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pos_ned.y, pos_ned.x, -pos_ned.z));
		transform.setRotation(uas->get_attitude_orientation());

		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();

		tf::poseTFToMsg(transform, pose->pose);
		pose->header.frame_id = fixed_frame_id;
		pose->header.stamp = ros::Time();

		publish_vehicle_marker();
		publish_vis_marker(pose);
	}

	void publish_vis_marker(geometry_msgs::PoseStampedPtr pose)
	{
		auto marker = boost::make_shared<visualization_msgs::Marker>();

		marker->header = pose->header;
		marker->type = visualization_msgs::Marker::CUBE;
		marker->ns = "fcu";
		marker->id = marker_id++;
		marker->action = visualization_msgs::Marker::ADD;
		marker->pose = pose->pose;
		marker->scale.x = marker_scale * 0.015;
		marker->scale.y = marker_scale * 0.015;
		marker->scale.z = marker_scale * 0.015;

		marker->color.a = 1.0;
		marker->color.r = 0.0;
		marker->color.g = 0.0;
		marker->color.b = 0.5;

		track_marker.publish(marker);
	}

	void publish_vehicle_marker() {
		/** Hexacopter marker code adapted from libsfly_viz
		 *  thanks to Markus Achtelik.
		 */

		const double sqrt2_2 = sqrt(2) / 2;
		int id = 1;

		auto marker = boost::make_shared<visualization_msgs::Marker>();

		// the marker will be displayed in frame_id
		marker->header.stamp = ros::Time();
		marker->header.frame_id = child_frame_id;
		marker->ns = "fcu";
		marker->action = visualization_msgs::Marker::ADD;
		marker->id = id;

		// make rotors
		marker->type = visualization_msgs::Marker::CYLINDER;
		marker->scale.x = 0.2 * marker_scale;
		marker->scale.y = 0.2 * marker_scale;
		marker->scale.z = 0.01 * marker_scale;
		marker->color.r = 0.4;
		marker->color.g = 0.4;
		marker->color.b = 0.4;
		marker->color.a = 0.8;
		marker->pose.position.z = 0;

		// front left/right
		marker->pose.position.x = 0.19 * marker_scale;
		marker->pose.position.y = 0.11 * marker_scale;
		marker->id--;
		vehicle_marker.publish(marker);

		marker->pose.position.x = 0.19 * marker_scale;
		marker->pose.position.y = -0.11 * marker_scale;
		marker->id--;
		vehicle_marker.publish(marker);

		// left/right
		marker->pose.position.x = 0;
		marker->pose.position.y = 0.22 * marker_scale;
		marker->id--;
		vehicle_marker.publish(marker);

		marker->pose.position.x = 0;
		marker->pose.position.y = -0.22 * marker_scale;
		marker->id--;
		vehicle_marker.publish(marker);

		// back left/right
		marker->pose.position.x = -0.19 * marker_scale;
		marker->pose.position.y = 0.11 * marker_scale;
		marker->id--;
		vehicle_marker.publish(marker);

		marker->pose.position.x = -0.19 * marker_scale;
		marker->pose.position.y = -0.11 * marker_scale;
		marker->id--;
		vehicle_marker.publish(marker);

		// make arms
		marker->type = visualization_msgs::Marker::CUBE;
		marker->scale.x = 0.44 * marker_scale;
		marker->scale.y = 0.02 * marker_scale;
		marker->scale.z = 0.01 * marker_scale;
		marker->color.r = 0.0;
		marker->color.g = 0.0;
		marker->color.b = 1.0;
		marker->color.a = 1.0;

		marker->pose.position.x = 0;
		marker->pose.position.y = 0;
		marker->pose.position.z = -0.015 * marker_scale;
		marker->pose.orientation.x = 0;
		marker->pose.orientation.y = 0;

		marker->pose.orientation.w = sqrt2_2;
		marker->pose.orientation.z = sqrt2_2;
		marker->id--;
		vehicle_marker.publish(marker);

		// 30 deg rotation  0.9659  0  0  0.2588
		marker->pose.orientation.w = 0.9659;
		marker->pose.orientation.z = 0.2588;
		marker->id--;
		vehicle_marker.publish(marker);

		marker->pose.orientation.w = 0.9659;
		marker->pose.orientation.z = -0.2588;
		marker->id--;
		vehicle_marker.publish(marker);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VisualizationPlugin, mavplugin::MavRosPlugin)

