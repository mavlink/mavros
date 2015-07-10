/**
 * @brief Copter visualization
 * @file copter_visualization.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 */
/*
 * Copyright 2014,2015 M.H.Kabir
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// parameters
static std::string fixed_frame_id;
static std::string child_frame_id;
static double marker_scale;

// merker publishers
ros::Publisher track_marker_pub;
ros::Publisher vehicle_marker_pub;


/**
 * @brief publish vehicle track
 */
static void publish_track_marker(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
	static int marker_id = 0;

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

	track_marker_pub.publish(marker);
}

/**
 * @brief publish vehicle
 */
static void publish_vehicle_marker() {
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
	marker->id++; // in older code there be decrement
	vehicle_marker_pub.publish(marker);

	marker->pose.position.x = 0.19 * marker_scale;
	marker->pose.position.y = -0.11 * marker_scale;
	marker->id++;
	vehicle_marker_pub.publish(marker);

	// left/right
	marker->pose.position.x = 0;
	marker->pose.position.y = 0.22 * marker_scale;
	marker->id++;
	vehicle_marker_pub.publish(marker);

	marker->pose.position.x = 0;
	marker->pose.position.y = -0.22 * marker_scale;
	marker->id++;
	vehicle_marker_pub.publish(marker);

	// back left/right
	marker->pose.position.x = -0.19 * marker_scale;
	marker->pose.position.y = 0.11 * marker_scale;
	marker->id++;
	vehicle_marker_pub.publish(marker);

	marker->pose.position.x = -0.19 * marker_scale;
	marker->pose.position.y = -0.11 * marker_scale;
	marker->id++;
	vehicle_marker_pub.publish(marker);

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
	marker->id++;
	vehicle_marker_pub.publish(marker);

	// 30 deg rotation  0.9659  0  0  0.2588
	marker->pose.orientation.w = 0.9659;
	marker->pose.orientation.z = 0.2588;
	marker->id++;
	vehicle_marker_pub.publish(marker);

	marker->pose.orientation.w = 0.9659;
	marker->pose.orientation.z = -0.2588;
	marker->id++;
	vehicle_marker_pub.publish(marker);
}

static void local_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
	publish_track_marker(pose);
	publish_vehicle_marker();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "copter_visualization");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	priv_nh.param<std::string>("fixed_frame_id", fixed_frame_id, "local_origin");
	priv_nh.param<std::string>("child_frame_id", child_frame_id, "fcu");
	priv_nh.param("marker_scale", marker_scale, 2.0);

	track_marker_pub = nh.advertise<visualization_msgs::Marker>("track_markers", 10);
	vehicle_marker_pub = nh.advertise<visualization_msgs::Marker>("vehicle_marker", 10);

	auto sub = nh.subscribe("local_position", 10, local_position_sub_cb);

	ros::spin();
	return 0;
}
