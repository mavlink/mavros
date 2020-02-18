/**
 * @brief Visualization
 * @file visualization.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 */
/*
 * Copyright 2014,2015 M.H.Kabir
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <vector>

#include <tf/tf.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// parameters
static std::string fixed_frame_id;
static std::string child_frame_id;
static double marker_scale;
static int max_track_size = 100;

// source subscribers
ros::Subscriber local_position_sub;
ros::Subscriber landing_target_sub;
ros::Subscriber lt_marker_sub;

// marker publishers
ros::Publisher track_marker_pub;
ros::Publisher vehicle_marker_pub;
ros::Publisher lt_marker_pub;
ros::Publisher wp_marker_pub;

// landing target marker size
geometry_msgs::Vector3 lt_size;

boost::shared_ptr<visualization_msgs::MarkerArray> vehicle_marker;

/**
 * @brief publish vehicle track
 */
static void publish_track_marker(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
	static boost::shared_ptr<visualization_msgs::Marker> track_marker;

	if ( !track_marker )
	{
		track_marker = boost::make_shared<visualization_msgs::Marker>();
		track_marker->type = visualization_msgs::Marker::CUBE_LIST;
		track_marker->ns = "fcu";
		track_marker->action = visualization_msgs::Marker::ADD;
		track_marker->scale.x = marker_scale * 0.015;
		track_marker->scale.y = marker_scale * 0.015;
		track_marker->scale.z = marker_scale * 0.015;
		track_marker->color.a = 1.0;
		track_marker->color.r = 0.0;
		track_marker->color.g = 0.0;
		track_marker->color.b = 0.5;
		track_marker->points.reserve(max_track_size);
	}

	static int marker_idx = 0;

	if ( track_marker->points.size() < max_track_size )
		track_marker->points.push_back(pose->pose.position);
	else track_marker->points[marker_idx] = pose->pose.position;

	marker_idx = ++marker_idx % max_track_size;

	track_marker->header = pose->header;
	track_marker_pub.publish(track_marker);
}

static void publish_wp_marker(const geometry_msgs::PoseStamped::ConstPtr &wp)
{
	static boost::shared_ptr<visualization_msgs::Marker> marker;

	if ( !marker )	// only instantiate marker once
	{
		marker = boost::make_shared<visualization_msgs::Marker>();

		marker->header = wp->header;
		marker->header.frame_id = fixed_frame_id;
		marker->type = visualization_msgs::Marker::ARROW;
		marker->ns = "wp";
		marker->action = visualization_msgs::Marker::ADD;
		marker->scale.x = marker_scale * 1.0;
		marker->scale.y = marker_scale * 0.1;
		marker->scale.z = marker_scale * 0.1;

		marker->color.a = 1.0;
		marker->color.r = 0.0;
		marker->color.g = 1.0;
		marker->color.b = 0.0;
	}

	marker->pose = wp->pose;
	wp_marker_pub.publish(marker);
}


/**
 * @brief publish landing target marker
 */
static void publish_lt_marker(const geometry_msgs::PoseStamped::ConstPtr &target)
{
	static int marker_id = 2;	// TODO: generate new marker for each target

	auto marker = boost::make_shared<visualization_msgs::Marker>();

	marker->header = target->header;
	marker->ns = "landing_target";
	marker->id = marker_id;
	marker->type = visualization_msgs::Marker::CUBE;
	marker->action = visualization_msgs::Marker::ADD;

	marker->color.a = 1.0;
	marker->color.r = 1.0;
	marker->color.g = 0.0;
	marker->color.b = 0.0;

	marker->scale.x = 1.0;
	marker->scale.y = 1.0;
	marker->scale.z = 1.0;

	// origin
	marker->pose = target->pose;
	lt_marker_pub.publish(marker);

	// cross arms
	marker->pose.position.x = lt_size.x;
	marker->pose.position.y = lt_size.y;
	marker->pose.position.z = lt_size.z;

	marker->id = ++marker_id;
	marker->pose.orientation.w = 0;
	marker->pose.orientation.x = 0;
	marker->pose.orientation.y = 0;
	marker->pose.orientation.w = 0;
	lt_marker_pub.publish(marker);

	marker->id = ++marker_id;
	// 90 degrees rotation
	marker->pose.orientation.w = 0.70711;
	marker->pose.orientation.x = 0;
	marker->pose.orientation.y = 0;
	marker->pose.orientation.w = 0.70711;
	lt_marker_pub.publish(marker);
}

/**
 * @brief publish vehicle
 */
static void create_vehicle_markers( int num_rotors, float arm_len, float body_width, float body_height, int prop_direction)
{
	if ( num_rotors <= 0 ) num_rotors = 2;

	/** Create markers only once for efficiency
	 *  TODO use visualization_msgs::MarkerArray?
	 */

	if ( vehicle_marker )
		return;

	vehicle_marker = boost::make_shared<visualization_msgs::MarkerArray>();
	vehicle_marker->markers.reserve( 2 * num_rotors + 1 );

	/** Hexacopter marker code adapted from libsfly_viz
	 *  thanks to Markus Achtelik.
	 */

	// rotor marker template
	visualization_msgs::Marker rotor;
	rotor.header.stamp = ros::Time();
	rotor.header.frame_id = child_frame_id;
	rotor.ns = "vehicle_rotor";
	rotor.action = visualization_msgs::Marker::ADD;
	rotor.type = visualization_msgs::Marker::CYLINDER;
	rotor.scale.x = 0.2 * marker_scale;
	rotor.scale.y = 0.2 * marker_scale;
	rotor.scale.z = 0.01 * marker_scale;
	rotor.pose.position.z = 0;

	// arm marker template
	visualization_msgs::Marker arm;
	arm.header.stamp = ros::Time();
	arm.header.frame_id = child_frame_id;
	arm.ns = "vehicle_arm";
	arm.action = visualization_msgs::Marker::ADD;
	arm.type = visualization_msgs::Marker::CUBE;
	arm.scale.x = arm_len * marker_scale;
	arm.scale.y = 0.02 * marker_scale;
	arm.scale.z = 0.01 * marker_scale;
	arm.color.r = 0.0;
	arm.color.g = 0.0;
	arm.color.b = 1.0;
	arm.color.a = 1.0;
	arm.pose.position.z = -0.015 * marker_scale;

	float angle_increment = 2 * M_PI / num_rotors;

	for ( float angle = angle_increment / 2; angle <= (2 * M_PI); angle += angle_increment )
	{
		if ( !prop_direction ) {
			rotor.color.r = 0.4;
			rotor.color.g = 0.4;
			rotor.color.b = 0.4;
			rotor.color.a = 0.8;
		} else {
			if ( angle <= (M_PI / 2) - 0.0175 || angle >= (M_PI * 3 / 2) + 0.0175 ) {
				rotor.color.r = 0.8;
				rotor.color.g = 0.8;
				rotor.color.b = 0.8;
				rotor.color.a = 0.8;
			} else {
				rotor.color.r = 1.0;
				rotor.color.g = 0;
				rotor.color.b = 0;
				rotor.color.a = 0.8;
			}
		}
		
		
		rotor.pose.position.x = arm_len * cos(angle) * marker_scale;
		rotor.pose.position.y = arm_len * sin(angle) * marker_scale;
		rotor.id++;

		arm.pose.position.x = rotor.pose.position.x / 2;
		arm.pose.position.y = rotor.pose.position.y / 2;
		arm.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
		arm.id++;

		vehicle_marker->markers.push_back(rotor);
		vehicle_marker->markers.push_back(arm);
	}

	// body marker template
	visualization_msgs::Marker body;
	body.header.stamp = ros::Time();
	body.header.frame_id = child_frame_id;
	body.ns = "vehicle_body";
	body.action = visualization_msgs::Marker::ADD;
	body.type = visualization_msgs::Marker::CUBE;
	body.scale.x = body_width * marker_scale;
	body.scale.y = body_width * marker_scale;
	body.scale.z = body_height * marker_scale;
	body.color.r = 0.0;
	body.color.g = 1.0;
	body.color.b = 0.0;
	body.color.a = 0.8;

	vehicle_marker->markers.push_back(body);
}

static void local_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
	publish_track_marker(pose);
	if (vehicle_marker) vehicle_marker_pub.publish(vehicle_marker);
}

void setpoint_local_pos_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &wp)
{
	publish_wp_marker(wp);
}

static void landing_target_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &target)
{
	publish_lt_marker(target);
}

static void lt_marker_sub_cb(const geometry_msgs::Vector3Stamped::ConstPtr &lt_marker)
{
	lt_size = lt_marker->vector;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "copter_visualization");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	int num_rotors, prop_direction;
	double arm_len, body_width, body_height;

	priv_nh.param<std::string>("fixed_frame_id", fixed_frame_id, "map");
	priv_nh.param<std::string>("child_frame_id", child_frame_id, "base_link");

	priv_nh.param("marker_scale", marker_scale, 1.0);
	priv_nh.param("num_rotors", num_rotors, 6);
	priv_nh.param("arm_len", arm_len, 0.22 );
	priv_nh.param("body_width", body_width, 0.15 );
	priv_nh.param("body_height", body_height, 0.10 );
	priv_nh.param("max_track_size", max_track_size, 1000 );
	priv_nh.param("prop_direction", prop_direction, 0);

	create_vehicle_markers( num_rotors, arm_len, body_width, body_height, prop_direction );

	track_marker_pub = nh.advertise<visualization_msgs::Marker>("track_markers", 10);
	vehicle_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("vehicle_marker", 10);
	wp_marker_pub = nh.advertise<visualization_msgs::Marker>("wp_markers", 10);
	lt_marker_pub = nh.advertise<visualization_msgs::Marker>("landing_target", 10);

	auto pos_sub = nh.subscribe("local_position", 10, local_position_sub_cb);
	auto wp_sub = nh.subscribe("local_setpoint", 10, setpoint_local_pos_sub_cb);
	lt_marker_sub = nh.subscribe("lt_marker", 10, lt_marker_sub_cb);

	ros::spin();
	return 0;
}
