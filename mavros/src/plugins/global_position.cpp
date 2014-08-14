/**
 * @brief Global Position plugin
 * @file global_position.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>

#include "gps_conversions.h"

namespace mavplugin {

/**
 * @brief Global position plugin.
 * 
 * Publishes global position. Convertion from GPS to UTF allows
 * publishing local position to TF and PoseWithCovarianceStamped.
 * 
 */
class GlobalPositionPlugin : public MavRosPlugin {
public:
	GlobalPositionPlugin() :
		uas(nullptr),
		send_tf(false),
		rot_cov(99999.0)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		gp_nh = ros::NodeHandle(nh, "global_position");

		gp_nh.param("global/send_tf", send_tf, true);
		gp_nh.param<std::string>("global/frame_id", frame_id, "local_origin");
		gp_nh.param<std::string>("global/child_frame_id", child_frame_id, "fcu");
		gp_nh.param<double>("rot_covariance", rot_cov, 99999.0);

		fix_pub = gp_nh.advertise<sensor_msgs::NavSatFix>("global", 10);
		pos_pub = gp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("local", 10);
		vel_pub = gp_nh.advertise<geometry_msgs::Vector3Stamped>("gps_vel", 10);
		rel_alt_pub = gp_nh.advertise<std_msgs::Float64>("rel_alt", 10);
		hdg_pub = gp_nh.advertise<std_msgs::Float64>("compass_hdg", 10);
	}

	std::string const get_name() const {
		return "GlobalPosition";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &GlobalPositionPlugin::handle_global_position_int)
		};
	}

private:
	UAS *uas;

	ros::NodeHandle gp_nh;
	ros::Publisher fix_pub;
	ros::Publisher pos_pub;
	ros::Publisher vel_pub;
	ros::Publisher hdg_pub;
	ros::Publisher rel_alt_pub;
	
	tf::TransformBroadcaster tf_broadcaster;

	std::string frame_id;		//!< origin for TF
	std::string child_frame_id;	//!< frame for TF and Pose
	bool send_tf;
	double rot_cov;

	void handle_global_position_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_global_position_int_t gp_pos;
		mavlink_msg_global_position_int_decode(msg, &gp_pos);
	
		ROS_DEBUG_THROTTLE_NAMED(10, "global_position", "Local position NED: boot_ms:%06d "
				"lat/lon/alt:(%d %d %d) relative_alt: (%d) gps_vel:(%d %d %d) compass_heading: (%d)",
				gp_pos.time_boot_ms,
				gp_pos.lat, gp_pos.lon, gp_pos.alt, gp_pos.relative_alt,
				gp_pos.vx, gp_pos.vy, gp_pos.vz, gp_pos.hdg);
		/**
		 * TODO: Upgrade to GLOBAL_POSITION_INT_COV so to get the covariance
		 */
		
		geometry_msgs::PoseWithCovarianceStampedPtr pose_cov = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
		
		sensor_msgs::NavSatFixPtr gps_cord = boost::make_shared<sensor_msgs::NavSatFix>();
		geometry_msgs::Vector3StampedPtr gps_vel =  boost::make_shared<geometry_msgs::Vector3Stamped>();
		std_msgs::Float64Ptr relative_alt = boost::make_shared<std_msgs::Float64>();
		std_msgs::Float64Ptr compass_heading = compass_heading = boost::make_shared<std_msgs::Float64>();
		
		/**
		 * TODO: Check if PX4 sends time stamp of GPS
		 *       or use system time handler...
		 */
		
		//gps_cord->header.stamp = gp_pos.time_boot_ms;
		gps_cord->header.frame_id = frame_id;
		gps_cord->latitude = gp_pos.lat;
		gps_cord->longitude = gp_pos.lon;
		gps_cord->altitude = gp_pos.alt;
		
		//gps_vel->header.stamp = gp_pos.time_boot_ms;
		gps_vel->header.frame_id = frame_id;
		gps_vel->vector.x = gp_pos.vx;
		gps_vel->vector.y = gp_pos.vy;
		gps_vel->vector.z = gp_pos.vz;
		
		compass_heading->data;
		relative_alt->data;
		
		/** Adapted from gps_umd ROS package [http://wiki.ros.org/gps_umd?distro=hydro]
		 *  Author: Ken Tossell <ken AT tossell DOT net>
		 */
		
		double northing, easting;
		std::string zone;
		
		mav_plugin::LLtoUTM(gps_cord->latitude, gps_cord->longitude, northing, easting, zone);
		
		//pose_cov->header.stamp = time_boot_ms;
		pose_cov->header.frame_id = frame_id;
		pose_cov->pose.pose.position.x = easting;
		pose_cov->pose.pose.position.y = northing;
		pose_cov->pose.pose.position.z = gp_pos.alt;
		
		pose_cov->pose.pose.orientation.x = 0;
		pose_cov->pose.pose.orientation.y = 0;
		pose_cov->pose.pose.orientation.z = 0;
		pose_cov->pose.pose.orientation.w = 1;
		
		// Use ENU covariance to build XYZRPY covariance
		boost::array<double, 36> covariance = {{
		gps_cord->position_covariance[0],
		gps_cord->position_covariance[1],
		gps_cord->position_covariance[2],
		0, 0, 0,
		gps_cord->position_covariance[3],
		gps_cord->position_covariance[4],
		gps_cord->position_covariance[5],
		0, 0, 0,
		gps_cord->position_covariance[6],
		gps_cord->position_covariance[7],
		gps_cord->position_covariance[8],
		0, 0, 0,
		0, 0, 0, rot_cov, 0, 0,
		0, 0, 0, 0, rot_cov, 0,
		0, 0, 0, 0, 0, rot_cov
		}};	  
		
		fix_pub.publish(gps_cord);
		pos_pub.publish(pose_cov);
		vel_pub.publish(gps_vel);
	
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pose_cov->pose.pose.position.y,
						 pose_cov->pose.pose.position.x,
						-pose_cov->pose.pose.position.z));
		
		transform.setRotation(tf::Quaternion(pose_cov->pose.pose.orientation.y,
						      pose_cov->pose.pose.orientation.x,
						     -pose_cov->pose.pose.orientation.z,
						      pose_cov->pose.pose.orientation.w));				    

		if (send_tf)
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						transform,
						pose_cov->header.stamp,
						frame_id, child_frame_id));		
	}
	
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::GlobalPositionPlugin, mavplugin::MavRosPlugin)