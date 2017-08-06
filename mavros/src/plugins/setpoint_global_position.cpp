/**
 * @brief SetpointGlobalPosition plugin
 * @file setpoint_global_position.cpp
 * @author Mohamed Abdelkader 2017 <mohamedashraf123@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017, Mohamed Abdelkader.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#define M_DEG_TO_RAD 0.01745329251994
#define CONSTANTS_RADIUS_OF_EARTH 6371000.0
//#define DBL_EPSILON 2.2204460492503131E-16

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint global position plugin
 *
 * Send global setpoint positions to FCU controller.
 */
class SetpointGlobalPositionPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointGlobalPositionPlugin>{
public:
	SetpointGlobalPositionPlugin() : PluginBase(),
		// NOTE not private handle, because we need to access gps topic, which is not under this node. Is it OK?
		sp_nh("")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		/* subscriber for current gps state, mavros/global_position/global. */
		gps_sub = sp_nh.subscribe("global_position/global", 10, &SetpointGlobalPositionPlugin::gps_cb, this);
		/* Subscribe for current local pose. */
		local_sub = sp_nh.subscribe("local_position/pose", 10, &SetpointGlobalPositionPlugin::local_cb, this) ;
		/* Subscriber for target gps state */
		setpoint_sub = sp_nh.subscribe("setpoint_global_position/global", 10, &SetpointGlobalPositionPlugin::setpoint_cb, this);

	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber setpoint_sub;
	ros::Subscriber gps_sub;
	ros::Subscriber local_sub;

	/* resulting change in x/y in NED frame. Set by lla2ned() function. */
	double dx, dy;

	/* Stores current gps state. */
	sensor_msgs::NavSatFix gps_msg;
	/* Current local pose */
	geometry_msgs::PoseStamped local_pose_msg;
	/* old time gps time stamp in [ms], to check if new gps msg is received */
	uint32_t old_gps_stamp = 0;

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Converts lat/long to 'change' in local NED frme
	 * @param curr_lat. Current latitude in deg
	 * @param curr_long. Current longitude in deg
	 * @param target_lat. Target latitude in deg
	 * @param target_long. Target longitude in deg
	 */
	 void lla2ned(double curr_lat, double curr_long, double target_lat, double target_long){
		 /* Convert to radins */
		double curr_lat_rad = curr_lat * M_DEG_TO_RAD;
		double curr_lon_rad = curr_long * M_DEG_TO_RAD;

		double curr_sin_lat = sin(curr_lat_rad);
		double curr_cos_lat = cos(curr_lat_rad);

		/* Convert target lat/long to radians */
		double lat_rad = target_lat * M_DEG_TO_RAD;
		double lon_rad = target_long * M_DEG_TO_RAD;

		double sin_lat = sin(lat_rad);
		double cos_lat = cos(lat_rad);

		/* cos the difference in long */
		double cos_d_lon = cos(lon_rad - curr_lon_rad);

		double arg = curr_sin_lat * sin_lat + curr_cos_lat * cos_lat * cos_d_lon;
		if (arg > 1.0)
			arg = 1.0;
		else if (arg < -1.0)
			arg = -1.0;

		double c = acos(arg);

		double k=0.0;
		if(abs(c) < DBL_EPSILON)
			k=1.0;
		else
			k=c/sin(c);


		dx = k * (curr_cos_lat * sin_lat - curr_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
		dy = k * cos_lat * sin(lon_rad - curr_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
	 }



	/* -*- callbacks -*- */

	void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg){
		gps_msg.latitude = msg->latitude;
		gps_msg.longitude = msg->longitude;
		gps_msg.altitude = msg->altitude;
	}

	void local_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
		local_pose_msg.pose.position.x = msg->pose.position.x;
		local_pose_msg.pose.position.y = msg->pose.position.y;
		local_pose_msg.pose.position.z = msg->pose.position.z;
	}

	/**
	 * @brief Callback which sends setpoint to FCU position controller,
	  current gps is updated.
	 *
	 * @warning Send only XYZ, Yaw. ENU frame.
	 */
	void setpoint_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr &req){
		using mavlink::common::MAV_FRAME;
		float yaw; /* target yaw */

		const uint16_t ignore_all_except_xyz_y = (1 << 11) | (7 << 6) | (7 << 3);

		/* get required change in NED frame */
		lla2ned(gps_msg.latitude, gps_msg.longitude, req->latitude, req->longitude);
		Eigen::Vector3d delta_pos(dx,dy,0.0);// dx/dy in ENU
		/* to ENU */
		delta_pos = ftf::transform_frame_ned_enu(delta_pos);

		/* initialize target local position.  in ENU*/
		Eigen::Vector3d target_pos(local_pose_msg.pose.position.x,
									local_pose_msg.pose.position.y,
									local_pose_msg.pose.position.z);
		/* Calculate target position in local ENU, by adding the deltas */
		target_pos = target_pos + delta_pos;

		/* convert to NED to prepare for sending */
		target_pos = ftf::transform_frame_enu_ned(target_pos);
		yaw = ftf::transform_frame_yaw_enu_ned(req->yaw);

		/* Only send if current gps is updated, to avoid divergence */
		if ( (req->header.stamp.toNSec() / 1000000) > old_gps_stamp){
			old_gps_stamp = req->header.stamp.toNSec() / 1000000;

			set_position_target_local_ned(req->header.stamp.toNSec() / 1000000,
						utils::enum_value(MAV_FRAME::LOCAL_NED),
						ignore_all_except_xyz_y,
						target_pos,
						Eigen::Vector3d::Zero(),
						Eigen::Vector3d::Zero(),
						yaw, 0.0);
		}

	}/* end of setpoint_cb */

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointGlobalPositionPlugin, mavros::plugin::PluginBase)
