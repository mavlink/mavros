/**
 * @brief Fake GPS with MOCAP plugin
 * @file mocap_fake_gps.cpp
 * @author Christoph Tobler <toblech@ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Christoph Tobler.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief MocapFakeGPS plugin
 *
 * Sends fake GPS from motion capture data to FCU.
 */

class MocapFakeGPSPlugin : public plugin::PluginBase
{
public:
	MocapFakeGPSPlugin() : PluginBase(),
		mp_nh("~fake_gps")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		last_pos_time = ros::Time(0.0);
		gps_period = ros::Duration(0.2);	// 5hz
		mocap_tf_d_sub = mp_nh.subscribe("fix", 1, &MocapFakeGPSPlugin::mocap_tf_d_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;

	ros::Subscriber mocap_tf_d_sub;
	ros::Subscriber mocap_tf_params_sub;

	double old_stamp;
	double old_north;
	double old_east;
	double old_down;
	ros::Time last_pos_time;
	ros::Duration gps_period;

	void mocap_tf_d_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		// Throttle incoming messages to 5hz
		if ((ros::Time::now() - last_pos_time) < gps_period) {
			return;
		}
		last_pos_time = ros::Time::now();

		const double lat_zurich = 47.3667 * M_PI / 180;	// rad
		const double lon_zurich = 8.5500 * M_PI / 180;	// rad
		const float earth_radius = 6371000;		// m

		Eigen::Quaterniond q_enu;

		tf::quaternionMsgToEigen(trans->transform.rotation, q_enu);
		auto q = ftf::transform_orientation_enu_ned(ftf::transform_orientation_baselink_aircraft(q_enu));

		auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(
						trans->transform.translation.x,
						trans->transform.translation.y,
						trans->transform.translation.z));

		double north = position.x();	//[m]
		double east = position.y();	//[m]
		double down = position.z();	//[m]
		double n_rad = north / earth_radius;
		double e_rad = east / earth_radius;
		double c = sqrt(n_rad * n_rad + e_rad * e_rad);
		double sin_c = sin(c);
		double cos_c = cos(c);
		double lat_rad;
		double lon_rad;
		if (c != 0.0) {
			lat_rad = asin(cos_c * sin(lat_zurich) + (n_rad * sin_c * cos(lat_zurich)) / c);
			lon_rad = (lon_zurich + atan2(e_rad * sin_c, c * cos(lat_zurich) * cos_c - n_rad * sin(lat_zurich) * sin_c));
		}
		else {
			lat_rad = lat_zurich;
			lon_rad = lon_zurich;
		}

		double dn = north - old_north;	//[m]
		double de = east - old_east;	//[m]
		double dd = down - old_down;	//[m]
		double dt = trans->header.stamp.toSec() - old_stamp;	//[s]

		//store old values
		old_north = north;
		old_east = east;
		old_down = down;
		old_stamp = trans->header.stamp.toSec();

		//calculate velocities
		double vn = 100 * dn / dt;	//[cm/s]
		double ve = 100 * de / dt;	//[cm/s]
		double vd = 100 * dd / dt;	//[cm/s]

		//calculate course over ground
		double cog_rad;
		if (vn == 0 && ve == 0) {
			cog_rad = 0;
		}
		else if (vn >= 0 && ve < 0) {
			cog_rad = M_PI * 5 / 2 - atan2(vn,ve);
		}
		else {
			cog_rad = M_PI / 2 - atan2(vn,ve);
		}
		double cog_deg = cog_rad * 180 / M_PI;

		// Fill in and send message
		mavlink::common::msg::HIL_GPS pos;
		pos.time_usec = trans->header.stamp.toNSec() / 1000;
		pos.lat = (lat_rad * 180 / M_PI) * 10000000;	// [degrees * 1E7]
		pos.lon = (lon_rad * 180 / M_PI) * 10000000;	// [degrees * 1E7]
		pos.alt = (408 - down) * 1000;			// [m * 1000] AMSL
		pos.vel = sqrt(vn * vn + ve * ve);		// [cm/s]
		pos.vn = vn;					// [cm/s]
		pos.ve = ve;					// [cm/s]
		pos.vd = vd;					// [cm/s]
		pos.cog = cog_deg * 100;			// [degrees * 100]
		pos.eph = 2;
		pos.epv = 2;
		pos.fix_type = 3;
		pos.satellites_visible = 5;
		UAS_FCU(m_uas)->send_message_ignore_drop(pos);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MocapFakeGPSPlugin, mavros::plugin::PluginBase)
