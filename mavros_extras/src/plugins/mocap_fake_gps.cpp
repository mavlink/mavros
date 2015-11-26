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
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <vicon_bridge/TfDistortInfo.h>

namespace mavplugin {
/**
 * @brief MocapFakeGPS plugin
 *
 * Sends fake GPS from motion capture data to FCU.
 */
class MocapFakeGPSPlugin : public MavRosPlugin
{
public:
	MocapFakeGPSPlugin() :
		mp_nh("~fake_gps"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;
		mocap_tf_d_sub = mp_nh.subscribe("/tf_distort/out", 1, &MocapFakeGPSPlugin::mocap_tf_d_cb, this);
		mocap_tf_params_sub = mp_nh.subscribe("/tf_distort/info", 1, &MocapFakeGPSPlugin::mocap_tf_params_cb, this);
		mocap_tf_sub = mp_nh.subscribe("/vicon/DJI_450/DJI_450_drop", 1, &MocapFakeGPSPlugin::mocap_tf_cb, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;
	UAS *uas;

	ros::Subscriber mocap_tf_d_sub;
	ros::Subscriber mocap_tf_sub;
	ros::Subscriber mocap_tf_params_sub;

	double old_x;
	double old_y;
	double old_z;
	double old_t;
	double old_n;
	double old_e;
	double gps_eph;
	double gps_epv;

	/* -*- low-level send -*- */
	void gps_pose_send
		(uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, 
			uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satelites_visible)
	{
		//ROS_WARN("sending message");
		mavlink_message_t msg;
		mavlink_msg_hil_gps_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_usec,
				fix_type,
				lat,
				lon,
				alt,
				eph,
				epv,
				vel,
				vn,
				ve,
				vd,
				cog,
				satelites_visible);
		UAS_FCU(uas)->send_message(&msg);
	}

	void mocap_pose_send
		(uint64_t usec,
			float q[4],
			float x, float y, float z)
	{
		mavlink_message_t msg;
		mavlink_msg_att_pos_mocap_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				q,
				x,
				y,
				z);
		UAS_FCU(uas)->send_message(&msg);
	}



	/* -*- callbacks -*- */
	void mocap_tf_params_cb(const vicon_bridge::TfDistortInfo::ConstPtr &params)
	{
		gps_eph = 100 * sqrt(2 * params->sigma_xy * params->sigma_xy); //[cm]
		gps_epv = 100 * params->sigma_z; //[cm]
	}


	void mocap_tf_d_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		double lat_zurich = 47.3667 * M_PI / 180 ;  // rad
  		double lon_zurich = 8.5500 * M_PI / 180;  // rad
  		float earth_radius = 6371000;  // m

  		Eigen::Quaterniond q_enu;
		float q[4];

		tf::quaternionMsgToEigen(trans->transform.rotation, q_enu);
		UAS::quaternion_to_mavlink(
				UAS::transform_frame_enu_ned(q_enu),
				q);

		auto position = UAS::transform_frame_enu_ned(
				Eigen::Vector3d(
					trans->transform.translation.x,
					trans->transform.translation.y,
					trans->transform.translation.z));

  		double north = position.x();
  		double east = position.y();
		double n_rad = north / earth_radius;
		double e_rad = east / earth_radius;
		double z = -position.z(); //[m]
		double c = sqrt(n_rad * n_rad + e_rad * e_rad);
		double sin_c = sin(c);
		double cos_c = cos(c);
		double lat_rad;
		double lon_rad;
		if (c != 0.0) {
			lat_rad = asin(cos_c * sin(lat_zurich) + (n_rad * sin_c * cos(lat_zurich)) / c);
		    	lon_rad = (lon_zurich + atan2(e_rad * sin_c, c * cos(lat_zurich) * cos_c - n_rad * sin(lat_zurich) * sin_c));
		} else {
		 	lat_rad = lat_zurich;
		    	lon_rad = lon_zurich;
		}

		double dn = north - old_n; //[m]
		double de = east - old_e; //[m]
		double dz = z - old_z; //[m]
		double dt = trans->header.stamp.toSec() - old_t; //[s]

		//store old values
		old_n = north;
		old_e = east;
		old_z = z;
		old_t = trans->header.stamp.toSec();

		//calculate velocities
		double _vn = 100 * dn / dt; //[cm/s]
		double _ve = 100 * de / dt; //[cm/s]
		double vz = 100 * dz / dt; //[cm/s]

		//calculate course over ground
		double _cog_rad;

		if (_vn == 0 && _ve == 0) {
			_cog_rad = 0;
		}else if (_vn >= 0 && _ve < 0) {
			_cog_rad = M_PI * 5 / 2 - atan2(_vn,_ve);
		} else {
			_cog_rad = M_PI / 2 - atan2(_vn,_ve);
		}

		double _cog_deg = _cog_rad * 180 / M_PI;

		uint8_t fix_type = 3; //3D
		int32_t lat = (lat_rad * 180 / M_PI) * 10000000; // [degrees * 1E7]
		int32_t lon = (lon_rad * 180 / M_PI) * 10000000; // [degrees * 1E7]
		int32_t alt = (408 + z) * 1000; //[m * 1000] AMSL
		uint16_t eph = gps_eph; //[cm]
		uint16_t epv = gps_epv; //[cm]
		uint16_t vel = sqrt(_vn*_vn + _ve*_ve); //[cm/s]
		int16_t vn = _vn; //[cm/s]
		int16_t ve = _ve; //[cm/s]
		int16_t vd = -vz; //[cm/s]
		uint16_t cog = _cog_deg * 100; //[degrees * 100]
		uint8_t satelites_visible = 5;

		gps_pose_send(trans->header.stamp.toNSec() / 1000,
				fix_type,
				lat,
				lon,
				alt,
				eph,
				epv,
				vel,
				vn,
				ve,
				vd,
				cog,
				satelites_visible);
	}

	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		Eigen::Quaterniond q_enu;
		float q[4];
		
		tf::quaternionMsgToEigen(trans->transform.rotation, q_enu);
		UAS::quaternion_to_mavlink(
				UAS::transform_frame_enu_ned(q_enu),
				q);

		auto position = UAS::transform_frame_enu_ned(
				Eigen::Vector3d(
					trans->transform.translation.x,
					trans->transform.translation.y,
					trans->transform.translation.z));

		mocap_pose_send(trans->header.stamp.toNSec() / 1000,
				q,
				position.x(),
				position.y(),
				position.z());
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapFakeGPSPlugin, mavplugin::MavRosPlugin)
