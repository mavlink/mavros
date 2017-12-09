/**
 * @brief Fake GPS with local position source plugin
 * @file fake_gps.cpp
 * @author Christoph Tobler <toblech@ethz.ch>
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Christoph Tobler.
 * Copyright 2017 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::GPS_FIX_TYPE;

/**
 * @brief Fake GPS plugin.
 *
 * Sends fake GPS from local position estimation source data (motion capture,
 * vision) to FCU - processed in HIL mode or out of it if parameter MAV_USEHILGPS
 * is set on PX4 Pro Autopilot Firmware; Ardupilot Firmware already supports it
 * without a flag set.
 */
class FakeGPSPlugin : public plugin::PluginBase,
	private plugin::TF2ListenerMixin<FakeGPSPlugin> {
public:
	FakeGPSPlugin() : PluginBase(),
		fp_nh("~fake_gps"),
		gps_rate(5.0),
		use_mocap(true),
		map_origin(0.0, 0.0, 0.0),
		mocap_transform(true),
		use_vision(false),
		tf_listen(false),
		tf_rate(10.0),
		eph(2.0),
		epv(2.0),
		satellites_visible(5),
		fix_type(GPS_FIX_TYPE::NO_GPS),
		// WGS-84 ellipsoid (a - equatorial radius, f - flattening of ellipsoid)
		earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f())
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		double _gps_rate;
		double origin_lat, origin_lon, origin_alt;

		last_pos_time = ros::Time(0.0);

		// general params
		int ft_i;
		fp_nh.param<int>("fix_type", ft_i, utils::enum_value(GPS_FIX_TYPE::NO_GPS));
		fix_type = static_cast<GPS_FIX_TYPE>(ft_i);
		fp_nh.param("gps_rate", _gps_rate, 5.0);		// GPS data rate of 5hz
		gps_rate: _gps_rate;
		fp_nh.param("eph", eph, 2.0);
		fp_nh.param("epv", epv, 2.0);
		fp_nh.param<int>("satellites_visible", satellites_visible, 5);

		// default origin/starting point: Zürich geodetic coordinates
		fp_nh.param("geo_origin/lat", origin_lat, 47.3667);	// [degrees]
		fp_nh.param("geo_origin/lon", origin_lon, 8.5500);	// [degrees]
		fp_nh.param("geo_origin/alt", origin_alt, 408.0);	// [meters - height over the WGS-84 ellipsoid]

		// init map origin with geodetic coordinates
		map_origin = {origin_lat, origin_lon, origin_alt};

		try {
			/**
			 * @brief Conversion of the origin from geodetic coordinates (LLA)
			 * to ECEF (Earth-Centered, Earth-Fixed)
			 */
			earth.Forward(map_origin.x(), map_origin.y(), map_origin.z(),
						ecef_origin.x(), ecef_origin.y(), ecef_origin.z());
		}
		catch (const std::exception& e) {
			ROS_INFO_STREAM("FGPS: Caught exception: " << e.what() << std::endl);
		}

		// source set params
		fp_nh.param("use_mocap", use_mocap, true);		// listen to MoCap source
		fp_nh.param("mocap_transform", mocap_transform, true);	// listen to MoCap source (TransformStamped if true; PoseStamped if false)
		fp_nh.param("tf/listen", tf_listen, false);		// listen to TF source
		fp_nh.param("use_vision", use_vision, false);		// listen to Vision source

		// tf params
		fp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		fp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "fix");
		fp_nh.param("tf/rate_limit", tf_rate, 10.0);

		if (use_mocap) {
			if (mocap_transform) {	// MoCap data in TransformStamped msg
				mocap_tf_sub = fp_nh.subscribe("mocap/tf", 10, &FakeGPSPlugin::mocap_tf_cb, this);
			}
			else {	// MoCap data in PoseStamped msg
				mocap_pose_sub = fp_nh.subscribe("mocap/pose", 10, &FakeGPSPlugin::mocap_pose_cb, this);
			}
		}
		else if (use_vision) {	// Vision data in PoseStamped msg
			vision_pose_sub = fp_nh.subscribe("vision", 10, &FakeGPSPlugin::vision_cb, this);
		}
		else if (tf_listen) {	// Pose aquired from TF Listener
			ROS_INFO_STREAM_NAMED("fake_gps", "Listen to transform " << tf_frame_id
										 << " -> " << tf_child_frame_id);
			tf2_start("FakeGPSVisionTF", &FakeGPSPlugin::transform_cb);
		}
		else {
			ROS_ERROR_NAMED("fake_gps", "No pose source!");
		}
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle fp_nh;

	ros::Rate gps_rate;
	ros::Time last_pos_time;

	// Constructor for a ellipsoid
	GeographicLib::Geocentric earth;

	ros::Subscriber mocap_tf_sub;
	ros::Subscriber mocap_pose_sub;
	ros::Subscriber vision_pose_sub;

	bool use_mocap;			//!< set use of mocap data (PoseStamped msg)
	bool use_vision;		//!< set use of vision data
	bool mocap_transform;		//!< set use of mocap data (TransformStamped msg)
	bool tf_listen;			//!< set use of TF Listener data

	double eph, epv;
	int satellites_visible;
	GPS_FIX_TYPE fix_type;

	double tf_rate;
	std::string tf_frame_id;
	std::string tf_child_frame_id;
	ros::Time last_transform_stamp;

	Eigen::Vector3d map_origin;	//!< geodetic origin [lla]
	Eigen::Vector3d ecef_origin;	//!< geocentric origin [m]
	Eigen::Vector3d old_ecef;	//!< previous geocentric position [m]
	double old_stamp;		//!< previous stamp [s]

	/* -*- mid-level helpers and low-level send -*- */

	/**
	 * @brief Send fake GPS coordinates through HIL_GPS Mavlink msg
	 */
	void send_fake_gps(const ros::Time &stamp, const Eigen::Vector3d &ecef_offset) {
		// Throttle incoming messages to 5hz
		if ((ros::Time::now() - last_pos_time) < ros::Duration(gps_rate)) {
			return;
		}
		last_pos_time = ros::Time::now();

		/**
		 * @note: HIL_GPS messages are accepted on PX4 Firmware out of HIL mode,
		 * if use_hil_gps flag is set (param MAV_USEHILGPS = 1).
		 * @todo: add GPS_INPUT msg as an alternative, as Ardupilot already supports it
		 */
		mavlink::common::msg::HIL_GPS fix {};

		Eigen::Vector3d geodetic;
		Eigen::Vector3d current_ecef(ecef_origin.x() + ecef_offset.x(),
					ecef_origin.y() + ecef_offset.y(),
					ecef_origin.z() + ecef_offset.z());

		try {
			earth.Reverse(current_ecef.x(), current_ecef.y(), current_ecef.z(),
						geodetic.x(), geodetic.y(), geodetic.z());
		}
		catch (const std::exception& e) {
			ROS_INFO_STREAM("FGPS: Caught exception: " << e.what() << std::endl);
		}

		Eigen::Vector3d vel = ((old_ecef - current_ecef) / (stamp.toSec() - old_stamp)) * 1e2;// [cm/s]

		// compute course over ground
		double cog;
		if (vel.x() == 0 && vel.y() == 0) {
			cog = 0;
		}
		else if (vel.x() >= 0 && vel.y() < 0) {
			cog = M_PI * 5 / 2 - atan2(vel.x(), vel.y());
		}
		else {
			cog = M_PI / 2 - atan2(vel.x(), vel.y());
		}

		// Fill in and send message
		fix.time_usec = stamp.toNSec() / 1000;	// [useconds]
		fix.lat = geodetic.x() * 1e7;		// [degrees * 1e7]
		fix.lon = geodetic.y() * 1e7;		// [degrees * 1e7]
		fix.alt = (geodetic.z() + GeographicLib::Geoid::ELLIPSOIDTOGEOID *
			  (*m_uas->egm96_5)(geodetic.x(), geodetic.y())) * 1e3;	// [meters * 1e3]
		fix.vel = vel.block<2, 1>(0, 0).norm();	// [cm/s]
		fix.vn = vel.x();			// [cm/s]
		fix.ve = vel.y();			// [cm/s]
		fix.vd = vel.z();			// [cm/s]
		fix.cog = cog * 1e2;			// [degrees * 1e2]
		fix.eph = eph * 1e2;			// [cm]
		fix.epv = epv * 1e2;			// [cm]
		fix.fix_type = utils::enum_value(fix_type);;
		fix.satellites_visible = satellites_visible;

		// store old values
		old_stamp = stamp.toSec();
		old_ecef = current_ecef;

		UAS_FCU(m_uas)->send_message_ignore_drop(fix);
	}

	/* -*- callbacks -*- */
	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		Eigen::Affine3d pos_enu;
		tf::transformMsgToEigen(trans->transform, pos_enu);

		send_fake_gps(trans->header.stamp, ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
	}

	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &req)
	{
		Eigen::Affine3d pos_enu;
		tf::poseMsgToEigen(req->pose, pos_enu);

		send_fake_gps(req->header.stamp, ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
	}

	void vision_cb(const geometry_msgs::PoseStamped::ConstPtr &req)
	{
		Eigen::Affine3d pos_enu;
		tf::poseMsgToEigen(req->pose, pos_enu);

		send_fake_gps(req->header.stamp, ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
	}

	void transform_cb(const geometry_msgs::TransformStamped &trans)
	{
		Eigen::Affine3d pos_enu;

		tf::transformMsgToEigen(trans.transform, pos_enu);

		send_fake_gps(trans.header.stamp, ftf::transform_frame_enu_ecef(Eigen::Vector3d(pos_enu.translation()), map_origin));
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::FakeGPSPlugin, mavros::plugin::PluginBase)
