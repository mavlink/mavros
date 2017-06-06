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
 * Copyright 2014,2017 Nuno Marques.
 * Copyright 2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>
#include <mavros/gps_conversions.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPointStamped.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Global position plugin.
 *
 * Publishes global position. Convertion from GPS to UTF allows
 * publishing local position to TF and PoseWithCovarianceStamped.
 *
 */
class GlobalPositionPlugin : public plugin::PluginBase {
public:
	GlobalPositionPlugin() : PluginBase(),
		gp_nh("~global_position"),
		tf_send(false),
		rot_cov(99999.0)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// general params
		gp_nh.param<std::string>("frame_id", frame_id, "map");
		gp_nh.param("rot_covariance", rot_cov, 99999.0);
		// tf subsection
		gp_nh.param("tf/send", tf_send, true);
		gp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		gp_nh.param<std::string>("tf/global_frame_id", tf_global_frame_id, "earth"); // The global_origin should be represented as "earth" coordinate frame (ECEF) (REP 105)
		gp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");

		UAS_DIAG(m_uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

		// gps data
		raw_fix_pub = gp_nh.advertise<sensor_msgs::NavSatFix>("raw/fix", 10);
		raw_vel_pub = gp_nh.advertise<geometry_msgs::TwistStamped>("raw/gps_vel", 10);

		// fused global position
		gp_fix_pub = gp_nh.advertise<sensor_msgs::NavSatFix>("global", 10);
		gp_odom_pub = gp_nh.advertise<nav_msgs::Odometry>("local", 10);
		gp_rel_alt_pub = gp_nh.advertise<std_msgs::Float64>("rel_alt", 10);
		gp_hdg_pub = gp_nh.advertise<std_msgs::Float64>("compass_hdg", 10);

		// global origin
		gp_global_origin_pub = gp_nh.advertise<geographic_msgs::GeoPointStamped>("gp_origin", 10);
		gp_set_global_origin_sub = gp_nh.subscribe("set_gp_origin", 10, &GlobalPositionPlugin::set_gp_origin_cb, this);

		// offset from local position to the global origin ("earth")
		gp_global_offset_pub = gp_nh.advertise<geometry_msgs::PoseStamped>("gp_lp_offset", 10);

	}

	Subscriptions get_subscriptions()
	{
		return {
				make_handler(&GlobalPositionPlugin::handle_gps_raw_int),
				// GPS_STATUS: there no corresponding ROS message, and it is not supported by APM
				make_handler(&GlobalPositionPlugin::handle_global_position_int),
				make_handler(&GlobalPositionPlugin::handle_gps_global_origin),
				make_handler(&GlobalPositionPlugin::handle_lpned_system_global_offset)
		};
	}

private:
	ros::NodeHandle gp_nh;

	ros::Publisher raw_fix_pub;
	ros::Publisher raw_vel_pub;
	ros::Publisher gp_odom_pub;
	ros::Publisher gp_fix_pub;
	ros::Publisher gp_hdg_pub;
	ros::Publisher gp_rel_alt_pub;
	ros::Publisher gp_global_origin_pub;
	ros::Publisher gp_global_offset_pub;

	ros::Subscriber gp_set_global_origin_sub;

	std::string frame_id;		//!< frame for topic headers
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_global_frame_id;	//!< global origin for TF
	std::string tf_child_frame_id;	//!< frame for TF and Pose
	bool tf_send;
	double rot_cov;

	template<typename MsgT>
	inline void fill_lla(MsgT &msg, sensor_msgs::NavSatFix::Ptr fix) {
		fix->latitude = msg.lat / 1E7;		// deg
		fix->longitude = msg.lon / 1E7;		// deg
		fix->altitude = msg.alt / 1E3;		// m
	}

	inline void fill_unknown_cov(sensor_msgs::NavSatFix::Ptr fix) {
		fix->position_covariance.fill(0.0);
		fix->position_covariance[0] = -1.0;
		fix->position_covariance_type =
					sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
	}

	/* -*- message handlers -*- */

	void handle_gps_raw_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RAW_INT &raw_gps)
	{
		auto fix = boost::make_shared<sensor_msgs::NavSatFix>();

		fix->header = m_uas->synchronized_header(frame_id, raw_gps.time_usec);

		fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
		if (raw_gps.fix_type > 2)
			fix->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
		else {
			ROS_WARN_THROTTLE_NAMED(30, "global_position", "GP: No GPS fix");
			fix->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
		}

		fill_lla(raw_gps, fix);

		float eph = (raw_gps.eph != UINT16_MAX) ? raw_gps.eph / 1E2F : NAN;
		float epv = (raw_gps.epv != UINT16_MAX) ? raw_gps.epv / 1E2F : NAN;

		if (!std::isnan(eph)) {
			const double hdop = eph;

			// From nmea_navsat_driver
			fix->position_covariance[0 + 0] = \
						fix->position_covariance[3 + 1] = std::pow(hdop, 2);
			fix->position_covariance[6 + 2] = std::pow(2 * hdop, 2);
			fix->position_covariance_type =
						sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		}
		else {
			fill_unknown_cov(fix);
		}

		// store & publish
		m_uas->update_gps_fix_epts(fix, eph, epv, raw_gps.fix_type, raw_gps.satellites_visible);
		raw_fix_pub.publish(fix);

		if (raw_gps.vel != UINT16_MAX &&
					raw_gps.cog != UINT16_MAX) {
			double speed = raw_gps.vel / 1E2;				// m/s
			double course = angles::from_degrees(raw_gps.cog / 1E2);	// rad

			auto vel = boost::make_shared<geometry_msgs::TwistStamped>();

			vel->header.frame_id = frame_id;
			vel->header.stamp = fix->header.stamp;

			// From nmea_navsat_driver
			vel->twist.linear.x = speed * std::sin(course);
			vel->twist.linear.y = speed * std::cos(course);

			raw_vel_pub.publish(vel);
		}
	}

	void handle_gps_global_origin(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_GLOBAL_ORIGIN &glob_orig)
	{
		auto g_origin = boost::make_shared<geographic_msgs::GeoPointStamped>();
		// auto header = m_uas->synchronized_header(frame_id, glob_orig.time_boot_ms);	#TODO: requires Mavlink msg update

		g_origin->header.frame_id = tf_global_frame_id;
		g_origin->header.stamp = ros::Time::now();

		// @todo: so to respect REP 105, we should convert from AMSL to ECEF using GeographicLib::GeoCoords (pending #693)
		// see <http://www.ros.org/reps/rep-0105.html>
		g_origin->position.latitude = glob_orig.latitude / 1E7;		// deg
		g_origin->position.longitude = glob_orig.longitude / 1E7;	// deg
		g_origin->position.altitude = glob_orig.altitude / 1E3;		// m

		gp_global_origin_pub.publish(g_origin);
	}

	/** @todo Handler for GLOBAL_POSITION_INT_COV */

	void handle_global_position_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GLOBAL_POSITION_INT &gpos)
	{
		auto odom = boost::make_shared<nav_msgs::Odometry>();
		auto fix = boost::make_shared<sensor_msgs::NavSatFix>();
		auto relative_alt = boost::make_shared<std_msgs::Float64>();
		auto compass_heading = boost::make_shared<std_msgs::Float64>();

		auto header = m_uas->synchronized_header(frame_id, gpos.time_boot_ms);

		// Global position fix
		fix->header = header;

		fill_lla(gpos, fix);

		// fill GPS status fields using GPS_RAW data
		auto raw_fix = m_uas->get_gps_fix();
		if (raw_fix) {
			fix->status.service = raw_fix->status.service;
			fix->status.status = raw_fix->status.status;
			fix->position_covariance = raw_fix->position_covariance;
			fix->position_covariance_type = raw_fix->position_covariance_type;
		}
		else {
			// no GPS_RAW_INT -> fix status unknown
			fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
			fix->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

			// we don't know covariance
			fill_unknown_cov(fix);
		}

		relative_alt->data = gpos.relative_alt / 1E3;	// in meters
		compass_heading->data = (gpos.hdg != UINT16_MAX) ? gpos.hdg / 1E2 : NAN;	// in degrees

		/* Global position odometry
		 *
		 * Assuming no transform is needed:
		 * X: latitude m
		 * Y: longitude m
		 * Z: altitude m
		 * VX: latitude vel m/s
		 * VY: longitude vel m/s
		 * VZ: altitude vel m/s
		 */
		odom->header = header;

		// Velocity
		tf::vectorEigenToMsg(
					Eigen::Vector3d(gpos.vx, gpos.vy, gpos.vz) / 1E2,
					odom->twist.twist.linear);

		// Velocity covariance unknown
		ftf::EigenMapCovariance6d vel_cov_out(odom->twist.covariance.data());
		vel_cov_out.fill(0.0);
		vel_cov_out(0) = -1.0;

		// Local position (UTM) calculation
		double northing, easting;
		std::string zone;

		/** @note Adapted from gps_umd ROS package @http://wiki.ros.org/gps_umd
		 *  Author: Ken Tossell <ken AT tossell DOT net>
		 */
		UTM::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

		odom->pose.pose.position.x = easting;
		odom->pose.pose.position.y = northing;
		odom->pose.pose.position.z = relative_alt->data;
		odom->pose.pose.orientation = m_uas->get_attitude_orientation();

		// Use ENU covariance to build XYZRPY covariance
		ftf::EigenMapConstCovariance3d gps_cov(fix->position_covariance.data());
		ftf::EigenMapCovariance6d pos_cov_out(odom->pose.covariance.data());
		pos_cov_out.setZero();
		pos_cov_out.block<3, 3>(0, 0) = gps_cov;
		pos_cov_out.block<3, 3>(3, 3).diagonal() <<
					rot_cov,
						rot_cov,
							rot_cov;

		// publish
		gp_fix_pub.publish(fix);
		gp_odom_pub.publish(odom);
		gp_rel_alt_pub.publish(relative_alt);
		gp_hdg_pub.publish(compass_heading);

		// TF
		if (tf_send) {
			geometry_msgs::TransformStamped transform;

			transform.header.stamp = odom->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;

			// setRotation()
			transform.transform.rotation = odom->pose.pose.orientation;

			// setOrigin()
			transform.transform.translation.x = odom->pose.pose.position.x;
			transform.transform.translation.y = odom->pose.pose.position.y;
			transform.transform.translation.z = odom->pose.pose.position.z;

			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}

	void handle_lpned_system_global_offset(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET &offset)
	{
		auto global_offset = boost::make_shared<geometry_msgs::PoseStamped>();
		global_offset->header = m_uas->synchronized_header(tf_global_frame_id, offset.time_boot_ms);

		auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(offset.x, offset.y, offset.z));
		auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
				ftf::transform_orientation_ned_enu(
					ftf::quaternion_from_rpy(offset.roll, offset.pitch, offset.yaw)));

		tf::pointEigenToMsg(enu_position, global_offset->pose.position);
		tf::quaternionEigenToMsg(enu_baselink_orientation, global_offset->pose.orientation);

		gp_global_offset_pub.publish(global_offset);

		// TF
		if (tf_send) {
			geometry_msgs::TransformStamped transform;

			transform.header.stamp = global_offset->header.stamp;
			transform.header.frame_id = tf_global_frame_id;
			transform.child_frame_id = tf_child_frame_id;

			// setRotation()
			transform.transform.rotation = global_offset->pose.orientation;

			// setOrigin()
			transform.transform.translation.x = global_offset->pose.position.x;
			transform.transform.translation.y = global_offset->pose.position.y;
			transform.transform.translation.z = global_offset->pose.position.z;

			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}

	/* -*- diagnostics -*- */
	void gps_diag_run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		int fix_type, satellites_visible;
		float eph, epv;

		m_uas->get_gps_epts(eph, epv, fix_type, satellites_visible);

		if (satellites_visible <= 0)
			stat.summary(2, "No satellites");
		else if (fix_type < 2)
			stat.summary(1, "No fix");
		else if (fix_type == 2)
			stat.summary(0, "2D fix");
		else if (fix_type >= 3)
			stat.summary(0, "3D fix");

		stat.addf("Satellites visible", "%zd", satellites_visible);
		stat.addf("Fix type", "%d", fix_type);

		if (!std::isnan(eph))
			stat.addf("EPH (m)", "%.2f", eph);
		else
			stat.add("EPH (m)", "Unknown");

		if (!std::isnan(epv))
			stat.addf("EPV (m)", "%.2f", epv);
		else
			stat.add("EPV (m)", "Unknown");
	}

	/* -*- callbacks -*- */

	void set_gp_origin_cb(const geographic_msgs::GeoPointStamped::ConstPtr &req)
	{
		mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN gpo;

		gpo.target_system = m_uas->get_tgt_system();
		// gpo.time_boot_ms = stamp.toNSec() / 1000;	#TODO: requires Mavlink msg update

		// @todo: add convertion from ECEF to AMSL #693
		gpo.latitude = req->position.latitude * 1E7;		// deg
		gpo.longitude = req->position.longitude * 1E7;	// deg
		gpo.altitude = req->position.altitude * 1E3;		// m

		UAS_FCU(m_uas)->send_message_ignore_drop(gpo);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::GlobalPositionPlugin, mavros::plugin::PluginBase)
