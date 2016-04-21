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
 * Copyright 2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>
#include <mavros/gps_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

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
		gp_nh("~global_position"),
		uas(nullptr),
		tf_send(false),
		rot_cov(99999.0)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		gp_nh.param<std::string>("frame_id", frame_id, "map");
		gp_nh.param("rot_covariance", rot_cov, 99999.0);
		// tf subsection
		gp_nh.param("tf/send", tf_send, true);
		gp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		gp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");

		UAS_DIAG(uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

		// gps data
		raw_fix_pub = gp_nh.advertise<sensor_msgs::NavSatFix>("raw/fix", 10);
		raw_vel_pub = gp_nh.advertise<geometry_msgs::TwistStamped>("raw/gps_vel", 10);

		// fused global position
		gp_fix_pub = gp_nh.advertise<sensor_msgs::NavSatFix>("global", 10);
		gp_odom_pub = gp_nh.advertise<nav_msgs::Odometry>("local", 10);
		gp_rel_alt_pub = gp_nh.advertise<std_msgs::Float64>("rel_alt", 10);
		gp_hdg_pub = gp_nh.advertise<std_msgs::Float64>("compass_hdg", 10);
	}

	const message_map get_rx_handlers() {
		return {
				MESSAGE_HANDLER(MAVLINK_MSG_ID_GPS_RAW_INT, &GlobalPositionPlugin::handle_gps_raw_int),
				// MAVLINK_MSG_ID_GPS_STATUS: there no corresponding ROS message, and it is not supported by APM
				MESSAGE_HANDLER(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &GlobalPositionPlugin::handle_global_position_int)
		};
	}

private:
	ros::NodeHandle gp_nh;
	UAS *uas;

	ros::Publisher raw_fix_pub;
	ros::Publisher raw_vel_pub;
	ros::Publisher gp_odom_pub;
	ros::Publisher gp_fix_pub;
	ros::Publisher gp_hdg_pub;
	ros::Publisher gp_rel_alt_pub;

	std::string frame_id;		//!< frame for topic headers
	std::string tf_frame_id;	//!< origin for TF
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

	void handle_gps_raw_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_gps_raw_int_t raw_gps;
		mavlink_msg_gps_raw_int_decode(msg, &raw_gps);

		auto fix = boost::make_shared<sensor_msgs::NavSatFix>();

		fix->header = uas->synchronized_header(frame_id, raw_gps.time_usec);

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
		uas->update_gps_fix_epts(fix, eph, epv, raw_gps.fix_type, raw_gps.satellites_visible);
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

	/** @todo Handler for GLOBAL_POSITION_INT_COV */

	void handle_global_position_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_global_position_int_t gpos;
		mavlink_msg_global_position_int_decode(msg, &gpos);

		auto odom = boost::make_shared<nav_msgs::Odometry>();
		auto fix = boost::make_shared<sensor_msgs::NavSatFix>();
		auto relative_alt = boost::make_shared<std_msgs::Float64>();
		auto compass_heading = boost::make_shared<std_msgs::Float64>();

		auto header = uas->synchronized_header(frame_id, gpos.time_boot_ms);

		// Global position fix
		fix->header = header;

		fill_lla(gpos, fix);

		// fill GPS status fields using GPS_RAW data
		auto raw_fix = uas->get_gps_fix();
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
		UAS::EigenMapCovariance6d vel_cov_out(odom->twist.covariance.data());
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
		odom->pose.pose.orientation = uas->get_attitude_orientation();

		// Use ENU covariance to build XYZRPY covariance
		UAS::EigenMapConstCovariance3d gps_cov(fix->position_covariance.data());
		UAS::EigenMapCovariance6d pos_cov_out(odom->pose.covariance.data());
		pos_cov_out <<
			gps_cov(0, 0) , gps_cov(0, 1) , gps_cov(0, 2) , 0.0     , 0.0     , 0.0     ,
			gps_cov(1, 0) , gps_cov(1, 1) , gps_cov(1, 2) , 0.0     , 0.0     , 0.0     ,
			gps_cov(2, 0) , gps_cov(2, 1) , gps_cov(2, 2) , 0.0     , 0.0     , 0.0     ,
			0.0           , 0.0           , 0.0           , rot_cov , 0.0     , 0.0     ,
			0.0           , 0.0           , 0.0           , 0.0     , rot_cov , 0.0     ,
			0.0           , 0.0           , 0.0           , 0.0     , 0.0     , rot_cov ;

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

			uas->tf2_broadcaster.sendTransform(transform);
		}
	}

	/* -*- diagnostics -*- */
	void gps_diag_run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		int fix_type, satellites_visible;
		float eph, epv;

		uas->get_gps_epts(eph, epv, fix_type, satellites_visible);

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
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::GlobalPositionPlugin, mavplugin::MavRosPlugin)
