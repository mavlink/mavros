/**
 * @brief GPS publish plugin
 * @file gps.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>

namespace mavplugin {
/**
 * @brief GPS plugin
 *
 * This plugin implements same ROS topics as nmea_navsat_driver package.
 */
class GPSPlugin : public MavRosPlugin {
public:
	GPSPlugin() :
		gps_nh("~gps"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		gps_nh.param<std::string>("frame_id", frame_id, "gps");

		UAS_DIAG(uas).add("GPS", this, &GPSPlugin::diag_run);

		fix_pub = gps_nh.advertise<sensor_msgs::NavSatFix>("fix", 10);
		vel_pub = gps_nh.advertise<geometry_msgs::TwistStamped>("gps_vel", 10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_GPS_RAW_INT, &GPSPlugin::handle_gps_raw_int),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_GPS_STATUS, &GPSPlugin::handle_gps_status),
		};
	}

private:
	ros::NodeHandle gps_nh;
	UAS *uas;
	std::string frame_id;

	ros::Publisher fix_pub;
	ros::Publisher vel_pub;


	/* -*- message handlers -*- */

	void handle_gps_raw_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_gps_raw_int_t raw_gps;
		mavlink_msg_gps_raw_int_decode(msg, &raw_gps);

		auto fix = boost::make_shared<sensor_msgs::NavSatFix>();

		fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
		if (raw_gps.fix_type > 2)
			fix->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
		else {
			ROS_WARN_THROTTLE_NAMED(60, "gps", "GPS: no fix");
			fix->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
		}

		fix->latitude = raw_gps.lat / 1E7;	// deg
		fix->longitude = raw_gps.lon / 1E7;	// deg
		fix->altitude = raw_gps.alt / 1E3;	// m

		float eph = (raw_gps.eph != UINT16_MAX) ? raw_gps.eph / 1E2F : NAN;
		float epv = (raw_gps.epv != UINT16_MAX) ? raw_gps.epv / 1E2F : NAN;

		if (!isnan(eph)) {
			const double hdop = eph;
			double hdop2 = std::pow(hdop, 2);

			// TODO: Check
			// From nmea_navsat_driver
			fix->position_covariance[0] = hdop2;
			fix->position_covariance[4] = hdop2;
			fix->position_covariance[8] = std::pow(2 * hdop, 2);
			fix->position_covariance_type =
					sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		}
		else {
			fix->position_covariance_type =
					sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
		}

		fix->header.frame_id = frame_id;
		fix->header.stamp = uas->synchronise_stamp(raw_gps.time_usec);

		// store & publish
		uas->update_gps_fix_epts(fix, eph, epv, raw_gps.fix_type, raw_gps.satellites_visible);
		fix_pub.publish(fix);

		if (raw_gps.vel != UINT16_MAX &&
				raw_gps.cog != UINT16_MAX) {
			double speed = raw_gps.vel / 1E2;	// m/s
			double course = angles::from_degrees(raw_gps.cog / 1E2);// rad

			auto vel = boost::make_shared<geometry_msgs::TwistStamped>();

			// From nmea_navsat_driver
			vel->twist.linear.x = speed * std::sin(course);
			vel->twist.linear.y = speed * std::cos(course);

			vel->header.frame_id = frame_id;
			vel->header.stamp = fix->header.stamp;

			vel_pub.publish(vel);
		}
	}

	void handle_gps_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		// TODO: not supported by APM:Plane,
		//       no standard ROS messages
		mavlink_gps_status_t gps_stat;
		mavlink_msg_gps_status_decode(msg, &gps_stat);

		ROS_INFO_THROTTLE_NAMED(30, "gps", "GPS stat sat visible: %d", gps_stat.satellites_visible);
	}


	/* -*- diagnostics -*- */
	void diag_run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
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

		// EPH in centimeters
		if (!isnan(eph))
			stat.addf("EPH (m)", "%.2f", eph);
		else
			stat.add("EPH (m)", "Unknown");

		// EPV in centimeters
		if (!isnan(epv))
			stat.addf("EPV (m)", "%.2f", epv);
		else
			stat.add("EPV (m)", "Unknown");
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::GPSPlugin, mavplugin::MavRosPlugin)

