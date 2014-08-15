/**
 * @brief GPS publish plugin
 * @file gps.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
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

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>

namespace mavplugin {

class GPSInfo : public diagnostic_updater::DiagnosticTask
{
public:
	explicit GPSInfo(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		satellites_visible(-1),
		fix_type(0),
		eph(UINT16_MAX),
		epv(UINT16_MAX)
	{ };

	void set_gps_raw(mavlink_gps_raw_int_t &gps) {
		satellites_visible = gps.satellites_visible;
		fix_type = gps.fix_type;
		eph = gps.eph;
		epv = gps.epv;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		const int satellites_visible_ = satellites_visible;
		const int fix_type_ = fix_type;
		const uint16_t eph_ = eph;
		const uint16_t epv_ = epv;

		if (satellites_visible_ < 0)
			stat.summary(2, "No satellites");
		else if (fix_type_ < 2 || fix_type_ > 3)
			stat.summary(1, "No fix");
		else if (fix_type_ == 2)
			stat.summary(0, "2D fix");
		else if (fix_type_ == 3)
			stat.summary(0, "3D fix");

		stat.addf("Satellites visible", "%zd", satellites_visible_);
		stat.addf("Fix type", "%d", fix_type_);

		// EPH in centimeters
		if (eph_ != UINT16_MAX)
			stat.addf("EPH (m)", "%.2f", eph_ / 1E2F);
		else
			stat.add("EPH (m)", "Unknown");

		// EPV in centimeters
		if (epv_ != UINT16_MAX)
			stat.addf("EPV (m)", "%.2f", epv_ / 1E2F);
		else
			stat.add("EPV (m)", "Unknown");
	}

private:
	std::atomic<int> satellites_visible;
	std::atomic<int> fix_type;
	std::atomic<uint16_t> eph;
	std::atomic<uint16_t> epv;
};


/**
 * @brief GPS plugin
 *
 * This plugin implements same ROS topics as nmea_navsat_driver package.
 */
class GPSPlugin : public MavRosPlugin {
public:
	GPSPlugin() :
		uas(nullptr),
		gps_diag("GPS")
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		nh.param<std::string>("gps/frame_id", frame_id, "gps");
		nh.param<std::string>("gps/time_ref_source", time_ref_source, frame_id);

		diag_updater.add(gps_diag);

		fix_pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 10);
		time_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", 10);
		vel_pub = nh.advertise<geometry_msgs::TwistStamped>("gps_vel", 10);
	}

	std::string const get_name() const {
		return "GPS";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_GPS_RAW_INT, &GPSPlugin::handle_gps_raw_int),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_GPS_STATUS, &GPSPlugin::handle_gps_status),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_SYSTEM_TIME, &GPSPlugin::handle_system_time),
		};
	}

private:
	UAS *uas;
	std::string frame_id;
	std::string time_ref_source;

	GPSInfo gps_diag;

	ros::Publisher fix_pub;
	ros::Publisher time_ref_pub;
	ros::Publisher vel_pub;

	void handle_gps_raw_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_gps_raw_int_t raw_gps;
		mavlink_msg_gps_raw_int_decode(msg, &raw_gps);

		sensor_msgs::NavSatFixPtr fix = boost::make_shared<sensor_msgs::NavSatFix>();
		geometry_msgs::TwistStampedPtr vel = boost::make_shared<geometry_msgs::TwistStamped>();

		gps_diag.set_gps_raw(raw_gps);
		if (raw_gps.fix_type < 2) {
			ROS_WARN_THROTTLE_NAMED(60, "gps", "GPS: no fix");
			return;
		}

		fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
		if (raw_gps.fix_type == 2 || raw_gps.fix_type == 3)
			fix->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
		else
			fix->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

		fix->latitude = raw_gps.lat / 1E7; // deg
		fix->longitude = raw_gps.lon / 1E7; // deg
		fix->altitude = raw_gps.alt / 1E3; // m

		if (raw_gps.eph != UINT16_MAX) {
			double hdop = raw_gps.eph / 1E2;
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
		fix->header.stamp = ros::Time::now();

		// store GPS data in UAS
		double eph = (raw_gps.eph != UINT16_MAX)? raw_gps.eph / 1E2 : NAN;
		double epv = (raw_gps.epv != UINT16_MAX)? raw_gps.epv / 1E2 : NAN;
		uas->set_gps_llae(fix->latitude, fix->longitude, fix->altitude, eph, epv);
		uas->set_gps_status(fix->status.status == sensor_msgs::NavSatStatus::STATUS_FIX);

		fix_pub.publish(fix);

		if (raw_gps.vel != UINT16_MAX &&
				raw_gps.cog != UINT16_MAX) {
			double speed = raw_gps.vel / 1E2; // m/s
			double course = angles::from_degrees(raw_gps.cog / 1E2); // rad

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

	void handle_system_time(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_system_time_t mtime;
		mavlink_msg_system_time_decode(msg, &mtime);

		if (mtime.time_unix_usec == 0) {
			ROS_WARN_THROTTLE_NAMED(60, "gps", "Wrong system time. Is GPS Ok? (boot_ms: %u)",
					mtime.time_boot_ms);
			return;
		}

		sensor_msgs::TimeReferencePtr time = boost::make_shared<sensor_msgs::TimeReference>();
		ros::Time time_ref(
				mtime.time_unix_usec / 1000000,			// t_sec
				(mtime.time_unix_usec % 1000000) * 1000);	// t_nsec

		time->source = time_ref_source;
		time->time_ref = time_ref;
		time->header.frame_id = time_ref_source;
		time->header.stamp = ros::Time::now();

		time_ref_pub.publish(time);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::GPSPlugin, mavplugin::MavRosPlugin)

