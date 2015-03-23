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
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/gps_conversions.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

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
		send_tf(false),
		rot_cov(99999.0)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		gp_nh.param("send_tf", send_tf, true);
		gp_nh.param<std::string>("frame_id", frame_id, "local_origin");
		gp_nh.param<std::string>("child_frame_id", child_frame_id, "fcu");
		gp_nh.param<double>("rot_covariance", rot_cov, 99999.0);

		fix_pub = gp_nh.advertise<sensor_msgs::NavSatFix>("global", 10);
		pos_pub = gp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("local", 10);
		vel_pub = gp_nh.advertise<geometry_msgs::Vector3Stamped>("gp_vel", 10);
		rel_alt_pub = gp_nh.advertise<std_msgs::Float64>("rel_alt", 10);
		hdg_pub = gp_nh.advertise<std_msgs::Float64>("compass_hdg", 10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &GlobalPositionPlugin::handle_global_position_int)
		};
	}

private:
	ros::NodeHandle gp_nh;
	UAS *uas;

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

	/**
	 * @todo Handler for GLOBAL_POSITION_INT_COV
	 */

	void handle_global_position_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_global_position_int_t gp_pos;
		mavlink_msg_global_position_int_decode(msg, &gp_pos);

		ROS_DEBUG_THROTTLE_NAMED(10, "global_position", "Global position: boot_ms:%06d "
				"lat/lon/alt:(%d %d %d) relative_alt: (%d) gp_vel:(%d %d %d) compass_heading: (%d)",
				gp_pos.time_boot_ms,
				gp_pos.lat, gp_pos.lon, gp_pos.alt, gp_pos.relative_alt,
				gp_pos.vx, gp_pos.vy, gp_pos.vz, gp_pos.hdg);

		auto pose_cov = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
		auto gp_fix = boost::make_shared<sensor_msgs::NavSatFix>();
		auto gp_vel = boost::make_shared<geometry_msgs::Vector3Stamped>();
		auto relative_alt = boost::make_shared<std_msgs::Float64>();
		auto compass_heading = boost::make_shared<std_msgs::Float64>();

		std_msgs::Header header;
		header.frame_id = frame_id;
		header.stamp = uas->synchronise_stamp(gp_pos.time_boot_ms);

		// Global position fix
		gp_fix->header = header;
		gp_fix->latitude = gp_pos.lat / 1E7;
		gp_fix->longitude = gp_pos.lon / 1E7;
		gp_fix->altitude = gp_pos.alt / 1E3;	// in meters

		// fill GPS status fields using GPS_RAW data
		auto raw_fix = uas->get_gps_fix();
		if (raw_fix) {
			gp_fix->status.service = raw_fix->status.service;
			gp_fix->status.status = raw_fix->status.status;
			gp_fix->position_covariance = raw_fix->position_covariance;
			gp_fix->position_covariance_type = raw_fix->position_covariance_type;
		}
		else {
			// no GPS_RAW_INT -> fix status unknown
			gp_fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
			gp_fix->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

			// we don't know covariance
			std::fill(gp_fix->position_covariance.begin(),
					gp_fix->position_covariance.end(), 0.0);
			gp_fix->position_covariance[0] = -1.0;
			gp_fix->position_covariance_type =
					sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
		}

		// Global position velocity
		gp_vel->header = header;
		gp_vel->vector.x = gp_pos.vx / 1E2;	// in m/s
		gp_vel->vector.y = gp_pos.vy / 1E2;
		gp_vel->vector.z = gp_pos.vz / 1E2;

		relative_alt->data = gp_pos.relative_alt / 1E3;	// in meters
		compass_heading->data = (gp_pos.hdg != UINT16_MAX) ? gp_pos.hdg / 1E2 : NAN;	// in degrees

		double northing, easting;
		std::string zone;

		/** Adapted from gps_umd ROS package @a http://wiki.ros.org/gps_umd
		 *  Author: Ken Tossell <ken AT tossell DOT net>
		 */
		UTM::LLtoUTM(gp_fix->latitude, gp_fix->longitude, northing, easting, zone);

		pose_cov->header = header;
		pose_cov->pose.pose.position.x = easting;
		pose_cov->pose.pose.position.y = northing;
		pose_cov->pose.pose.position.z = relative_alt->data;

		// XXX Check #193
		tf::quaternionTFToMsg(uas->get_attitude_orientation(), pose_cov->pose.pose.orientation);

		// Use ENU covariance to build XYZRPY covariance
		boost::array<double, 36> covariance = {
			gp_fix->position_covariance[0],
			gp_fix->position_covariance[1],
			gp_fix->position_covariance[2],
			0, 0, 0,
			gp_fix->position_covariance[3],
			gp_fix->position_covariance[4],
			gp_fix->position_covariance[5],
			0, 0, 0,
			gp_fix->position_covariance[6],
			gp_fix->position_covariance[7],
			gp_fix->position_covariance[8],
			0, 0, 0,
			0, 0, 0, rot_cov, 0, 0,
			0, 0, 0, 0, rot_cov, 0,
			0, 0, 0, 0, 0, rot_cov
		};

		pose_cov->pose.covariance = covariance;

		fix_pub.publish(gp_fix);
		pos_pub.publish(pose_cov);
		vel_pub.publish(gp_vel);
		rel_alt_pub.publish(relative_alt);
		hdg_pub.publish(compass_heading);

		if (send_tf) {
			tf::Transform transform;
			// XXX: we realy need change frame?
			transform.setOrigin(tf::Vector3(pose_cov->pose.pose.position.y,
						pose_cov->pose.pose.position.x,
						-pose_cov->pose.pose.position.z));

			transform.setRotation(uas->get_attitude_orientation());

			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						transform,
						pose_cov->header.stamp,
						frame_id, child_frame_id));
		}
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::GlobalPositionPlugin, mavplugin::MavRosPlugin)
