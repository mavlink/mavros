/**
 * @brief GPS status plugin
 * @file gps_status.cpp
 * @author Amilcar Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Ardupilot.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/GPSRTK.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::RTK_BASELINE_COORDINATE_SYSTEM;

/**
 * @brief Mavlink GPS status plugin.
 *
 * This plugin publishes GPS sensor data from a Mavlink compatible FCU to ROS.
 */
class GpsStatusPlugin : public plugin::PluginBase {
public:
	GpsStatusPlugin() : PluginBase(),
		gpsstatus_nh("~gpsstatus")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		gps1_raw_pub = gpsstatus_nh.advertise<mavros_msgs::GPSRAW>("gps1/raw", 10);
		gps2_raw_pub = gpsstatus_nh.advertise<mavros_msgs::GPSRAW>("gps2/raw", 10);
		gps1_rtk_pub = gpsstatus_nh.advertise<mavros_msgs::GPSRTK>("gps1/rtk", 10);
		gps2_rtk_pub = gpsstatus_nh.advertise<mavros_msgs::GPSRTK>("gps2/rtk", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&GpsStatusPlugin::handle_gps_raw_int),
			       make_handler(&GpsStatusPlugin::handle_gps2_raw),
			       make_handler(&GpsStatusPlugin::handle_gps_rtk),
			       make_handler(&GpsStatusPlugin::handle_gps2_rtk)
		};
	}

private:
	ros::NodeHandle gpsstatus_nh;

	ros::Publisher gps1_raw_pub;
	ros::Publisher gps2_raw_pub;
	ros::Publisher gps1_rtk_pub;
	ros::Publisher gps2_rtk_pub;

	/* -*- callbacks -*- */
	/**
	 * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS_RAW_INT">mavlink GPS_RAW_INT message</a> into the gps1/raw topic.
	 */
	void handle_gps_raw_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RAW_INT &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::GPSRAW>();
		ros_msg->header            = m_uas->synchronized_header("/wgs84", mav_msg.time_usec);
		ros_msg->fix_type          = mav_msg.fix_type;
		ros_msg->lat               = mav_msg.lat;
		ros_msg->lon               = mav_msg.lon;
		ros_msg->alt               = mav_msg.alt;
		ros_msg->eph               = mav_msg.eph;
		ros_msg->epv               = mav_msg.epv;
		ros_msg->vel               = mav_msg.vel;
		ros_msg->cog               = mav_msg.cog;
		ros_msg->satellites_visible = mav_msg.satellites_visible;
		ros_msg->alt_ellipsoid     = mav_msg.alt_ellipsoid;
		ros_msg->h_acc             = mav_msg.h_acc;
		ros_msg->v_acc             = mav_msg.v_acc;
		ros_msg->vel_acc           = mav_msg.vel_acc;
		ros_msg->hdg_acc           = mav_msg.hdg_acc;
		ros_msg->dgps_numch        = UINT8_MAX;	// information not available in GPS_RAW_INT mavlink message
		ros_msg->dgps_age          = UINT32_MAX;// information not available in GPS_RAW_INT mavlink message

		gps1_raw_pub.publish(ros_msg);
	}

	/**
	 * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS2_RAW">mavlink GPS2_RAW message</a> into the gps2/raw topic.
	 */
	void handle_gps2_raw(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS2_RAW &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::GPSRAW>();
		ros_msg->header            = m_uas->synchronized_header("/wgs84", mav_msg.time_usec);
		ros_msg->fix_type          = mav_msg.fix_type;
		ros_msg->lat               = mav_msg.lat;
		ros_msg->lon               = mav_msg.lon;
		ros_msg->alt               = mav_msg.alt;
		ros_msg->eph               = mav_msg.eph;
		ros_msg->epv               = mav_msg.epv;
		ros_msg->vel               = mav_msg.vel;
		ros_msg->cog               = mav_msg.cog;
		ros_msg->satellites_visible = mav_msg.satellites_visible;
		ros_msg->alt_ellipsoid     = INT32_MAX;	// information not available in GPS2_RAW mavlink message
		ros_msg->h_acc             = UINT32_MAX;// information not available in GPS2_RAW mavlink message
		ros_msg->v_acc             = UINT32_MAX;// information not available in GPS2_RAW mavlink message
		ros_msg->vel_acc           = UINT32_MAX;// information not available in GPS2_RAW mavlink message
		ros_msg->hdg_acc           = UINT32_MAX;// information not available in GPS2_RAW mavlink message
		ros_msg->dgps_numch        = mav_msg.dgps_numch;
		ros_msg->dgps_age          = mav_msg.dgps_age;

		gps2_raw_pub.publish(ros_msg);
	}

	/**
	 * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS_RTK">mavlink GPS_RTK message</a> into the gps1/rtk topic.
	 */
	void handle_gps_rtk(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RTK &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::GPSRTK>();
		switch (static_cast<RTK_BASELINE_COORDINATE_SYSTEM>(mav_msg.baseline_coords_type))
		{
		case RTK_BASELINE_COORDINATE_SYSTEM::ECEF:
			ros_msg->header.frame_id = "earth";
			break;
		case RTK_BASELINE_COORDINATE_SYSTEM::NED:
			ros_msg->header.frame_id = "map";
			break;
		default:
			ROS_ERROR_NAMED("gps_status", "GPS_RTK.baseline_coords_type MAVLink field has unknown \"%d\" value", mav_msg.baseline_coords_type);
		}
		ros_msg->header              = m_uas->synchronized_header(ros_msg->header.frame_id, mav_msg.time_last_baseline_ms * 1000);
		ros_msg->rtk_receiver_id     = mav_msg.rtk_receiver_id;
		ros_msg->wn                  = mav_msg.wn;
		ros_msg->tow                 = mav_msg.tow;
		ros_msg->rtk_health          = mav_msg.rtk_health;
		ros_msg->rtk_rate            = mav_msg.rtk_rate;
		ros_msg->nsats               = mav_msg.nsats;
		ros_msg->baseline_a          = mav_msg.baseline_a_mm;
		ros_msg->baseline_b          = mav_msg.baseline_b_mm;
		ros_msg->baseline_c          = mav_msg.baseline_c_mm;
		ros_msg->accuracy            = mav_msg.accuracy;
		ros_msg->iar_num_hypotheses  = mav_msg.iar_num_hypotheses;

		gps1_rtk_pub.publish(ros_msg);
	}

	/**
	 * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS2_RTK">mavlink GPS2_RTK message</a> into the gps2/rtk topic.
	 */
	void handle_gps2_rtk(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS2_RTK &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::GPSRTK>();
		switch (static_cast<RTK_BASELINE_COORDINATE_SYSTEM>(mav_msg.baseline_coords_type))
		{
		case RTK_BASELINE_COORDINATE_SYSTEM::ECEF:
			ros_msg->header.frame_id = "earth";
			break;
		case RTK_BASELINE_COORDINATE_SYSTEM::NED:
			ros_msg->header.frame_id = "map";
			break;
		default:
			ROS_ERROR_NAMED("gps_status", "GPS_RTK2.baseline_coords_type MAVLink field has unknown \"%d\" value", mav_msg.baseline_coords_type);
		}
		ros_msg->header              = m_uas->synchronized_header(ros_msg->header.frame_id, mav_msg.time_last_baseline_ms * 1000);
		ros_msg->rtk_receiver_id     = mav_msg.rtk_receiver_id;
		ros_msg->wn                  = mav_msg.wn;
		ros_msg->tow                 = mav_msg.tow;
		ros_msg->rtk_health          = mav_msg.rtk_health;
		ros_msg->rtk_rate            = mav_msg.rtk_rate;
		ros_msg->nsats               = mav_msg.nsats;
		ros_msg->baseline_a          = mav_msg.baseline_a_mm;
		ros_msg->baseline_b          = mav_msg.baseline_b_mm;
		ros_msg->baseline_c          = mav_msg.baseline_c_mm;
		ros_msg->accuracy            = mav_msg.accuracy;
		ros_msg->iar_num_hypotheses  = mav_msg.iar_num_hypotheses;

		gps2_rtk_pub.publish(ros_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GpsStatusPlugin, mavros::plugin::PluginBase)
