/**
 * @brief PX4Flow plugin
 * @file px4flow.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/OpticalFlowRad.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>

namespace mavplugin {
/**
 * @brief PX4 Optical Flow plugin
 *
 * This plugin can publish data from PX4Flow camera to ROS
 */
class PX4FlowPlugin : public MavRosPlugin {
public:
	PX4FlowPlugin() :
		flow_nh("~px4flow"),
		uas(nullptr),
		ranger_fov(0.0),
		ranger_min_range(0.3),
		ranger_max_range(5.0)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		flow_nh.param<std::string>("frame_id", frame_id, "px4flow");

		/** Default rangefinder is Maxbotix HRLV-EZ4 */
		flow_nh.param("ranger_fov", ranger_fov, 0.0);	/** @todo Check MAxbotix HRLV-EZ4 Field-of-View */
		flow_nh.param("ranger_min_range", ranger_min_range, 0.3);
		flow_nh.param("ranger_max_range", ranger_max_range, 5.0);

		flow_rad_pub = flow_nh.advertise<mavros_msgs::OpticalFlowRad>("raw/optical_flow_rad", 10);
		range_pub = flow_nh.advertise<sensor_msgs::Range>("ground_distance", 10);
		temp_pub = flow_nh.advertise<sensor_msgs::Temperature>("temperature", 10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, &PX4FlowPlugin::handle_optical_flow_rad)
		};
	}

private:
	ros::NodeHandle flow_nh;
	UAS *uas;

	std::string frame_id;

	double ranger_fov;
	double ranger_min_range;
	double ranger_max_range;

	ros::Publisher flow_rad_pub;
	ros::Publisher range_pub;
	ros::Publisher temp_pub;

	void handle_optical_flow_rad(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_optical_flow_rad_t flow_rad;
		mavlink_msg_optical_flow_rad_decode(msg, &flow_rad);

		auto header = uas->synchronized_header(frame_id, flow_rad.time_usec);

		/**
		 * Raw message with axes mapped to ROS conventions and temp in degrees celsius.
		 *
		 * The optical flow camera is essentially an angular sensor, so conversion is like
		 * gyroscope. (aircraft -> baselink)
		 */
		auto int_xy = UAS::transform_frame_aircraft_baselink(
				Eigen::Vector3d(
					flow_rad.integrated_x,
					flow_rad.integrated_y,
					0.0));
		auto int_gyro = UAS::transform_frame_aircraft_baselink(
				Eigen::Vector3d(
					flow_rad.integrated_xgyro,
					flow_rad.integrated_ygyro,
					flow_rad.integrated_zgyro));

		auto flow_rad_msg = boost::make_shared<mavros_msgs::OpticalFlowRad>();

		flow_rad_msg->header = header;
		flow_rad_msg->integration_time_us = flow_rad.integration_time_us;

		flow_rad_msg->integrated_x = int_xy.x();
		flow_rad_msg->integrated_y = int_xy.y();

		flow_rad_msg->integrated_xgyro = int_gyro.x();
		flow_rad_msg->integrated_ygyro = int_gyro.y();
		flow_rad_msg->integrated_zgyro = int_gyro.z();

		flow_rad_msg->temperature = flow_rad.temperature / 100.0f;	// in degrees celsius
		flow_rad_msg->time_delta_distance_us = flow_rad.time_delta_distance_us;
		flow_rad_msg->distance = flow_rad.distance;
		flow_rad_msg->quality = flow_rad.quality;

		flow_rad_pub.publish(flow_rad_msg);

		// Temperature
		auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();

		temp_msg->header = header;
		temp_msg->temperature = flow_rad_msg->temperature;

		temp_pub.publish(temp_msg);

		// Rangefinder
		/**
		 * @todo: use distance_sensor plugin only to publish this data
		 * (which receives DISTANCE_SENSOR msg with multiple rangefinder
		 * sensors data)
		 *
		 * @todo: suggest modification on MAVLink OPTICAL_FLOW_RAD msg
		 * which removes sonar data fields from it
		 */
		auto range_msg = boost::make_shared<sensor_msgs::Range>();

		range_msg->header = header;

		range_msg->radiation_type = sensor_msgs::Range::ULTRASOUND;
		range_msg->field_of_view = ranger_fov;
		range_msg->min_range = ranger_min_range;
		range_msg->max_range = ranger_max_range;
		range_msg->range = flow_rad.distance;

		range_pub.publish(range_msg);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::PX4FlowPlugin, mavplugin::MavRosPlugin)
