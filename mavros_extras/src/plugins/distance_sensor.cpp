/**
 * @brief Distance Sensor plugin
 * @file distance_sensor.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_extras/DistanceSensor.h>
#include <sensor_msgs/Range.h>

namespace mavplugin {
/**
 * @brief Distance sensor plugin
 *
 * This plugin allows publishing distance sensor data, which is connected to
 * an offboard/companion computer through USB/Serial, to the FCU or vice-versa.
 */
class DistanceSensorPlugin : public MavRosPlugin {
public:
	DistanceSensorPlugin() :
		dist_nh("~distance_sensor"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		dist_nh.param<std::string>("frame_id", frame_id, "distance_sensor");

		dist_nh.param("input/rangefinder_type", rangefinder_type, 0);	// default = laser (0)

		//Default sonar rangefinder is Maxbotix HRLV-EZ4
		dist_nh.param("input/sonar_fov", sonar_fov, 0.0);	// TODO
		dist_nh.param("input/sonar_min_range", sonar_min_range, 30.0); // in centimeters
		dist_nh.param("input/sonar_max_range", sonar_max_range, 500.0); 

		dist_nh.param("sonar_covariance", sonar_covariance, 0);
		dist_nh.param("output/sonar_orientation", sonar_orientation, 0); // TODO: Check!
		dist_nh.param("output/sonar_id", sonar_id, 1);

		//Default laser rangefinder is PulsedLight LIDAR Lite (fixed orientation)
		dist_nh.param("input/laser_fov", laser_fov, 0.0);	// TODO
		dist_nh.param("input/laser_min_range", laser_min_range, 1.0); // in centimeters
		dist_nh.param("input/laser_max_range", laser_max_range, 4000.0);

		dist_nh.param("laser_covariance", laser_covariance, 0);
		dist_nh.param("output/laser_orientation", sonar_orientation, 0); // TODO: Check!
		dist_nh.param("output/laser_id", laser_id, 0);

		// TODO:
		//		- variable topic advertising depending on the rangefinder type;
		//		this means incrementing number of the publishing topics and change their names according to
		//		the the type of sensor and its ID. p.e.: topics "laser_distance_1", "laser_distance_2"...

		if (rangefinder_type == 0)
			dist_laser_in_pub = dist_nh.advertise<mavros_extras::DistanceSensor>("laser_distance", 10);
		else if (rangefinder_type == 1)
			dist_sonar_in_pub = dist_nh.advertise<mavros_extras::DistanceSensor>("sonar_distance", 10);
		else ROS_ERROR_NAMED("rangefinder_type", "Invalid rangefinder type! Valid values are 0 (laser) and 1 (ultrasound)!");

		dist_sensor_sub = dist_nh.subscribe("range_data", 10, &DistanceSensorPlugin::dist_sensor_cb, this);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_DISTANCE_SENSOR, &DistanceSensorPlugin::handle_distance_sensor)
		};
	}

private:
	ros::NodeHandle dist_nh;
	UAS *uas;

	/* -*- variables -*- */
	std::string frame_id;

	double sonar_fov;
	double laser_fov;
	double sonar_min_range;
	double laser_min_range;
	double sonar_max_range;
	double laser_max_range;

	int rangefinder_type;
	int sonar_id;
	int laser_id;
	int sonar_orientation;
	int laser_orientation;
	int sonar_covariance;
	int laser_covariance;

	/* -*- publishers -*- */
	ros::Publisher dist_laser_in_pub;
	ros::Publisher dist_sonar_in_pub;

	/* -*- subscribers -*- */
	ros::Subscriber dist_sensor_sub;

	/* -*- low-level send -*- */
	void distance_sensor(uint32_t time_boot_ms,
			uint32_t min_distance,
			uint32_t max_distance,
			uint32_t current_distance,
			uint8_t type, uint8_t id,
			uint8_t orientation, uint8_t covariance) {
		mavlink_message_t msg;
		mavlink_msg_distance_sensor_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms,
				min_distance,
				max_distance,
				current_distance,
				type,
				id,
				orientation,
				covariance);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * Receive distance sensor data from FCU.
	 */
	void handle_distance_sensor(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_distance_sensor_t dist_sen;
		mavlink_msg_distance_sensor_decode(msg, &dist_sen);

		std_msgs::Header header;
		header.stamp = uas->synchronise_stamp(dist_sen.time_boot_ms);
		header.frame_id = frame_id;

		sensor_msgs::Range range;

		if (dist_sen.type == 0) {	// Laser type
			auto dist_laser_msg = boost::make_shared<mavros_extras::DistanceSensor>();

			dist_laser_msg->header = header;

			dist_laser_msg->range.header = header;
			dist_laser_msg->range.radiation_type = sensor_msgs::Range::INFRARED; // FIXME: request LASER type for Range.msg
			dist_laser_msg->range.field_of_view = laser_fov;
			dist_laser_msg->range.min_range = laser_min_range;
			dist_laser_msg->range.max_range = laser_max_range;
			dist_laser_msg->range.range = dist_sen.current_distance;

			dist_laser_msg->type = dist_sen.type;
			dist_laser_msg->id = dist_sen.id;
			dist_laser_msg->orientation = dist_sen.orientation;

			if (dist_laser_msg->covariance != 0.0) dist_laser_msg->covariance = dist_sen.covariance;
			else dist_laser_msg->covariance = laser_covariance;

			dist_laser_in_pub.publish(dist_laser_msg);
		}

		else if (dist_sen.type == 1) {	// Sonar type
			auto dist_sonar_msg = boost::make_shared<mavros_extras::DistanceSensor>();

			dist_sonar_msg->header = header;

			dist_sonar_msg->range.header = header;
			dist_sonar_msg->range.radiation_type = sensor_msgs::Range::ULTRASOUND;
			dist_sonar_msg->range.field_of_view = sonar_fov;
			dist_sonar_msg->range.min_range = sonar_min_range;
			dist_sonar_msg->range.max_range = sonar_max_range;
			dist_sonar_msg->range.range = dist_sen.current_distance;

			dist_sonar_msg->type = dist_sen.type;
			dist_sonar_msg->id = dist_sen.id;
			dist_sonar_msg->orientation = dist_sen.orientation;

			if (dist_sonar_msg->covariance != 0.0) dist_sonar_msg->covariance = dist_sen.covariance;
			else dist_sonar_msg->covariance = sonar_covariance;

			dist_sonar_in_pub.publish(dist_sonar_msg);
		}
	}

	// XXX TODO: Calculate laser/sonar covariances (both in FCU and on the ROS app side)

	// XXX TODO: Determine FOV for Sonar

	// XXX TODO: Use the sensors orientation to publish a transform between the sensor frame and FCU frame

	/**
	 * Send laser rangefinder distance sensor data to FCU.
	 */
	void send_laser_dist_sensor_data(const ros::Time &stamp, uint16_t min_distance,
			uint16_t max_distance, uint16_t current_distance) {
		distance_sensor(stamp.toNSec() / 1000000,
				min_distance,
				max_distance,
				current_distance,
				0, laser_id,
				laser_orientation,
				sonar_covariance);
	}

	/**
	 * Send laser rangefinder distance sensor data to FCU.
	 */
	void send_sonar_dist_sensor_data(const ros::Time &stamp, uint16_t min_distance,
			uint16_t max_distance, uint16_t current_distance) {
		distance_sensor(stamp.toNSec() / 1000000,
				min_distance,
				max_distance,
				current_distance,
				1, sonar_id,
				sonar_orientation,
				laser_covariance);
	}

	/* -*- callbacks -*- */
	void dist_sensor_cb(const sensor_msgs::Range::ConstPtr &req) {
		if (req->radiation_type == 0) {
			send_sonar_dist_sensor_data(req->header.stamp,
					req->min_range * 1E-2,
					req->max_range * 1E-2,
					req->range * 1E-2);
		}
		else if (req->radiation_type == 1) {
			send_laser_dist_sensor_data(req->header.stamp,
					req->min_range * 1E-2,
					req->max_range * 1E-2,
					req->range * 1E-2);
		}
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::DistanceSensorPlugin, mavplugin::MavRosPlugin)