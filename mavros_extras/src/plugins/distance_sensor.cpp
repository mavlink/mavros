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

#include <unordered_map>

#include <sensor_msgs/Range.h>

namespace mavplugin {

class DistanceSensorPlugin;

/**
 * @brief Distance sensor mapping storage item
 */
class DistanceSensorItem {
public:
	typedef boost::shared_ptr<DistanceSensorItem> Ptr;

	DistanceSensorItem() :
		owner(nullptr),
		is_subscriber(false),
		sensor_id(0),
		orientation(-1),
		covariance(0)
	{ }

	// params
	bool is_subscriber;	//!< this item is subscriber, else publisher
	uint8_t sensor_id; 	//!< id of sensor
	int orientation;	//!< check orientation of sensor if != -1
	int covariance;		//!< in centimeters, current specification
	std::string frame_id;	//!< frame id for send

	// topic handle
	ros::Publisher pub;
	ros::Subscriber sub;
	std::string topic_name;

	DistanceSensorPlugin *owner;

	void range_cb(const sensor_msgs::Range::ConstPtr &msg) {
		ROS_WARN("CB XXXX TODO: id: %d, owner: %p", sensor_id, owner);
	}

	static Ptr create_item(DistanceSensorPlugin *owner, std::string topic_name);
};

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

		XmlRpc::XmlRpcValue map_dict;
		if (!dist_nh.getParam("", map_dict)) {
			ROS_WARN_NAMED("distance_sensor", "DS: plugin not configured!");
			return;
		}

		ROS_ASSERT(map_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);

		for (auto &pair : map_dict) {
			ROS_DEBUG_NAMED("distance_sensor", "DS: initializing mapping for %s", pair.first.c_str());
			auto it = DistanceSensorItem::create_item(this, pair.first);

			if (it)
				sensor_map[it->sensor_id] = it;
			else
				ROS_ERROR_NAMED("distance_sensor", "DS: bad config for %s", pair.first.c_str());
		}
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_DISTANCE_SENSOR, &DistanceSensorPlugin::handle_distance_sensor)
		};
	}

private:
	friend class DistanceSensorItem;

	ros::NodeHandle dist_nh;
	UAS *uas;

	std::unordered_map<uint8_t, DistanceSensorItem::Ptr> sensor_map;

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
#if 0
		mavlink_distance_sensor_t dist_sen;
		mavlink_msg_distance_sensor_decode(msg, &dist_sen);

		std_msgs::Header header;
		header.stamp = uas->synchronise_stamp(dist_sen.time_boot_ms);

		sensor_msgs::Range range;
		std::string sensor_id = std::to_string(dist_sen.id);

		auto covariance = boost::make_shared<std_msgs::Float32>();

		if (dist_sen.type == 0) {	// Laser type
			auto dist_laser_msg = boost::make_shared<sensor_msgs::Range>();

			header.frame_id = "laser" + sensor_id + "_distance";

			dist_laser_msg->header = header;
			dist_laser_msg->radiation_type = sensor_msgs::Range::INFRARED; // FIXME: request LASER type for Range.msg
			dist_laser_msg->field_of_view = laser_fov;
			dist_laser_msg->min_range = laser_min_range;
			dist_laser_msg->max_range = laser_max_range;
			dist_laser_msg->range = dist_sen.current_distance;

			//dist_laser_msg->orientation = dist_sen.orientation;

			if (dist_sen.covariance != 0.0)
				covariance->data = dist_sen.covariance;
			else
				covariance->data = laser_covariance;

			dist_laser_in_pub.publish(dist_laser_msg);
		}

		else if (dist_sen.type == 1) {	// Sonar type
			auto dist_sonar_msg = boost::make_shared<sensor_msgs::Range>();

			header.frame_id = "sonar" + sensor_id + "_distance";

			dist_sonar_msg->header = header;
			dist_sonar_msg->radiation_type = sensor_msgs::Range::ULTRASOUND;
			dist_sonar_msg->field_of_view = sonar_fov;
			dist_sonar_msg->min_range = sonar_min_range;
			dist_sonar_msg->max_range = sonar_max_range;
			dist_sonar_msg->range = dist_sen.current_distance;

			//dist_sonar_msg->orientation = dist_sen.orientation;

			if (dist_sen.covariance != 0.0)
				covariance->data = dist_sen.covariance;
			else
				covariance->data = laser_covariance;

			dist_sonar_in_pub.publish(dist_sonar_msg);
		}
		
#endif
	}

	// XXX TODO: Calculate laser/sonar covariances (both in FCU and on the ROS app side)

	// XXX TODO: Determine FOV for Sonar

	// XXX TODO: Use the sensors orientation to publish a transform between the sensor frame and FCU frame
#if 0
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
				laser_covariance);
	}

	/**
	 * Send sonar rangefinder distance sensor data to FCU.
	 */
	void send_sonar_dist_sensor_data(const ros::Time &stamp, uint16_t min_distance,
			uint16_t max_distance, uint16_t current_distance) {
		distance_sensor(stamp.toNSec() / 1000000,
				min_distance,
				max_distance,
				current_distance,
				1, sonar_id,
				sonar_orientation,
				sonar_covariance);
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
#endif
};

DistanceSensorItem::Ptr DistanceSensorItem::create_item(DistanceSensorPlugin *owner, std::string topic_name)
{
	auto p = boost::make_shared<DistanceSensorItem>();

	ros::NodeHandle pnh(owner->dist_nh, topic_name);

	p->owner = owner;
	p->topic_name = topic_name;

	// load and parse params
	// first decide pub/sub type
	pnh.param("subscriber", p->is_subscriber, false);

	// sensor id
	int id;
	if (!pnh.getParam("id", id)) {
		ROS_ERROR_NAMED("distance_sensor", "DS: %s: `id` not set!", topic_name.c_str());
		return nullptr;
	}
	p->sensor_id = id;

	if (!p->is_subscriber) {
		// publisher params
		// frame_id is required
		if (!pnh.getParam("frame_id", p->frame_id)) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: `frame_id` not set!", topic_name.c_str());
			return nullptr;
		}

		// orientation check
		pnh.param("orientation", p->orientation, -1);
	}
	else {
		// subscriber params
		// orientation is required
		if (!pnh.getParam("orientation", p->orientation)) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: `orientation` not set!", topic_name.c_str());
			return nullptr;
		}

		// optional
		pnh.param("covariance", p->covariance, 0);
	}

	// create topic handles
	if (!p->is_subscriber)
		p->pub = owner->dist_nh.advertise<sensor_msgs::Range>(topic_name, 10);
	else
		p->sub = owner->dist_nh.subscribe(topic_name, 10, &DistanceSensorItem::range_cb, p.get());

	return p;
}


};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::DistanceSensorPlugin, mavplugin::MavRosPlugin)
