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
#include <unordered_map>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

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
		is_subscriber(false),
		send_tf(false),
		sensor_id(0),
		field_of_view(0),
		orientation(-1),
		covariance(0),
		owner(nullptr),
		data_index(0)
	{ }

	// params
	bool is_subscriber;	//!< this item is a subscriber, else is a publisher
	bool send_tf;		//!< defines if a transform is sent or not
	uint8_t sensor_id;	//!< id of the sensor
	double field_of_view;	//!< FOV of the sensor
	Eigen::Vector3d position;	//!< sensor position
	int orientation;	//!< check orientation of sensor if != -1
	int covariance;		//!< in centimeters, current specification
	std::string frame_id;	//!< frame id for send

	// topic handle
	ros::Publisher pub;
	ros::Subscriber sub;
	std::string topic_name;

	DistanceSensorPlugin *owner;

	void range_cb(const sensor_msgs::Range::ConstPtr &msg);
	static Ptr create_item(DistanceSensorPlugin *owner, std::string topic_name);

private:
	std::vector<float> data;	//!< array allocation for measurements
	size_t data_index;		//!< array index

	/**
	 * Calculate measurements variance to send to the FCU.
	 */
	float calculate_variance(float range) {
		if (data.size() < 50)
			// limits the size of the array to 50 elements
			data.push_back(range);
		else {
			data[data_index] = range;	// it starts rewriting the values from 1st element
			if (++data_index > 49)
				data_index = 0;		// restarts the index when achieves the last element
		}

		float average, variance, sum = 0, sum_ = 0;

		/*  Compute the sum of all elements */
		for (auto d : data)
			sum += d;

		average = sum / data.size();

		/*  Compute the variance */
		for (auto d : data)
			sum_ += (d - average) * (d - average);

		variance = sum_ / data.size();

		return variance;
	}
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
		mavlink_distance_sensor_t dist_sen;
		mavlink_msg_distance_sensor_decode(msg, &dist_sen);

		auto it = sensor_map.find(dist_sen.id);
		if (it == sensor_map.end()) {
			ROS_ERROR_NAMED("distance_sensor",
					"DS: no mapping for sensor id: %d, type: %d, orientation: %d",
					dist_sen.id, dist_sen.type, dist_sen.orientation);
			return;
		}

		auto sensor = it->second;
		if (sensor->is_subscriber) {
			ROS_ERROR_NAMED("distance_sensor",
					"DS: %s (id %d) is subscriber, but i got sensor data for that id from FCU",
					sensor->topic_name.c_str(), sensor->sensor_id);
			return;
		}

		if (sensor->orientation >= 0 && dist_sen.orientation != sensor->orientation) {
			ROS_ERROR_NAMED("distance_sensor",
					"DS: %s: received sensor data has different orientation (%s) than in config (%s)!",
					sensor->topic_name.c_str(),
					UAS::str_sensor_orientation(static_cast<MAV_SENSOR_ORIENTATION>(dist_sen.orientation)).c_str(),
					UAS::str_sensor_orientation(static_cast<MAV_SENSOR_ORIENTATION>(sensor->orientation)).c_str());
		}

		auto range = boost::make_shared<sensor_msgs::Range>();

		range->header = uas->synchronized_header(sensor->frame_id, dist_sen.time_boot_ms);

		range->min_range = dist_sen.min_distance * 1E-2; // in meters
		range->max_range = dist_sen.max_distance * 1E-2;
		range->field_of_view = sensor->field_of_view;

		if (dist_sen.type == MAV_DISTANCE_SENSOR_LASER) {
			range->radiation_type = sensor_msgs::Range::INFRARED;
		}
		else if (dist_sen.type == MAV_DISTANCE_SENSOR_ULTRASOUND) {
			range->radiation_type = sensor_msgs::Range::ULTRASOUND;
		}
		else {
			ROS_ERROR_NAMED("distance_sensor",
					"DS: %s: Wrong/undefined type of sensor (type: %d). Droping!...",
					sensor->topic_name.c_str(), dist_sen.type);
			return;
		}

		range->range = dist_sen.current_distance * 1E-2;	// in meters

		if (sensor->send_tf) {
			/* variables init */
			auto q = UAS::sensor_orientation_matching(static_cast<MAV_SENSOR_ORIENTATION>(dist_sen.orientation));

			geometry_msgs::TransformStamped transform;

			transform.header = uas->synchronized_header("fcu", dist_sen.time_boot_ms);
			transform.child_frame_id = sensor->frame_id;

			/* rotation and position set */
			tf::quaternionEigenToMsg(q, transform.transform.rotation);
			tf::vectorEigenToMsg(sensor->position, transform.transform.translation);

			/* transform broadcast */
			uas->tf2_broadcaster.sendTransform(transform);
		}

		sensor->pub.publish(range);
	}
};

void DistanceSensorItem::range_cb(const sensor_msgs::Range::ConstPtr &msg)
{
	uint8_t type = 0;
	uint8_t covariance_ = 0;

	if (covariance > 0) covariance_ = covariance;
	else covariance_ = uint8_t(calculate_variance(msg->range) * 1E2);	// in cm
	/** @todo Propose changing covarince from uint8_t to float */
	ROS_DEBUG_NAMED("distance_sensor", "DS: %d: sensor variance: %f", sensor_id, calculate_variance(msg->range) * 1E2);

	// current mapping, may change later
	if (msg->radiation_type == sensor_msgs::Range::INFRARED)
		type = MAV_DISTANCE_SENSOR_LASER;
	else if (msg->radiation_type == sensor_msgs::Range::ULTRASOUND)
		type = MAV_DISTANCE_SENSOR_ULTRASOUND;

	owner->distance_sensor(
			msg->header.stamp.toNSec() / 1000000,
			msg->min_range / 1E-2,
			msg->max_range / 1E-2,
			msg->range / 1E-2,
			type,
			sensor_id,
			orientation,
			covariance_);
}

DistanceSensorItem::Ptr DistanceSensorItem::create_item(DistanceSensorPlugin *owner, std::string topic_name)
{
	auto p = boost::make_shared<DistanceSensorItem>();
	std::string orientation_str;

	ros::NodeHandle pnh(owner->dist_nh, topic_name);

	p->owner = owner;
	p->topic_name = topic_name;

	// load and parse params
	// first decide the type of topic (sub or pub)
	pnh.param("subscriber", p->is_subscriber, false);

	// sensor id
	int id;
	if (!pnh.getParam("id", id)) {
		ROS_ERROR_NAMED("distance_sensor", "DS: %s: `id` not set!", topic_name.c_str());
		p.reset(); return p;	// return nullptr because of a bug related to gcc 4.6
	}
	p->sensor_id = id;

	// orientation, checks later
	if (!pnh.getParam("orientation", orientation_str))
		p->orientation = -1;	// not set
	else
		// lookup for numeric value
		p->orientation = UAS::orientation_from_str(orientation_str);


	if (!p->is_subscriber) {
		// publisher params
		// frame_id and FOV is required
		if (!pnh.getParam("frame_id", p->frame_id)) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: `frame_id` not set!", topic_name.c_str());
			p.reset(); return p;	// nullptr
		}

		if (!pnh.getParam("field_of_view", p->field_of_view)) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: sensor FOV not set!", topic_name.c_str());
			p.reset(); return p;	// nullptr
		}

		// unset allowed, setted wrong - not
		if (p->orientation == -1 && !orientation_str.empty()) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: defined orientation (%s) is not valid!",
					topic_name.c_str(), orientation_str.c_str());
			p.reset(); return p;	// nullptr
		}

		// optional
		pnh.param("send_tf", p->send_tf, false);
		if (p->send_tf) {	// sensor position defined if 'send_tf' set to TRUE
			pnh.param("sensor_position/x", p->position.x(), 0.0);
			pnh.param("sensor_position/y", p->position.y(), 0.0);
			pnh.param("sensor_position/z", p->position.z(), 0.0);
			ROS_DEBUG_NAMED("sensor_position", "DS: %s: Sensor position at: %f, %f, %f", topic_name.c_str(),
					p->position.x(), p->position.y(), p->position.z());
		}
	}
	else {
		// subscriber params
		// orientation is required
		if (orientation_str.empty()) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: orientation not set!", topic_name.c_str());
			p.reset(); return p;	// nullptr
		}
		else if (p->orientation == -1) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: defined orientation (%s) is not valid!", topic_name.c_str(), orientation_str.c_str());
			p.reset(); return p;	// nullptr
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
}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::DistanceSensorPlugin, mavplugin::MavRosPlugin)
