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
	uint8_t sensor_id;	//!< id of sensor
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
			ROS_WARN_THROTTLE_NAMED(30, "distance_sensor",
					"DS: no mapping for sensor id: %d, type: %d, orientation: %d",
					dist_sen.id, dist_sen.type, dist_sen.orientation);
			return;
		}

		auto sensor = it->second;

		if (sensor->is_subscriber) {
			ROS_WARN_THROTTLE_NAMED(30, "distance_sensor",
					"DS: %s (id %d) is subscriber, but i got sensor data for that id form FCU",
					sensor->topic_name.c_str(), sensor->sensor_id);
			return;
		}

		if (sensor->orientation >= 0 && dist_sen.orientation != sensor->orientation) {
			ROS_WARN_THROTTLE_NAMED(10, "distance_sensor",
					"DS: %s: received sensor data has different orientation (%d) than in config (%d)!",
					sensor->topic_name.c_str(), dist_sen.orientation, sensor->orientation);
		}


		auto range = boost::make_shared<sensor_msgs::Range>();

		range->header.stamp = uas->synchronise_stamp(dist_sen.time_boot_ms);
		range->header.frame_id = sensor->frame_id;

		range->min_range = 0.0;	// XXX TODO
		range->max_range = 0.0;
		range->field_of_view = 0.0;

		if (dist_sen.type == MAV_DISTANCE_SENSOR_LASER)
			range->radiation_type = sensor_msgs::Range::INFRARED;
		else if (dist_sen.type == MAV_DISTANCE_SENSOR_ULTRASOUND)
			range->radiation_type = sensor_msgs::Range::ULTRASOUND;
		else
			range->radiation_type = 0;	// XXX !!!!

		sensor->pub.publish(range);
	}
};

void DistanceSensorItem::range_cb(const sensor_msgs::Range::ConstPtr &msg)
{
	uint8_t type = 0;

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
			covariance);
}

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
		p.reset(); return p;	// return nullptr cause a bug with gcc 4.6
	}
	p->sensor_id = id;

	if (!p->is_subscriber) {
		// publisher params
		// frame_id is required
		if (!pnh.getParam("frame_id", p->frame_id)) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: `frame_id` not set!", topic_name.c_str());
			p.reset(); return p;	// nullptr
		}

		// orientation check
		pnh.param("orientation", p->orientation, -1);
	}
	else {
		// subscriber params
		// orientation is required
		if (!pnh.getParam("orientation", p->orientation)) {
			ROS_ERROR_NAMED("distance_sensor", "DS: %s: `orientation` not set!", topic_name.c_str());
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
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::DistanceSensorPlugin, mavplugin::MavRosPlugin)
