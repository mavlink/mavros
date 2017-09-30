/**
 * @brief Debug messages plugin
 * @file debug_value.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/DebugValue.h>

namespace mavros {
namespace extra_plugins {
//! @brief Plugin for Debug msgs from MAVLink API
class DebugValuePlugin : public plugin::PluginBase {
public:
	DebugValuePlugin() : PluginBase(),
		debug_nh("~debug_value")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// subscribers
		debug_sub = debug_nh.subscribe("send", 10, &DebugValuePlugin::debug_cb, this);

		// publishers
		debug_pub = debug_nh.advertise<mavros_msgs::DebugValue>("debug", 10);
		/** @todo create a pub config based on the naming of the debug msgs,
		 * using a map_dict as in distance_sensor plugin
		 */
		debug_vector_pub = debug_nh.advertise<mavros_msgs::DebugValue>("debug_vector", 10);
		named_value_float_pub = debug_nh.advertise<mavros_msgs::DebugValue>("named_value_float", 10);
		named_value_int_pub = debug_nh.advertise<mavros_msgs::DebugValue>("named_value_int", 10);
	}

	Subscriptions get_subscriptions() {
		return {
			       make_handler(&DebugValuePlugin::handle_debug),
			       make_handler(&DebugValuePlugin::handle_debug_vector),
			       make_handler(&DebugValuePlugin::handle_named_value_float),
			       make_handler(&DebugValuePlugin::handle_named_value_int)
		};
	}

private:
	ros::NodeHandle debug_nh;

	ros::Subscriber debug_sub;

	ros::Publisher debug_pub;
	ros::Publisher debug_vector_pub;
	ros::Publisher named_value_float_pub;
	ros::Publisher named_value_int_pub;

	/* -*- helpers -*- */

	/**
	 * @brief Helper function to log debug messages
	 * @param type	Type of debug message
	 * @param stamp	Stamp of the message
	 * @param name	Name/key of the debub value (if applicable)
	 * @param index	Index of the key value (if applicable)
	 * @param data	Data value received (float, named float, named int, 3D vector, array)
	 */
	template <typename T>
	void debug_logger(const std::string &type, const ros::Time &stamp, const std::string &name,
				const int &index, const T &value)
	{
		ROS_DEBUG_STREAM_NAMED("debug_value", type << "\t"
							   << stamp   << "\t"
							   << name    << "\t["
							   << index   << "]\tvalue:"
							   << value.data());
	}

	/* -*- message handlers -*- */

	/**
	 * @brief Handle DEBUG message.
	 * Message specification: http://mavlink.org/messages/common/#DEBUG
	 * @param msg	Received Mavlink msg
	 * @param debug	DEBUG msg
	 */
	void handle_debug(const mavlink::mavlink_message_t *msg, mavlink::common::msg::DEBUG &debug)
	{
		auto debug_msg = boost::make_shared<mavros_msgs::DebugValue>();

		ros::Time stamp = m_uas->synchronise_stamp(debug.time_boot_ms);

		std::array<float, 1> val{{debug.value}};
		debug_logger("DEBUG", stamp, "UKN", debug.ind, val);

		debug_msg->header.stamp = stamp;
		debug_msg->type = debug_msg->TYPE_DEBUG;
		debug_msg->index = debug.ind;
		debug_msg->value_float = debug.value;

		debug_pub.publish(debug_msg);
	}

	/**
	 * @brief Handle DEBUG_VECT message.
	 * Message specification: http://mavlink.org/messages/common/#DEBUG_VECT
	 * @param msg	Received Mavlink msg
	 * @param debug	DEBUG_VECT msg
	 */
	void handle_debug_vector(const mavlink::mavlink_message_t *msg, mavlink::common::msg::DEBUG_VECT &debug)
	{
		auto debug_msg = boost::make_shared<mavros_msgs::DebugValue>();

		std::string name = mavlink::to_string(debug.name);
		ros::Time stamp = m_uas->synchronise_stamp(debug.time_usec);

		std::array<float, 3> val{{debug.x, debug.y, debug.z}};
		debug_logger("DEBUG_VECT", stamp, name, -1, val);

		debug_msg->header.stamp = stamp;
		debug_msg->name = name;
		debug_msg->index = -1;
		debug_msg->type = debug_msg->TYPE_DEBUG_VECT;
		std::copy(val.begin(), val.end(), debug_msg->data.begin());

		debug_vector_pub.publish(debug_msg);
	}

	/**
	 * @todo: add handler for DEBUG_ARRAY (https://github.com/mavlink/mavlink/pull/734)
	 */

	/**
	 * @brief Handle NAMED_VALUE_FLOAT message.
	 * Message specification: http://mavlink.org/messages/common/#NAMED_VALUE_FLOAT
	 * @param msg	Received Mavlink msg
	 * @param value	NAMED_VALUE_FLOAT msg
	 */
	void handle_named_value_float(const mavlink::mavlink_message_t *msg, mavlink::common::msg::NAMED_VALUE_FLOAT &value)
	{
		auto value_msg = boost::make_shared<mavros_msgs::DebugValue>();

		ros::Time stamp = m_uas->synchronise_stamp(value.time_boot_ms);
		std::string name = mavlink::to_string(value.name);

		std::array<float, 1> val{{value.value}};
		debug_logger("NAMED_VALUE_FLOAT", stamp, name, -1, val);

		value_msg->header.stamp = stamp;
		value_msg->name = name;
		value_msg->index = -1;
		value_msg->type = value_msg->TYPE_NAMED_VALUE_FLOAT;
		value_msg->value_float = value.value;

		named_value_float_pub.publish(value_msg);
	}

	/**
	 * @brief Handle NAMED_VALUE_INT message.
	 * Message specification: http://mavlink.org/messages/common/#NAMED_VALUE_INT
	 * @param msg	Received Mavlink msg
	 * @param value	NAMED_VALUE_INT msg
	 */
	void handle_named_value_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::NAMED_VALUE_INT &value)
	{
		auto value_msg = boost::make_shared<mavros_msgs::DebugValue>();

		std::string name = mavlink::to_string(value.name);
		ros::Time stamp = m_uas->synchronise_stamp(value.time_boot_ms);

		std::array<int, 1> val{{value.value}};
		debug_logger("NAMED_VALUE_FLOAT", stamp, name, -1, val);

		value_msg->header.stamp = stamp;
		value_msg->name = name;
		value_msg->index = -1;
		value_msg->type = value_msg->TYPE_NAMED_VALUE_INT;
		value_msg->value_float = value.value;

		named_value_float_pub.publish(value_msg);
	}

	/* -*- callbacks -*- */

	/**
	 * @brief Debug callbacks
	 * @param req	pointer to mavros_msgs/Debug.msg being published
	 */
	void debug_cb(const mavros_msgs::DebugValue::ConstPtr &req)
	{
		switch (req->type) {
		case mavros_msgs::DebugValue::TYPE_DEBUG:
		{
			mavlink::common::msg::DEBUG debug {};

			debug.time_boot_ms = req->header.stamp.toNSec() / 1000000;
			debug.ind = req->index;
			debug.value = req->value_float;

			UAS_FCU(m_uas)->send_message_ignore_drop(debug);
			break;
		}
		case mavros_msgs::DebugValue::TYPE_DEBUG_VECT:
		{
			mavlink::common::msg::DEBUG_VECT debug {};

			debug.time_usec = req->header.stamp.toNSec() / 1000;
			mavlink::set_string(debug.name, req->name);
			// [[[cog:
			// for a,b in zip("xyz", range(0,3)):
			//     cog.outl("debug.%s = req->data[%s];" % (a, b))
			// ]]]
			debug.x = req->data[0];
			debug.y = req->data[1];
			debug.z = req->data[2];
			// [[[end]]] (checksum: f4918ce98ca3183f93f6aff20d4ab7ec)

			UAS_FCU(m_uas)->send_message_ignore_drop(debug);
			break;
		}
		case mavros_msgs::DebugValue::TYPE_DEBUG_ARRAY:
		{
			// @todo
			return;
		}
		case mavros_msgs::DebugValue::TYPE_NAMED_VALUE_FLOAT:
		{
			mavlink::common::msg::NAMED_VALUE_FLOAT value {};

			value.time_boot_ms = req->header.stamp.toNSec() / 1000000;
			mavlink::set_string(value.name, req->name);
			value.value = req->value_float;

			UAS_FCU(m_uas)->send_message_ignore_drop(value);
			break;
		}
		case mavros_msgs::DebugValue::TYPE_NAMED_VALUE_INT:
		{
			mavlink::common::msg::NAMED_VALUE_INT value {};

			value.time_boot_ms = req->header.stamp.toNSec() / 1000000;
			mavlink::set_string(value.name, req->name);
			value.value = req->value_int;

			UAS_FCU(m_uas)->send_message_ignore_drop(value);
			break;
		}
		default:
			ROS_ERROR_NAMED("debug", "Wrong debug type (%d). Droping!...", req->type);
			return;
		}
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::DebugValuePlugin, mavros::plugin::PluginBase)
