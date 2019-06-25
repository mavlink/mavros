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
	 * @param dv	Data value
	 */
	void debug_logger(const std::string &type, const mavros_msgs::DebugValue &dv)
	{
		using DV = mavros_msgs::DebugValue;

		std::string name = (dv.name == "") ? "UNK" : dv.name;

		std::ostringstream ss;
		if (dv.type == DV::TYPE_NAMED_VALUE_INT) {
			ss << dv.value_int;
		}
		else if (dv.type == DV::TYPE_DEBUG_VECT) {
			ss << "[";
			bool is_first = true;
			for (auto v : dv.data) {
				if (!is_first) {
					ss << ", ";
				}

				ss << v;
				is_first = false;
			}

			ss << "]";
		}
		else {
			ss << dv.value_float;
		}


		ROS_DEBUG_STREAM_NAMED("debug_value", type << "\t"
							   << dv.header.stamp   << "\t"
							   << name    << "\t["
							   << dv.index   << "]\tvalue:"
							   << ss.str());
	}

	/* -*- message handlers -*- */

	/**
	 * @brief Handle DEBUG message.
	 * Message specification: https://mavlink.io/en/messages/common.html#DEBUG
	 * @param msg	Received Mavlink msg
	 * @param debug	DEBUG msg
	 */
	void handle_debug(const mavlink::mavlink_message_t *msg, mavlink::common::msg::DEBUG &debug)
	{
		// [[[cog:
		// p = "dv_msg"
		// val = "debug"
		//
		// def common_filler(type_, time_f, index, name):
		//     if isinstance(index, str):
		//         index = val + "." + index
		//
		//     _args = globals()
		//     _args.update(locals())
		//
		//     cog.outl("""auto {p} = boost::make_shared<mavros_msgs::DebugValue>();""".format(**_args))
		//     cog.outl("""{p}->header.stamp = m_uas->synchronise_stamp({val}.{time_f});""".format(**_args))
		//     cog.outl("""{p}->type = mavros_msgs::DebugValue::{type_};""".format(**_args))
		//     cog.outl("""{p}->index = {index};""".format(**_args))
		//     if name:
		//         cog.outl("""{p}->name = mavlink::to_string({val}.{name});""".format(**_args))
		//
		// common_filler("TYPE_DEBUG", "time_boot_ms", "ind", None)
		// cog.outl("""{p}->value_float = {val}.value;""".format(**locals()))
		// ]]]
		auto dv_msg = boost::make_shared<mavros_msgs::DebugValue>();
		dv_msg->header.stamp = m_uas->synchronise_stamp(debug.time_boot_ms);
		dv_msg->type = mavros_msgs::DebugValue::TYPE_DEBUG;
		dv_msg->index = debug.ind;
		dv_msg->value_float = debug.value;
		// [[[end]]] (checksum: 82e058cb699846b34885ce3defc6ac58)

		debug_logger(debug.get_name(), *dv_msg);
		debug_pub.publish(dv_msg);
	}

	/**
	 * @brief Handle DEBUG_VECT message.
	 * Message specification: https://mavlink.io/en/messages/common.html#DEBUG_VECT
	 * @param msg	Received Mavlink msg
	 * @param debug	DEBUG_VECT msg
	 */
	void handle_debug_vector(const mavlink::mavlink_message_t *msg, mavlink::common::msg::DEBUG_VECT &debug)
	{
		// [[[cog:
		// common_filler("TYPE_DEBUG_VECT", "time_usec", -1, "name")
		//
		// fields = "xyz"
		// pd = p + "->data"
		// cog.outl("""{pd}.resize({l});""".format(l=len(fields), **locals()))
		// for i, f in enumerate(fields):
		//     cog.outl("""{pd}[{i}] = {val}.{f};""".format(**locals()))
		// ]]]
		auto dv_msg = boost::make_shared<mavros_msgs::DebugValue>();
		dv_msg->header.stamp = m_uas->synchronise_stamp(debug.time_usec);
		dv_msg->type = mavros_msgs::DebugValue::TYPE_DEBUG_VECT;
		dv_msg->index = -1;
		dv_msg->name = mavlink::to_string(debug.name);
		dv_msg->data.resize(3);
		dv_msg->data[0] = debug.x;
		dv_msg->data[1] = debug.y;
		dv_msg->data[2] = debug.z;
		// [[[end]]] (checksum: 966bca4d95f06349cf065f3887c69471)

		debug_logger(debug.get_name(), *dv_msg);
		debug_vector_pub.publish(dv_msg);
	}

	/**
	 * @todo: add handler for DEBUG_ARRAY (https://github.com/mavlink/mavlink/pull/734)
	 */

	/**
	 * @brief Handle NAMED_VALUE_FLOAT message.
	 * Message specification: https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT
	 * @param msg	Received Mavlink msg
	 * @param value	NAMED_VALUE_FLOAT msg
	 */
	void handle_named_value_float(const mavlink::mavlink_message_t *msg, mavlink::common::msg::NAMED_VALUE_FLOAT &value)
	{
		// [[[cog:
		// val="value"
		// common_filler("TYPE_NAMED_VALUE_FLOAT", "time_boot_ms", -1, "name")
		// cog.outl("""{p}->value_float = {val}.value;""".format(**locals()))
		// ]]]
		auto dv_msg = boost::make_shared<mavros_msgs::DebugValue>();
		dv_msg->header.stamp = m_uas->synchronise_stamp(value.time_boot_ms);
		dv_msg->type = mavros_msgs::DebugValue::TYPE_NAMED_VALUE_FLOAT;
		dv_msg->index = -1;
		dv_msg->name = mavlink::to_string(value.name);
		dv_msg->value_float = value.value;
		// [[[end]]] (checksum: 8d70c469d67ab346574c99d20b91a501)

		debug_logger(value.get_name(), *dv_msg);
		named_value_float_pub.publish(dv_msg);
	}

	/**
	 * @brief Handle NAMED_VALUE_INT message.
	 * Message specification: https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT
	 * @param msg	Received Mavlink msg
	 * @param value	NAMED_VALUE_INT msg
	 */
	void handle_named_value_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::NAMED_VALUE_INT &value)
	{
		// [[[cog:
		// common_filler("TYPE_NAMED_VALUE_INT", "time_boot_ms", -1, "name")
		// cog.outl("""{p}->value_int = {val}.value;""".format(**locals()))
		// ]]]
		auto dv_msg = boost::make_shared<mavros_msgs::DebugValue>();
		dv_msg->header.stamp = m_uas->synchronise_stamp(value.time_boot_ms);
		dv_msg->type = mavros_msgs::DebugValue::TYPE_NAMED_VALUE_INT;
		dv_msg->index = -1;
		dv_msg->name = mavlink::to_string(value.name);
		dv_msg->value_int = value.value;
		// [[[end]]] (checksum: b7ce125baba6e8918f6b866a9d5aa77c)

		debug_logger(value.get_name(), *dv_msg);
		named_value_int_pub.publish(dv_msg);
	}

	/* -*- callbacks -*- */

	/**
	 * @brief Debug callbacks
	 * @param req	pointer to mavros_msgs/Debug.msg being published
	 */
	void debug_cb(const mavros_msgs::DebugValue::ConstPtr &req)
	{
		switch (req->type) {
		case mavros_msgs::DebugValue::TYPE_DEBUG: {
			mavlink::common::msg::DEBUG debug {};

			debug.time_boot_ms = req->header.stamp.toNSec() / 1000000;
			debug.ind = req->index;
			debug.value = req->value_float;

			UAS_FCU(m_uas)->send_message_ignore_drop(debug);
			break;
		}
		case mavros_msgs::DebugValue::TYPE_DEBUG_VECT: {
			mavlink::common::msg::DEBUG_VECT debug {};

			debug.time_usec = req->header.stamp.toNSec() / 1000;
			mavlink::set_string(debug.name, req->name);
			// [[[cog:
			// for i, f in enumerate("xyz"):
			//     cog.outl("debug.{f} = req->data[{i}];".format(**locals()))
			// ]]]
			debug.x = req->data[0];
			debug.y = req->data[1];
			debug.z = req->data[2];
			// [[[end]]] (checksum: f4918ce98ca3183f93f6aff20d4ab7ec)

			UAS_FCU(m_uas)->send_message_ignore_drop(debug);
			break;
		}
		//case mavros_msgs::DebugValue::TYPE_DEBUG_ARRAY:		{
		//	return;
		//}
		case mavros_msgs::DebugValue::TYPE_NAMED_VALUE_FLOAT: {
			mavlink::common::msg::NAMED_VALUE_FLOAT value {};

			value.time_boot_ms = req->header.stamp.toNSec() / 1000000;
			mavlink::set_string(value.name, req->name);
			value.value = req->value_float;

			UAS_FCU(m_uas)->send_message_ignore_drop(value);
			break;
		}
		case mavros_msgs::DebugValue::TYPE_NAMED_VALUE_INT: {
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
