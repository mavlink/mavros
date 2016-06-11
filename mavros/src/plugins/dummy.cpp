/**
 * @brief Dummy plugin
 * @file dummy.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @example dummy.cpp
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace mavros {
namespace std_plugins {

/**
 * @brief Dummy plugin.
 *
 * Example and "how to" for users.
 */
class DummyPlugin : public plugin::PluginBase {
public:
	DummyPlugin() :
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{

		ROS_INFO_NAMED("dummy", "initialize");
	}

	/**
	 * This function returns message subscriptions.
	 *
	 * Each entry defined by @a MESSAGE_HANDLER() macro.
	 * Macro create code for automatic message decoding.
	 *
	 * @a MESSAGE_HANDLER_RAW() macro doesn't do decoding.
	 */
	Subscriptions&& get_subscriptions() {
		mavlink::common::msg::HEARTBEAT hb;
		return std::move(Subscriptions{
			//MESSAGE_HANDLER(mavlink::common::msg::HEARTBEAT, handle_heartbeat),
			//MESSAGE_HANDLER(mavlink::common::msg::SYS_STATUS, handle_sys_status),
			//MESSAGE_HANDLER_RAW(mavlink::common::msg::STATUSTEXT::MSG_ID, &DummyPlugin::handle_statustext),
		});
	}

private:
	ros::NodeHandle nh;

	void handle_heartbeat(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HEARTBEAT &hb) {
		ROS_INFO_STREAM_NAMED("dummy", "Dummy::handle_heartbeat: " << hb.to_yaml());
	}

	void handle_sys_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SYS_STATUS &st) {
		ROS_INFO_STREAM_NAMED("dummy", "Dummy::handle_sys_status: " << st.to_yaml());
	}

	void handle_statustext(const mavlink::mavlink_message_t *msg, const mavconn::Framing f) {
		ROS_INFO_NAMED("dummy", "Dummy::handle_statustext(%p, %u)", msg, msg->msgid);
	}
};

}	// namespace std_plugins
}	// namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::DummyPlugin, mavros::plugin::PluginBase)

