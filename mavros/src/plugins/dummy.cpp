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

namespace mavros {
namespace std_plugins {

/**
 * @brief Dummy plugin.
 *
 * Example and "how to" for users.
 */
class DummyPlugin : public plugin::PluginBase {
public:
	DummyPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		ROS_INFO_NAMED("dummy", "Dummy::initialize");
	}

	/**
	 * This function returns message subscriptions.
	 *
	 * Each subscription made by PluginBase::make_handler() template.
	 * Two variations:
	 *  - With automatic decoding and framing error filtering (see handle_heartbeat)
	 *  - Raw message with framig status (see handle_systemtext)
	 */
	Subscriptions get_subscriptions() override {
		return {
			/* automatic message deduction by second argument */
			make_handler(&DummyPlugin::handle_heartbeat),
			make_handler(&DummyPlugin::handle_sys_status),
			/* handle raw message, check framing! */
			make_handler(mavlink::common::msg::STATUSTEXT::MSG_ID, &DummyPlugin::handle_statustext_raw),
			make_handler(&DummyPlugin::handle_statustext),
		};
	}

private:
	ros::NodeHandle nh;

	void handle_heartbeat(const mavlink::mavlink_message_t *msg, mavlink::minimal::msg::HEARTBEAT &hb) {
		ROS_INFO_STREAM_NAMED("dummy", "Dummy::handle_heartbeat: " << hb.to_yaml());
	}

	void handle_sys_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SYS_STATUS &st) {
		ROS_INFO_STREAM_NAMED("dummy", "Dummy::handle_sys_status: " << st.to_yaml());
	}

	void handle_statustext(const mavlink::mavlink_message_t *msg, mavlink::common::msg::STATUSTEXT &st) {
		ROS_INFO_STREAM_NAMED("dummy", "Dummy::handle_statustext: " << st.to_yaml());
	}

	void handle_statustext_raw(const mavlink::mavlink_message_t *msg, const mavconn::Framing f) {
		ROS_INFO_NAMED("dummy", "Dummy::handle_statustext_raw(%p, %d) from %u.%u", msg, utils::enum_value(f), msg->sysid, msg->compid);
	}
};

}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::DummyPlugin, mavros::plugin::PluginBase)
