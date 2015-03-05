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
 * Copyright 2013 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace mavplugin {
/**
 * @brief Dummy plugin.
 *
 * Example and "how to" for users.
 */
class DummyPlugin : public MavRosPlugin {
public:
	DummyPlugin() :
		nh("~"),
		uas(nullptr)
	{ };

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		uas = &uas_;

		ROS_INFO_NAMED("dummy", "initialize");
	}

	/**
	 * This function returns message<->handler mapping
	 *
	 * Each entry defined by @a MESSAGE_HANDLER() macro
	 */
	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT, &DummyPlugin::handle_heartbeat),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_SYS_STATUS, &DummyPlugin::handle_sys_status),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_STATUSTEXT, &DummyPlugin::handle_statustext)
		};
	}

private:
	ros::NodeHandle nh;
	UAS *uas;

	void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		ROS_INFO_NAMED("dummy", "Dummy::handle_heartbeat(%p, %u, %u)",
				msg, sysid, compid);
	}

	void handle_sys_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		ROS_INFO_NAMED("dummy", "Dummy::handle_sys_status(%p, %u, %u)",
				msg, sysid, compid);
	}

	void handle_statustext(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		ROS_INFO_NAMED("dummy", "Dummy::handle_statustext(%p, %u, %u)",
				msg, sysid, compid);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::DummyPlugin, mavplugin::MavRosPlugin)

