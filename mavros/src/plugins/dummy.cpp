/**
 * @brief Dummy plugin
 * @file dummy.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace mavplugin {

/**
 * @brief Dummy plugin.
 * Example and "how to" for users.
 *
 * @example
 */
class DummyPlugin : public MavRosPlugin {
public:
	DummyPlugin() {
		ROS_INFO_NAMED("dummy", "dummy constructor");
	};

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		ROS_INFO_NAMED("dummy", "initialize");
	};

	/**
	 * Returns plugin name (CamelCase)
	 */
	std::string const get_name() const {
		return "Dummy";
	};

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

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::DummyPlugin, mavplugin::MavRosPlugin)

