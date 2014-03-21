/**
 * @brief MAVROS Plugin base
 * @file mavros_plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
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

#pragma once

#include <diagnostic_updater/diagnostic_updater.h>
#include <mavros/mavconn_interface.h>
#include <mavros/mavros_uas.h>

namespace mavplugin {

class MavRosPlugin
{
public:
	virtual ~MavRosPlugin() {};
	virtual void initialize(UAS &uas,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater) = 0;
	virtual std::string get_name() = 0;
	virtual std::vector<uint8_t> get_supported_messages() = 0;
	virtual void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) = 0;

protected:
	MavRosPlugin() {};
};

}; // namespace mavplugin
