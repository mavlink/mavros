/**
 * @brief MAVROS Plugin base
 * @file mavros_plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *  @brief MAVROS Plugin system
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

#include <boost/noncopyable.hpp>

namespace mavplugin {

/**
 * @brief MAVROS Plugin base class
 */
class MavRosPlugin : private boost::noncopyable
{
public:
	virtual ~MavRosPlugin() {};

	/**
	 * @brief Plugin initializer
	 *
	 * @param[in] uas           UAS instance (handles FCU connection and some statuses)
	 * @param[in] nh            main mavros ros::NodeHandle (private)
	 * @param[in] diag_updater  main diagnostic updater
	 */
	virtual void initialize(UAS &uas,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater) = 0;

	/**
	 * @brief Plugin name (CamelCase)
	 */
	virtual const std::string get_name() const = 0;

	/**
	 * @brief List of messages supported by message_rx_cb()
	 */
	virtual const std::vector<uint8_t> get_supported_messages() const = 0;

	/**
	 * @brief Message receive callback
	 */
	virtual void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) = 0;

protected:
	/**
	 * @brief Plugin constructor
	 * Should not do anything before initialize()
	 */
	MavRosPlugin() {};
};

}; // namespace mavplugin
