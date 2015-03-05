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
 * Copyright 2013,2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <map>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>
#include <mavros/mavros_uas.h>

namespace mavplugin {
using mavros::UAS;
typedef std::lock_guard<std::recursive_mutex> lock_guard;
typedef std::unique_lock<std::recursive_mutex> unique_lock;

/**
 * @brief Helper macros to define message handler map item
 */
#define MESSAGE_HANDLER(_message_id, _class_method_ptr)	\
	{ _message_id, boost::bind(_class_method_ptr, this, _1, _2, _3) }

/**
 * @brief MAVROS Plugin base class
 */
class MavRosPlugin
{
private:
	MavRosPlugin(const MavRosPlugin&) = delete;

public:
	typedef boost::function<void (const mavlink_message_t *msg, uint8_t sysid, uint8_t compid)> message_handler;
	typedef std::map<uint8_t, message_handler> message_map;
	// pluginlib return boost::shared_ptr
	typedef boost::shared_ptr<MavRosPlugin> Ptr;
	typedef boost::shared_ptr<MavRosPlugin const> ConstPtr;

	virtual ~MavRosPlugin() {};

	/**
	 * @brief Plugin initializer
	 *
	 * @param[in] uas           UAS instance (handles FCU connection and some statuses)
	 */
	virtual void initialize(UAS &uas) = 0;

	/**
	 * @brief Return map with message rx handlers
	 */
	virtual const message_map get_rx_handlers() = 0;

protected:
	/**
	 * @brief Plugin constructor
	 * Should not do anything before initialize()
	 */
	MavRosPlugin() {};
};
};	// namespace mavplugin
