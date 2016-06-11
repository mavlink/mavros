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

#include <tuple>
#include <vector>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>
#include <mavros/mavros_uas.h>

namespace mavros {
namespace plugin {

using mavros::UAS;
typedef std::lock_guard<std::recursive_mutex> lock_guard;
typedef std::unique_lock<std::recursive_mutex> unique_lock;

/**
 * @brief Helper macros to define message handler map item
 */
//#define MESSAGE_HANDLER(_message_id, _class_method_ptr)	\
//	{ _message_id, boost::bind(_class_method_ptr, this, _1, _2, _3) }

#define MESSAGE_HANDLER(_message_id, _class_method_ptr)

/**
 * @brief MAVROS Plugin base class
 */
class PluginBase
{
private:
	PluginBase(const PluginBase&) = delete;

public:
	using TypeInfoRef = std::reference_wrapper<const std::type_info>;
	//! generic message handler callback
	using HandlerCb = mavconn::MAVConnInterface::ReceivedCb;
	//! Tuple: MSG ID, MSG NAME, message type into reference, message handler callback
	using HandlerInfo = std::tuple<mavlink::msgid_t, const char *, TypeInfoRef, HandlerCb>;
	//! Subscriptions vector
	using Subscriptions = std::vector<HandlerInfo>;

	// pluginlib return boost::shared_ptr
	using Ptr = boost::shared_ptr<PluginBase>;
	using ConstPtr = boost::shared_ptr<PluginBase const>;

	virtual ~PluginBase() {};

	/**
	 * @brief Plugin initializer
	 *
	 * @param[in] uas  @p UAS instance
	 */
	virtual void initialize(UAS &uas) {
		m_uas = &uas;
	}

	/**
	 * @brief Return vector of MAVLink message subscriptions (handlers)
	 */
	virtual Subscriptions&& get_subscriptions() = 0;

protected:
	/**
	 * @brief Plugin constructor
	 * Should not do anything before initialize()
	 */
	PluginBase() {};

	UAS *m_uas;
};
}	// namespace plugin
}	// namespace mavros
