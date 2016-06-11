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

// XXX TODO: extract message-type from handler like NodeHandle::subscribe()

/**
 * @brief Helper macros to define message subscription item
 */
#define MESSAGE_HANDLER(_message_t, _class_method)				\
	PluginBase::HandlerInfo{ _message_t::MSG_ID, "NONAME", typeid(_message_t), 	\
		[this](const mavlink::mavlink_message_t *msg, const mavconn::Framing framing) { \
			if (framing != mavconn::Framing::ok)	return;		\
			_message_t obj; mavlink::MsgMap map(msg); obj.deserialize(map);		\
			_class_method(msg, obj);				\
		}								\
	}

/**
 * @brief Helpre macros to define mavlink_message_t subscription item
 */
#define MESSAGE_HANDLER_RAW(_message_id, _class_method_ptr)			\
	PluginBase::HandlerInfo{ (_message_id), nullptr, typeid(mavlink::mavlink_message_t),		\
		std::bind(_class_method_ptr, this, std::placeholders::_1, std::placeholders::_2)	\
	}


// XXX MESSAGE_HANDLER_LAMBDA???


/**
 * @brief MAVROS Plugin base class
 */
class PluginBase
{
private:
	PluginBase(const PluginBase&) = delete;

public:
	//! generic message handler callback
	using HandlerCb = mavconn::MAVConnInterface::ReceivedCb;
	//! Tuple: MSG ID, MSG NAME, message type into hash_code, message handler callback
	using HandlerInfo = std::tuple<mavlink::msgid_t, const char *, size_t, HandlerCb>;
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

		ROS_WARN_STREAM("plugin parent init");
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
