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
#include <functional>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>
#include <mavros/mavros_uas.h>

namespace mavros {
namespace plugin {
using mavros::UAS;
typedef std::lock_guard<std::recursive_mutex> lock_guard;
typedef std::unique_lock<std::recursive_mutex> unique_lock;

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
	using HandlerInfo = std::tuple<mavlink::msgid_t, const char*, size_t, HandlerCb>;
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
	virtual Subscriptions get_subscriptions() = 0;

protected:
	/**
	 * @brief Plugin constructor
	 * Should not do anything before initialize()
	 */
	PluginBase() : m_uas(nullptr) {};

	UAS *m_uas;

	// TODO: filtered handlers

	/**
	 * Make subscription to raw message.
	 *
	 * @param[in] id  message id
	 * @param[in] fn  pointer to member function (handler)
	 */
	template<class _C>
	HandlerInfo make_handler(const mavlink::msgid_t id, void (_C::*fn)(const mavlink::mavlink_message_t *msg, const mavconn::Framing framing)) {
		auto bfn = std::bind(fn, static_cast<_C*>(this), std::placeholders::_1, std::placeholders::_2);
		const auto type_hash_ = typeid(mavlink::mavlink_message_t).hash_code();

		return HandlerInfo{ id, nullptr, type_hash_, bfn };
	}

	/**
	 * Make subscription to message with automatic decoding.
	 *
	 * @param[in] fn  pointer to member function (handler)
	 */
	template<class _C, class _T>
	HandlerInfo make_handler(void (_C::*fn)(const mavlink::mavlink_message_t*, _T &)) {
		auto bfn = std::bind(fn, static_cast<_C*>(this), std::placeholders::_1, std::placeholders::_2);
		const auto id = _T::MSG_ID;
		const auto name = _T::NAME;
		const auto type_hash_ = typeid(_T).hash_code();

		return HandlerInfo{
			       id, name, type_hash_,
			       [bfn](const mavlink::mavlink_message_t *msg, const mavconn::Framing framing) {
				       if (framing != mavconn::Framing::ok)
					       return;

				       mavlink::MsgMap map(msg);
				       _T obj;
				       obj.deserialize(map);

				       bfn(msg, obj);
			       }
		};
	}

	/**
	 * Common callback called on connection change
	 */
	virtual void connection_cb(bool connected) {
		ROS_BREAK();
	}

	/**
	 * Shortcut for connection_cb() registration
	 */
	inline void enable_connection_cb() {
		m_uas->add_connection_change_handler(std::bind(&PluginBase::connection_cb, this, std::placeholders::_1));
	}

	/**
	 * Common callback called only when capabilities change
	 */
	virtual void capabilities_cb(UAS::MAV_CAP capabilities) {
		ROS_BREAK();
	}

	/**
	 * Shortcut for capabilities_cb() registration
	 */
	void enable_capabilities_cb() {
		m_uas->add_capabilities_change_handler(std::bind(&PluginBase::capabilities_cb, this, std::placeholders::_1));
	}
};
}	// namespace plugin
}	// namespace mavros
