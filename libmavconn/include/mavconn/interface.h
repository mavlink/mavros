/**
 * @brief MAVConn class interface
 * @file interface.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 *  @brief MAVConn connection handling library
 *
 *  This lib provide simple interface to MAVLink enabled devices
 *  such as autopilots.
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <boost/system/system_error.hpp>

#include <set>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <unordered_map>
#include <mavconn/simplesignal.h>
#include <mavconn/mavlink_dialect.h>

#include <boost/signals2.hpp>

namespace mavconn {
class MsgBuffer;

using steady_clock = std::chrono::steady_clock;
using lock_guard = std::lock_guard<std::recursive_mutex>;

static constexpr auto MAV_COMP_ID_UDP_BRIDGE = 240;

/**
 * @brief Common exception for communication error
 */
class DeviceError : public std::runtime_error {
public:
	/**
	 * @breif Construct error.
	 */
	template <typename T>
	DeviceError(const char *module, T msg) :
		std::runtime_error(make_message(module, msg))
	{ };

	template <typename T>
	static std::string make_message(const char *module, T msg) {
		std::ostringstream ss;
		ss << "DeviceError:" << module << ":" << msg_to_string(msg);
		return ss.str();
	}

	static std::string msg_to_string(const char *description) {
		return description;
	}

	static std::string msg_to_string(int errnum) {
		return ::strerror(errnum);
	}

	static std::string msg_to_string(boost::system::system_error &err) {
		return err.what();
	}
};

/**
 * @brief Generic mavlink interface
 */
class MAVConnInterface {
private:
	MAVConnInterface(const MAVConnInterface&) = delete;

public:
	//using MessageSig = signal::Signal<void (const mavlink::mavlink_message_t *message, uint8_t system_id, uint8_t component_id)>;
	using MessageSig = boost::signals2::signal<void (const mavlink::mavlink_message_t *message)>;
	using Ptr = std::shared_ptr<MAVConnInterface>;
	using ConstPtr = std::shared_ptr<MAVConnInterface const>;
	using WeakPtr = std::weak_ptr<MAVConnInterface>;

	struct IOStat {
		size_t tx_total_bytes;	//!< total bytes transferred
		size_t rx_total_bytes;	//!< total bytes received
		float tx_speed;		//!< current transfer speed [B/s]
		float rx_speed;		//!< current receive speed [B/s]
	};

	/**
	 * @param[in] system_id     sysid for send_message
	 * @param[in] component_id  compid for send_message
	 */
	MAVConnInterface(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE);

	/**
	 * @brief Close connection.
	 */
	virtual void close() = 0;

	/**
	 * @brief Send message (mavlink_message_t)
	 *
	 * Can be used to forward messages from other connection channel.
	 *
	 * @note Does not do finalization!
	 *
	 * @param[in] *message  not changed
	 */
	virtual void send_message(const mavlink::mavlink_message_t *message) = 0;

	/**
	 * @brief Send message (children of mavlink::Message)
	 *
	 * Does serialization inside.
	 * System and Component ID = from this object.
	 *
	 * @param[in] &message  not changed
	 */
	virtual void send_message(const mavlink::Message &message) = 0;

	/**
	 * @brief Send raw bytes (for some quirks)
	 */
	virtual void send_bytes(const uint8_t *bytes, size_t length) = 0;

	/**
	 * @brief Message receive signal
	 */
	MessageSig message_received;
	signal::Signal<void()> port_closed;

	virtual mavlink::mavlink_status_t get_status();
	virtual IOStat get_iostat();
	virtual bool is_open() = 0;

	inline uint8_t get_system_id() {
		return sys_id;
	};
	inline void set_system_id(uint8_t sysid) {
		sys_id = sysid;
	};
	inline uint8_t get_component_id() {
		return comp_id;
	};
	inline void set_component_id(uint8_t compid) {
		comp_id = compid;
	};

	/**
	 * @brief Construct connection from URL
	 *
	 * Supported URL schemas:
	 * - serial://
	 * - udp://
	 * - tcp://
	 * - tcp-l://
	 *
	 * Please see user's documentation for details.
	 *
	 * @param[in] url           resource locator
	 * @param[in] system_id     optional system id
	 * @param[in] component_id  optional component id
	 * @return @a Ptr to constructed interface class,
	 *         or throw @a DeviceError if error occured.
	 */
	static Ptr open_url(std::string url,
			uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE);

protected:
	uint8_t sys_id;
	uint8_t comp_id;

	static std::unordered_map<mavlink::msgid_t, const mavlink::mavlink_msg_entry_t*> message_entries;

	// Bye-bye channel limitation!
	//static int new_channel();
	//static void delete_channel(int chan);

	/**
	 * This helper function construct new MsgBuffer from message.
	 */
	MsgBuffer *new_msgbuffer(const mavlink::mavlink_message_t *message);
	MsgBuffer *new_msgbuffer(const mavlink::Message &message);

	inline mavlink::mavlink_status_t *get_status_p(void) {
		return &m_status;
	}

	inline mavlink::mavlink_message_t *get_buffer_p(void) {
		return &m_buffer;
	}

	/**
	 * Parse buffer and emit massage_received.
	 */
	void parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize, size_t bytes_received);

	void iostat_tx_add(size_t bytes);
	void iostat_rx_add(size_t bytes);

	void log_recv(const char *pfx, mavlink::mavlink_message_t &msg);
	void log_send(const char *pfx, const mavlink::mavlink_message_t *msg);
	void log_send_obj(const char *pfx, const mavlink::Message &msg);

private:
	friend const mavlink::mavlink_msg_entry_t* mavlink::mavlink_get_msg_entry(uint32_t msgid);

	mavlink::mavlink_status_t m_status;
	mavlink::mavlink_message_t m_buffer;

	std::atomic<size_t> tx_total_bytes, rx_total_bytes;
	std::recursive_mutex iostat_mutex;
	size_t last_tx_total_bytes, last_rx_total_bytes;
	std::chrono::time_point<steady_clock> last_iostat;

	// initialize message_entries map
	static std::recursive_mutex init_mutex;
	// autogenerated. placed in mavlink_helpers.cpp
	static void init_msg_entry(void);
};
}	// namespace mavconn
