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
#include <boost/signals2.hpp>

#include <set>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
//#include <mavconn/simplesignal.h>
#include <mavconn/mavlink_dialect.h>

namespace mavconn {
class MsgBuffer;

using steady_clock = std::chrono::steady_clock;
using lock_guard = std::lock_guard<std::recursive_mutex>;

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
		return strerror(errnum);
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
	//using MessageSig = signal::Signal<void (const mavlink_message_t *message, uint8_t system_id, uint8_t component_id)>;
	using MessageSig = boost::signals2::signal<void (const mavlink_message_t *message, uint8_t system_id, uint8_t component_id)>;
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
	virtual ~MAVConnInterface() {
		delete_channel(channel);
	};

	/**
	 * @brief Close connection.
	 */
	virtual void close() = 0;

	/**
	 * @brief Send message with default link system/component id
	 */
	inline void send_message(const mavlink_message_t *message) {
		send_message(message, sys_id, comp_id);
	};

	/**
	 * @brief Send message
	 * Can be used in message_received signal.
	 *
	 * @param[in] *message  not changed
	 * @param[in] sysid     message sys id
	 * @param[in] compid    message component id
	 */
	virtual void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid) = 0;

	/**
	 * @brief Send raw bytes (for some quirks)
	 */
	virtual void send_bytes(const uint8_t *bytes, size_t length) = 0;

	/**
	 * @brief Message receive signal
	 */
	MessageSig message_received;
	//signal::Signal<void()> port_closed;
	boost::signals2::signal<void()> port_closed;

	virtual mavlink_status_t get_status();
	virtual IOStat get_iostat();
	virtual bool is_open() = 0;

	inline int get_channel() {
		return channel;
	};
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
	int channel;
	uint8_t sys_id;
	uint8_t comp_id;

#if MAVLINK_CRC_EXTRA
	static const uint8_t mavlink_crcs[];
#endif

	static int new_channel();
	static void delete_channel(int chan);
	static int channes_available();

	/**
	 * This helper function construct new MsgBuffer from message.
	 */
	MsgBuffer *new_msgbuffer(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);

	/**
	 * Parse buffer and emit massage_received.
	 */
	void parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize, size_t bytes_received);

	void iostat_tx_add(size_t bytes);
	void iostat_rx_add(size_t bytes);

	void log_recv(const char *pfx, mavlink_message_t &msg);
	void log_send(const char *pfx, const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

private:
	friend mavlink_status_t* ::mavlink_get_channel_status(uint8_t chan);
	friend mavlink_message_t* ::mavlink_get_channel_buffer(uint8_t chan);

	static std::recursive_mutex channel_mutex;
	static std::set<int> allocated_channels;
	static mavlink_status_t channel_status[MAVLINK_COMM_NUM_BUFFERS];
	static mavlink_message_t channel_buffer[MAVLINK_COMM_NUM_BUFFERS];

	std::atomic<size_t> tx_total_bytes, rx_total_bytes;
	std::recursive_mutex iostat_mutex;
	size_t last_tx_total_bytes, last_rx_total_bytes;
	std::chrono::time_point<steady_clock> last_iostat;
};
}	// namespace mavconn
