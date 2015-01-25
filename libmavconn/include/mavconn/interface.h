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
 * Copyright 2013,2014 Vladimir Ermakov.
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

#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/system/system_error.hpp>

#include <set>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <memory>
#include <sstream>
#include <mavconn/mavlink_dialect.h>

namespace mavconn {
namespace sig2 = boost::signals2;

class MsgBuffer;

#if __cplusplus == 201103L
using steady_clock = std::chrono::steady_clock;
#elif defined(__GXX_EXPERIMENTAL_CXX0X__)
typedef std::chrono::monotonic_clock steady_clock;
#else
#error Unknown C++11 or C++0x wall clock class
#endif

/**
 * @brief Common exception for communication error
 */
class DeviceError : public std::exception {
private:
	std::string e_what_;

public:
	/**
	 * @breif Construct error with description.
	 */
	explicit DeviceError(const char *module, const char *description) {
		std::ostringstream ss;
		ss << "DeviceError:" << module << ": " << description;
		e_what_ = ss.str();
	}

	/**
	 * @brief Construct error from errno
	 */
	explicit DeviceError(const char *module, int errnum) {
		std::ostringstream ss;
		ss << "DeviceError:" << module << ":" << errnum << ": " << strerror(errnum);
		e_what_ = ss.str();
	}

	/**
	 * @brief Construct error from boost error exception
	 */
	explicit DeviceError(const char *module, boost::system::system_error &err) {
		std::ostringstream ss;
		ss << "DeviceError:" << module << ":" << err.what();
		e_what_ = ss.str();
	}

	DeviceError(const DeviceError& other) : e_what_(other.e_what_) {}
	virtual ~DeviceError() throw() {}
	virtual const char *what() const throw() {
		return e_what_.c_str();
	}
};

/**
 * @brief Generic mavlink interface
 */
class MAVConnInterface {
private:
	MAVConnInterface(const MAVConnInterface&) = delete;

public:
	typedef sig2::signal<void(const mavlink_message_t *message, uint8_t system_id, uint8_t component_id)> MessageSig;
	typedef boost::shared_ptr<MAVConnInterface> Ptr;
	typedef boost::shared_ptr<MAVConnInterface const> ConstPtr;
	typedef boost::weak_ptr<MAVConnInterface> WeakPtr;

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
	sig2::signal<void()> port_closed;

	virtual mavlink_status_t get_status();
	virtual IOStat get_iostat();
	virtual bool is_open() = 0;

	inline int get_channel() { return channel; };
	inline uint8_t get_system_id() { return sys_id; };
	inline void set_system_id(uint8_t sysid) { sys_id = sysid; };
	inline uint8_t get_component_id() { return comp_id; };
	inline void set_component_id(uint8_t compid) { comp_id = compid; };

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

	void iostat_tx_add(size_t bytes);
	void iostat_rx_add(size_t bytes);

private:
	static std::recursive_mutex channel_mutex;
	static std::set<int> allocated_channels;

	std::atomic<size_t> tx_total_bytes, rx_total_bytes;
	std::recursive_mutex iostat_mutex;
	size_t last_tx_total_bytes, last_rx_total_bytes;
	std::chrono::time_point<steady_clock> last_iostat;
};

}; // namespace mavconn
