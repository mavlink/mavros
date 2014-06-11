/**
 * @brief MAVConn class interface
 * @file mavconn_interface.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
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

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <set>
#include <mavros/mavconn_mavlink.h>

namespace mavconn {
namespace sig2 = boost::signals2;
namespace asio = boost::asio;

class MAVConnInterface {
public:
	MAVConnInterface(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE);
	virtual ~MAVConnInterface() {
		delete_channel(channel);
	};

	inline void send_message(const mavlink_message_t *message) {
		send_message(message, sys_id, comp_id);
	};
	virtual void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid) = 0;
	virtual void send_bytes(const uint8_t *bytes, size_t length) = 0;

	sig2::signal<void(const mavlink_message_t *message, uint8_t system_id, uint8_t component_id)> message_received;
	sig2::signal<void()> port_closed;

	virtual mavlink_status_t get_status() = 0;
	virtual bool is_open() = 0;

	inline int get_channel() { return channel; };
	inline uint8_t get_system_id() { return sys_id; };
	inline void set_system_id(uint8_t sysid) { sys_id = sysid; };
	inline uint8_t get_component_id() { return comp_id; };
	inline void set_component_id(uint8_t compid) { comp_id = compid; };

protected:
	int channel;
	uint8_t sys_id;
	uint8_t comp_id;

#if MAVLINK_CRC_EXTRA
	static const uint8_t mavlink_crcs[];
#endif

	static int new_channel();
	static void delete_channel(int chan);

private:
	static std::set<int> allocated_channels;
};

}; // namespace mavconn
