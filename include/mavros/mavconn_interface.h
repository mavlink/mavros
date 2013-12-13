/**
 * @file mavconn_interface.h
 * @author Vladimit Ermkov <voon341@gmail.com>
 */
/*
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

#include <mavlink/v1.0/ardupilotmega/mavlink.h>

namespace mavconn {
namespace sig2 = boost::signals2;

class MAVConnInterface {
public:
	virtual bool send_message(const mavlink_message_t *message) = 0;
	virtual bool send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid) = 0;

	sig2::signal<void(const mavlink_message_t *massage, uint8_t sysid, uint8_t compid)> message_received;
	sig2::signal<void()> port_closed;

	virtual mavlink_status_t *get_status();
	virtual bool is_open() = 0;

protected:
	uint8_t conn_id;
	uint8_t sys_id;
	uint8_t comp_id;

	static int new_channel();
	static void delete_channel(int chan);

private:
	static std::set<int> allocated_channels;
};

}; // namespace mavconn
