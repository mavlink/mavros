/**
 * @brief MAVConn UDP link class
 * @file mavconn_udp.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
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

#include <list>
#include <atomic>
#include <boost/asio.hpp>
#include <mavconn/interface.h>
#include <mavconn/msgbuffer.h>

namespace mavconn {

/**
 * @brief UDP interface
 *
 * @note IPv4 only
 */
class MAVConnUDP : public MAVConnInterface {
public:
	/**
	 * @param[id] bind_host    bind host
	 * @param[id] bind_port    bind port
	 * @param[id] remote_host  remote host (optional)
	 * @param[id] remote_port  remote port (optional)
	 */
	MAVConnUDP(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string bind_host = "localhost", unsigned short bind_port = 14555,
			std::string remote_host = "", unsigned short remote_port = 14550);
	~MAVConnUDP();

	void close();

	using MAVConnInterface::send_message;
	void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
	void send_bytes(const uint8_t *bytes, size_t length);

	inline bool is_open() { return socket.is_open(); };

private:
	boost::asio::io_service io_service;
	std::unique_ptr<boost::asio::io_service::work> io_work;
	std::thread io_thread;

	std::atomic<bool> remote_exists;
	boost::asio::ip::udp::socket socket;
	boost::asio::ip::udp::endpoint remote_ep;
	boost::asio::ip::udp::endpoint last_remote_ep;
	boost::asio::ip::udp::endpoint bind_ep;

	std::atomic<bool> tx_in_progress;
	std::list<MsgBuffer*> tx_q;
	uint8_t rx_buf[MsgBuffer::MAX_SIZE];
	std::recursive_mutex mutex;

	void do_recvfrom();
	void async_receive_end(boost::system::error_code, size_t bytes_transferred);
	void do_sendto(bool check_tx_state);
	void async_sendto_end(boost::system::error_code, size_t bytes_transferred);
};

}; // namespace mavconn

