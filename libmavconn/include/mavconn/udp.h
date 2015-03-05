/**
 * @brief MAVConn UDP link class
 * @file mavconn_udp.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
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

