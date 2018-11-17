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
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

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
class MAVConnUDP : public MAVConnInterface,
	public std::enable_shared_from_this<MAVConnUDP> {
public:
	static constexpr auto DEFAULT_BIND_HOST = "localhost";
	static constexpr auto DEFAULT_BIND_PORT = 14555;
	static constexpr auto DEFAULT_REMOTE_HOST = "";
	static constexpr auto DEFAULT_REMOTE_PORT = 14550;
	//! Markers for broadcast modes. Not valid domain names.
	static constexpr auto BROADCAST_REMOTE_HOST = "***i want broadcast***";
	static constexpr auto PERMANENT_BROADCAST_REMOTE_HOST = "***permanent broadcast***";

	/**
	 * @param[id] bind_host    bind host
	 * @param[id] bind_port    bind port
	 * @param[id] remote_host  remote host (optional)
	 * @param[id] remote_port  remote port (optional)
	 */
	MAVConnUDP(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string bind_host = DEFAULT_BIND_HOST, unsigned short bind_port = DEFAULT_BIND_PORT,
			std::string remote_host = DEFAULT_REMOTE_HOST, unsigned short remote_port = DEFAULT_REMOTE_PORT);
	~MAVConnUDP();

	void close() override;

	void send_message(const mavlink::mavlink_message_t *message) override;
	void send_message(const mavlink::Message &message, const uint8_t source_compid) override;
	void send_bytes(const uint8_t *bytes, size_t length) override;

	inline bool is_open() override {
		return socket.is_open();
	}

private:
	boost::asio::io_service io_service;
	std::unique_ptr<boost::asio::io_service::work> io_work;
	std::thread io_thread;
	bool permanent_broadcast;

	std::atomic<bool> remote_exists;
	boost::asio::ip::udp::socket socket;
	boost::asio::ip::udp::endpoint remote_ep;
	boost::asio::ip::udp::endpoint recv_ep;
	boost::asio::ip::udp::endpoint last_remote_ep;
	boost::asio::ip::udp::endpoint bind_ep;

	std::atomic<bool> tx_in_progress;
	std::deque<MsgBuffer> tx_q;
	std::array<uint8_t, MsgBuffer::MAX_SIZE> rx_buf;
	std::recursive_mutex mutex;

	void do_recvfrom();
	void do_sendto(bool check_tx_state);
};
}	// namespace mavconn
