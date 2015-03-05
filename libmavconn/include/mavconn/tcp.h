/**
 * @brief MAVConn TCP link classes
 * @file mavconn_tcp.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2014,2015 Vladimir Ermakov, All rights reserved.
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
 * @brief TCP client interface
 *
 * @note IPv4 only
 */
class MAVConnTCPClient : public MAVConnInterface {
public:
	/**
	 * Create generic TCP client (connect to the server)
	 * @param[id] server_addr    remote host
	 * @param[id] server_port    remote port
	 */
	MAVConnTCPClient(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string server_host = "localhost", unsigned short server_port = 5760);
	/**
	 * Special client variation for use in MAVConnTCPServer
	 */
	explicit MAVConnTCPClient(uint8_t system_id, uint8_t component_id,
			boost::asio::io_service &server_io);
	~MAVConnTCPClient();

	void close();

	using MAVConnInterface::send_message;
	void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
	void send_bytes(const uint8_t *bytes, size_t length);

	inline bool is_open() { return socket.is_open(); };

private:
	friend class MAVConnTCPServer;
	boost::asio::io_service io_service;
	std::unique_ptr<boost::asio::io_service::work> io_work;
	std::thread io_thread;

	boost::asio::ip::tcp::socket socket;
	boost::asio::ip::tcp::endpoint server_ep;

	std::atomic<bool> tx_in_progress;
	std::list<MsgBuffer*> tx_q;
	uint8_t rx_buf[MsgBuffer::MAX_SIZE];
	std::recursive_mutex mutex;

	/**
	 * This special function called by TCP server when connection accepted.
	 */
	void client_connected(int server_channel);

	void do_recv();
	void async_receive_end(boost::system::error_code, size_t bytes_transferred);
	void do_send(bool check_tx_state);
	void async_send_end(boost::system::error_code, size_t bytes_transferred);
};

/**
 * @brief TCP server interface
 *
 * @note IPv4 only
 */
class MAVConnTCPServer : public MAVConnInterface {
public:
	/**
	 * @param[id] server_addr    bind host
	 * @param[id] server_port    bind port
	 */
	MAVConnTCPServer(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string bind_host = "localhost", unsigned short bind_port = 5760);
	~MAVConnTCPServer();

	void close();

	using MAVConnInterface::send_message;
	void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
	void send_bytes(const uint8_t *bytes, size_t length);

	mavlink_status_t get_status();
	IOStat get_iostat();
	inline bool is_open() { return acceptor.is_open(); };

private:
	boost::asio::io_service io_service;
	std::unique_ptr<boost::asio::io_service::work> io_work;
	std::thread io_thread;

	boost::asio::ip::tcp::acceptor acceptor;
	boost::asio::ip::tcp::endpoint bind_ep;

	boost::shared_ptr<MAVConnTCPClient> acceptor_client;
	std::list<boost::shared_ptr<MAVConnTCPClient> > client_list;
	std::recursive_mutex mutex;

	void do_accept();
	void async_accept_end(boost::system::error_code);

	// client slots
	void client_closed(boost::weak_ptr<MAVConnTCPClient> weak_instp);
	void recv_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
};

}; // namespace mavconn

