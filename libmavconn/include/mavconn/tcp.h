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
 * Copyright 2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <list>
#include <atomic>
#include <cstring>
#include <boost/asio.hpp>
#include <mavconn/interface.h>
#include <mavconn/msgbuffer.h>


namespace mavconn {
/**
 * @brief TCP client interface
 *
 * @note IPv4 only
 */
class MAVConnTCPClient : public MAVConnInterface,
	public std::enable_shared_from_this<MAVConnTCPClient> {
public:
	static constexpr auto DEFAULT_SERVER_HOST = "localhost";
	static constexpr auto DEFAULT_SERVER_PORT = 5760;

	/**
	 * Create generic TCP client (connect to the server)
	 * @param[id] server_addr    remote host
	 * @param[id] server_port    remote port
	 */
	MAVConnTCPClient(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string server_host = DEFAULT_SERVER_HOST, unsigned short server_port = DEFAULT_SERVER_PORT);
	/**
	 * Special client variation for use in MAVConnTCPServer
	 */
	explicit MAVConnTCPClient(uint8_t system_id, uint8_t component_id,
			boost::asio::io_service &server_io);
	~MAVConnTCPClient();

	void close() override;

	void send_message(const mavlink::mavlink_message_t *message) override;
	void send_message(const mavlink::Message &message, const uint8_t source_compid) override;
	void send_bytes(const uint8_t *bytes, size_t length) override;

	inline bool is_open() override {
		return socket.is_open();
	}

private:
	friend class MAVConnTCPServer;
	boost::asio::io_service io_service;
	std::unique_ptr<boost::asio::io_service::work> io_work;
	std::thread io_thread;

	boost::asio::ip::tcp::socket socket;
	boost::asio::ip::tcp::endpoint server_ep;

	std::atomic<bool> is_destroying;

	std::atomic<bool> tx_in_progress;
	std::deque<MsgBuffer> tx_q;
	std::array<uint8_t, MsgBuffer::MAX_SIZE> rx_buf;
	std::recursive_mutex mutex;

	/**
	 * This special function called by TCP server when connection accepted.
	 */
	void client_connected(size_t server_channel);

	void do_recv();
	void do_send(bool check_tx_state);
};

/**
 * @brief TCP server interface
 *
 * @note IPv4 only
 */
class MAVConnTCPServer : public MAVConnInterface,
	public std::enable_shared_from_this<MAVConnTCPServer> {
public:
	static constexpr auto DEFAULT_BIND_HOST = "localhost";
	static constexpr auto DEFAULT_BIND_PORT = 5760;

	/**
	 * @param[id] server_addr    bind host
	 * @param[id] server_port    bind port
	 */
	MAVConnTCPServer(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string bind_host = DEFAULT_BIND_HOST, unsigned short bind_port = DEFAULT_BIND_PORT);
	~MAVConnTCPServer();

	void close() override;

	void send_message(const mavlink::mavlink_message_t *message) override;
	void send_message(const mavlink::Message &message, const uint8_t source_compid) override;
	void send_bytes(const uint8_t *bytes, size_t length) override;

	mavlink::mavlink_status_t get_status() override;
	IOStat get_iostat() override;
	inline bool is_open() override {
		return acceptor.is_open();
	}

private:
	boost::asio::io_service io_service;
	std::unique_ptr<boost::asio::io_service::work> io_work;
	std::thread io_thread;

	boost::asio::ip::tcp::acceptor acceptor;
	boost::asio::ip::tcp::endpoint bind_ep;

	std::atomic<bool> is_destroying;

	std::list<std::shared_ptr<MAVConnTCPClient> > client_list;
	std::recursive_mutex mutex;

	void do_accept();

	// client slots
	void client_closed(std::weak_ptr<MAVConnTCPClient> weak_instp);
	void recv_message(const mavlink::mavlink_message_t *message, const Framing framing);
};
}	// namespace mavconn
