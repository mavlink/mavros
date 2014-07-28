/**
 * @brief MAVConn TCP link classes
 * @file mavconn_tcp.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
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
#include <ev++.h>
#include <mavros/mavconn_interface.h>
#include <mavros/mavconn_msgbuffer.h>

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

namespace mavconn {

/**
 * @brief TCP client interface
 *
 * @note IPv4 only
 */
class MAVConnTCPClient : public MAVConnInterface {
public:
	/**
	 * @param[id] server_addr    remote host
	 * @param[id] server_port    remote port
	 */
	MAVConnTCPClient(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string server_host = "localhost", unsigned short server_port = 5760);
	~MAVConnTCPClient();

	void close();

	using MAVConnInterface::send_message;
	void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
	void send_bytes(const uint8_t *bytes, size_t length);

	inline mavlink_status_t get_status() { return *mavlink_get_channel_status(channel); };
	inline bool is_open() { return sockfd != -1; };

private:
	ev::io io;
	int sockfd;

	sockaddr_in server_addr;

	std::list<MsgBuffer*> tx_q;
	boost::recursive_mutex mutex;

	void event_cb(ev::io &watcher, int revents);
	void read_cb(ev::io &watcher);
	void write_cb(ev::io &watcher);
};

#if 0

/**
 * @brief TCP server internal client class
 *
 * @note Because mavlink_message_parse() require chanel allocation,
 *       this class also use MAVConnInterface.
 */
class MAVConnTCPServerClient : public MAVConnInterface {
public:
	/**
	 * @param[in] clientfd    socket from MAVConnTCPServer
	 */
	MAVConnTCPServerClient(uint8_t system_id, uint8_t component_id, int clientfd);
	~MAVConnTCPServerClient();

	void close();

	void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
	void send_bytes(const uint8_t *bytes, size_t length);

private:
	ev::io io;
	int sockfd;

	std::list<MsgBuffer*> tx_q;
	boost::recursive_mutex mutex;

	void event_cb(ev::io &watcher, int revents);
	void read_cb(ev::io &watcher);
	void write_cb(ev::io &watcher);
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
			std::string server_addr = "localhost", unsigned short server_port = 5760);
	~MAVConnTCPServer();

	void close();

	using MAVConnInterface::send_message;
	void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
	void send_bytes(const uint8_t *bytes, size_t length);

	inline mavlink_status_t get_status() { return *mavlink_get_channel_status(channel); };
	inline bool is_open() { return sockfd != -1; };

private:
	ev::io io;
	int sockfd;

	sockaddr_in bind_addr;

	std::list<MAVConnTCPServerClient *> client_list;
	boost::recursive_mutex mutex;

	void event_cb(ev::io &watcher, int revents);
};

#endif

}; // namespace mavconn

