/**
 * @file mavconn_tcp.cpp
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

#include <mavros/utils.h>
#include <mavros/mavconn_tcp.h>
#include <ros/console.h>
#include <ros/assert.h>

namespace mavconn {
using boost::system::error_code;
using boost::asio::io_service;
using boost::asio::ip::tcp;
using boost::asio::buffer;
using lock_guard = std::lock_guard<std::recursive_mutex>;


static bool resolve_address_tcp(io_service &io, std::string host, unsigned short port, tcp::endpoint &ep)
{
	bool result = false;
	tcp::resolver resolver(io);
	error_code ec;

	tcp::resolver::query query(host, "");
	std::for_each(resolver.resolve(query, ec), tcp::resolver::iterator(),
		[&](const tcp::endpoint &q_ep) {
			ep = q_ep;
			ep.port(port);
			result = true;
			ROS_DEBUG_STREAM_NAMED("mavconn", "tcp: host " << host << " resolved as " << ep);
		});

	if (ec) {
		ROS_WARN_STREAM_NAMED("mavconn", "tcp: resolve error: " << ec);
		result = false;
	}

	return result;
}


/* -*- TCP client variant -*- */

MAVConnTCPClient::MAVConnTCPClient(uint8_t system_id, uint8_t component_id,
		std::string server_host, unsigned short server_port) :
	MAVConnInterface(system_id, component_id),
	tx_in_progress(false),
	io_service(),
	io_work(new io_service::work(io_service)),
	socket(io_service)
{
	if (!resolve_address_tcp(io_service, server_host, server_port, server_ep))
		throw DeviceError("tcp: resolve", "Bind address resolve failed");

	ROS_INFO_STREAM_NAMED("mavconn", "tcp" << channel << ": Server address: " << server_ep);

	try {
		socket.open(tcp::v4());
		socket.connect(server_ep);
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("tcp", err);
	}

	// give some work to io_service before start
	io_service.post(boost::bind(&MAVConnTCPClient::do_recv, this));

	// run io_service for async io
	std::thread t(boost::bind(&io_service::run, &this->io_service));
	mavutils::set_thread_name(t, "MAVConnTCPc%d", channel);
	io_thread.swap(t);
}

#if 0
MAVConnTCPClient::MAVConnTCPClient(uint8_t system_id, uint8_t component_id,
		int client_sd, sockaddr_in &client_addr) :
	MAVConnInterface(system_id, component_id),
	sockfd(client_sd),
	server_addr(client_addr)
{
	ROS_INFO_NAMED("mavconn", "tcp-l: Got client, channel %d, address: %s:%d",
			channel,
			inet_ntoa(server_addr.sin_addr),
			ntohs(server_addr.sin_port));

	if (fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL, 0) | O_NONBLOCK) == -1)
		throw DeviceError("tcp-l: fcntl", errno);

	// run io for async io
	io.set<MAVConnTCPClient, &MAVConnTCPClient::event_cb>(this);
	io.start(sockfd, ev::READ);
	start_default_loop(); // XXX: alredy started by server
}
#endif

MAVConnTCPClient::~MAVConnTCPClient() {
	close();
}

void MAVConnTCPClient::close() {
	lock_guard lock(mutex);
	if (!socket.is_open())
		return;

	io_work.reset();
	io_service.stop();
	socket.close();
	/* emit */ port_closed();

	// clear tx queue
	std::for_each(tx_q.begin(), tx_q.end(),
			[](MsgBuffer *p) { delete p; });
	tx_q.clear();

	io_thread.join();
}

void MAVConnTCPClient::send_bytes(const uint8_t *bytes, size_t length)
{
	ROS_ASSERT_MSG(is_open(), "Should not send messages on closed channel!");
	MsgBuffer *buf = new MsgBuffer(bytes, length);
	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
		io_service.post(boost::bind(&MAVConnTCPClient::do_send, this, true));
	}
}

void MAVConnTCPClient::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	ROS_ASSERT_MSG(is_open(), "Should not send messages on closed channel!");
	ROS_ASSERT(message != nullptr);
	MsgBuffer *buf = nullptr;

	/* if sysid/compid pair not match we need explicit finalize
	 * else just copy to buffer */
	if (message->sysid != sysid || message->compid != compid) {
		mavlink_message_t msg = *message;

#if MAVLINK_CRC_EXTRA
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len, mavlink_crcs[msg.msgid]);
#else
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len);
#endif

		buf = new MsgBuffer(&msg);
	}
	else
		buf = new MsgBuffer(message);

	ROS_DEBUG_NAMED("mavconn", "tcp%d:send: Message-ID: %d [%d bytes] Sys-Id: %d Comp-Id: %d",
			channel, message->msgid, message->len, sysid, compid);

	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
		io_service.post(boost::bind(&MAVConnTCPClient::do_send, this, true));
	}
}

void MAVConnTCPClient::do_recv()
{
	socket.async_receive(
			buffer(rx_buf, sizeof(rx_buf)),
			boost::bind(&MAVConnTCPClient::async_receive_end,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
}

void MAVConnTCPClient::async_receive_end(error_code error, size_t bytes_transferred)
{
	mavlink_message_t message;
	mavlink_status_t status;

	if (error) {
		ROS_ERROR_STREAM_NAMED("mavconn", "tcp" << channel << ":receive: " << error);
		close();
		return;
	}

	for (ssize_t i = 0; i < bytes_transferred; i++) {
		if (mavlink_parse_char(channel, rx_buf[i], &message, &status)) {
			ROS_DEBUG_NAMED("mavconn", "tcp%d:recv: Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d",
					channel, message.msgid, message.len, message.sysid, message.compid);

			/* emit */ message_received(&message, message.sysid, message.compid);
		}
	}

	do_recv();
}

void MAVConnTCPClient::do_send(bool check_tx_state)
{
	if (check_tx_state && tx_in_progress)
		return;

	lock_guard lock(mutex);
	if (tx_q.empty())
		return;

	MsgBuffer *buf = tx_q.front();
	socket.async_send(
			buffer(buf->dpos(), buf->nbytes()),
			boost::bind(&MAVConnTCPClient::async_send_end,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
}

void MAVConnTCPClient::async_send_end(error_code error, size_t bytes_transferred)
{
	if (error) {
		ROS_ERROR_STREAM_NAMED("mavconn", "tcp" << channel << ":sendto: " << error);
		close();
		return;
	}

	lock_guard lock(mutex);
	if (tx_q.empty())
		return;

	MsgBuffer *buf = tx_q.front();
	buf->pos += bytes_transferred;
	if (buf->nbytes() == 0) {
		tx_q.pop_front();
		delete buf;
	}

	if (!tx_q.empty())
		do_send(false);
}


/* -*- TCP server variant -*- */
#if 0
MAVConnTCPServer::MAVConnTCPServer(uint8_t system_id, uint8_t component_id,
		std::string server_host, unsigned short server_port) :
	MAVConnInterface(system_id, component_id),
	sockfd(-1)
{
	if (!resolve_address_tcp(server_host, server_port, bind_addr))
		throw DeviceError("tcp-l: resolve", "Bind address resolve failed");

	ROS_INFO_NAMED("mavconn", "tcp-l: Bind address: %s:%d",
			inet_ntoa(bind_addr.sin_addr), ntohs(bind_addr.sin_port));

	sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
		throw DeviceError("tcp-l: socket", errno);

	int one = 1;
	if (::setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) < 0)
		throw DeviceError("tcp-l: setsockopt", errno);

	if (::bind(sockfd, (sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
		throw DeviceError("tcp-l: bind", errno);

	if (::listen(sockfd, channes_available()))
		throw DeviceError("tcp-l: listen", errno);

	if (fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL, 0) | O_NONBLOCK) == -1)
		throw DeviceError("tcp-l: fcntl", errno);

	// run io for async io
	io.set<MAVConnTCPServer, &MAVConnTCPServer::accept_cb>(this);
	io.start(sockfd, ev::READ);
	start_default_loop();
}

MAVConnTCPServer::~MAVConnTCPServer() {
	close();
}

void MAVConnTCPServer::close() {
	if (sockfd < 0)
		return;

	std::for_each(client_list.cbegin(), client_list.cend(),
			[](MAVConnTCPClient *instp) {
		instp->close();
	});

	io.stop();
	::close(sockfd); sockfd = -1;

	/* emit */ port_closed();
}

void MAVConnTCPServer::send_bytes(const uint8_t *bytes, size_t length)
{
	boost::recursive_mutex::scoped_lock lock(mutex);
	std::for_each(client_list.begin(), client_list.end(),
			[bytes, length](MAVConnTCPClient *instp) {
		instp->send_bytes(bytes, length);
	});
}

void MAVConnTCPServer::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	boost::recursive_mutex::scoped_lock lock(mutex);
	std::for_each(client_list.begin(), client_list.end(),
			[message, sysid, compid](MAVConnTCPClient *instp) {
		instp->send_message(message, sysid, compid);
	});
}

void MAVConnTCPServer::accept_cb(ev::io &watcher, int revents)
{
	if (ev::ERROR & revents) {
		ROS_ERROR_NAMED("mavconn", "accept_cb::revents: 0x%08x", revents);
		close();
		return;
	}

	if (!(ev::READ & revents))
		return;

	struct sockaddr_in client_addr;
	socklen_t client_len = sizeof(client_addr);

	int client_sd = accept(watcher.fd, (struct sockaddr *)&client_addr, &client_len);
	if (client_sd < 0) {
		ROS_ERROR_NAMED("mavconn", "tcp-l: accept: %s", strerror(errno));
		return;
	}

	if (channes_available() <= 0) {
		ROS_ERROR_NAMED("mavconn", "tcp-l:accept_cb: all channels in use, drop connection");
		::close(client_sd);
		return;
	}

	MAVConnTCPClient *instp = new MAVConnTCPClient(sys_id, comp_id, client_sd, client_addr);
	instp->message_received.connect(boost::bind(&MAVConnTCPServer::recv_message, this, _1, _2, _3));
	instp->port_closed.connect(boost::bind(&MAVConnTCPServer::client_closed, this, instp));

	client_list.push_back(instp);
}

void MAVConnTCPServer::client_closed(MAVConnTCPClient *instp)
{
	ROS_INFO_NAMED("mavconn", "tcp-l: Client connection closed, channel %d, address: %s:%d",
			instp->channel,
			inet_ntoa(instp->server_addr.sin_addr),
			ntohs(instp->server_addr.sin_port));

	client_list.remove(instp);
	delete instp;
}

void MAVConnTCPServer::recv_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	message_received(message, sysid, compid);
}
#endif
}; // namespace mavconn
