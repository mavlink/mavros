/**
 * @brief MAVConn TCP link classes
 * @file tcp.cpp
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

#include <cassert>
#include <console_bridge/console.h>

#include <mavconn/thread_utils.h>
#include <mavconn/tcp.h>

namespace mavconn {
using boost::system::error_code;
using boost::asio::io_service;
using boost::asio::ip::tcp;
using boost::asio::buffer;
using mavutils::to_string_ss;
typedef std::lock_guard<std::recursive_mutex> lock_guard;

#define PFXd	"mavconn: tcp%d: "


static bool resolve_address_tcp(io_service &io, int chan, std::string host, unsigned short port, tcp::endpoint &ep)
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
			logDebug(PFXd "host %s resolved as %s", chan, host.c_str(), to_string_ss(ep).c_str());
		});

	if (ec) {
		logWarn(PFXd "resolve error: %s", chan, ec.message().c_str());
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
	if (!resolve_address_tcp(io_service, channel, server_host, server_port, server_ep))
		throw DeviceError("tcp: resolve", "Bind address resolve failed");

	logInform(PFXd "Server address: %s", channel, to_string_ss(server_ep).c_str());

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

MAVConnTCPClient::MAVConnTCPClient(uint8_t system_id, uint8_t component_id,
		boost::asio::io_service &server_io) :
	MAVConnInterface(system_id, component_id),
	socket(server_io)
{
	// waiting when server call client_connected()
}

void MAVConnTCPClient::client_connected(int server_channel) {
	logInform(PFXd "Got client, channel: %d, address: %s",
			server_channel, channel, to_string_ss(server_ep).c_str());

	// start recv
	socket.get_io_service().post(boost::bind(&MAVConnTCPClient::do_recv, this));
}

MAVConnTCPClient::~MAVConnTCPClient() {
	close();
}

void MAVConnTCPClient::close() {
	lock_guard lock(mutex);
	if (!is_open())
		return;

	io_work.reset();
	io_service.stop();
	socket.close();

	// clear tx queue
	for (auto &p : tx_q)
		delete p;
	tx_q.clear();

	if (io_thread.joinable())
		io_thread.join();

	/* emit */ port_closed();
}

void MAVConnTCPClient::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!is_open()) {
		logError(PFXd "send: channel closed!", channel);
		return;
	}

	MsgBuffer *buf = new MsgBuffer(bytes, length);
	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
	}
	socket.get_io_service().post(boost::bind(&MAVConnTCPClient::do_send, this, true));
}

void MAVConnTCPClient::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	assert(message != nullptr);

	if (!is_open()) {
		logError(PFXd "send: channel closed!", channel);
		return;
	}

	logDebug(PFXd "send: Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d Seq: %d",
			channel, message->msgid, message->len, sysid, compid, message->seq);

	MsgBuffer *buf = new_msgbuffer(message, sysid, compid);
	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
	}
	socket.get_io_service().post(boost::bind(&MAVConnTCPClient::do_send, this, true));
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
		logError(PFXd "receive: %s", channel, error.message().c_str());
		close();
		return;
	}

	iostat_rx_add(bytes_transferred);
	for (size_t i = 0; i < bytes_transferred; i++) {
		if (mavlink_parse_char(channel, rx_buf[i], &message, &status)) {
			logDebug(PFXd "recv: Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d Seq: %d",
					channel, message.msgid, message.len, message.sysid, message.compid, message.seq);

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

	tx_in_progress = true;
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
		logError(PFXd "send: %s", channel, error.message().c_str());
		close();
		return;
	}

	iostat_tx_add(bytes_transferred);
	lock_guard lock(mutex);
	if (tx_q.empty()) {
		tx_in_progress = false;
		return;
	}

	MsgBuffer *buf = tx_q.front();
	buf->pos += bytes_transferred;
	if (buf->nbytes() == 0) {
		tx_q.pop_front();
		delete buf;
	}

	if (!tx_q.empty())
		do_send(false);
	else
		tx_in_progress = false;
}


/* -*- TCP server variant -*- */

MAVConnTCPServer::MAVConnTCPServer(uint8_t system_id, uint8_t component_id,
		std::string server_host, unsigned short server_port) :
	MAVConnInterface(system_id, component_id),
	io_service(),
	acceptor(io_service)
{
	if (!resolve_address_tcp(io_service, channel, server_host, server_port, bind_ep))
		throw DeviceError("tcp-l: resolve", "Bind address resolve failed");

	logInform(PFXd "Bind address: %s", channel, to_string_ss(bind_ep).c_str());

	try {
		acceptor.open(tcp::v4());
		acceptor.set_option(tcp::acceptor::reuse_address(true));
		acceptor.bind(bind_ep);
		acceptor.listen(channes_available());
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("tcp-l", err);
	}

	// give some work to io_service before start
	io_service.post(boost::bind(&MAVConnTCPServer::do_accept, this));

	// run io_service for async io
	std::thread t(boost::bind(&io_service::run, &this->io_service));
	mavutils::set_thread_name(t, "MAVConnTCPs%d", channel);
	io_thread.swap(t);
}

MAVConnTCPServer::~MAVConnTCPServer() {
	close();
}

void MAVConnTCPServer::close() {
	lock_guard lock(mutex);
	if (!is_open())
		return;

	logInform(PFXd "Terminating server. "
			"All connections will be closed.", channel);

	io_service.stop();
	acceptor.close();

	if (io_thread.joinable())
		io_thread.join();

	/* emit */ port_closed();
}

mavlink_status_t MAVConnTCPServer::get_status()
{
	mavlink_status_t status{};

	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		auto inst_status = instp->get_status();

#define ADD_STATUS(_field)	\
		status._field += inst_status._field

		ADD_STATUS(packet_rx_success_count);
		ADD_STATUS(packet_rx_drop_count);
		ADD_STATUS(buffer_overrun);
		ADD_STATUS(parse_error);
		/* seq counters always 0 for this connection type */

#undef ADD_STATUS
	};

	return status;
}

MAVConnInterface::IOStat MAVConnTCPServer::get_iostat()
{
	MAVConnInterface::IOStat iostat{};

	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		auto inst_iostat = instp->get_iostat();

#define ADD_IOSTAT(_field)	\
		iostat._field += inst_iostat._field

		ADD_IOSTAT(tx_total_bytes);
		ADD_IOSTAT(rx_total_bytes);
		ADD_IOSTAT(tx_speed);
		ADD_IOSTAT(rx_speed);

#undef ADD_IOSTAT
	};

	return iostat;
}

void MAVConnTCPServer::send_bytes(const uint8_t *bytes, size_t length)
{
	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		instp->send_bytes(bytes, length);
	}
}

void MAVConnTCPServer::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		instp->send_message(message, sysid, compid);
	}
}

void MAVConnTCPServer::do_accept()
{
	acceptor_client.reset();
	acceptor_client = boost::make_shared<MAVConnTCPClient>(sys_id, comp_id, io_service);
	acceptor.async_accept(
			acceptor_client->socket,
			acceptor_client->server_ep,
			boost::bind(&MAVConnTCPServer::async_accept_end,
				this,
				boost::asio::placeholders::error));
}

void MAVConnTCPServer::async_accept_end(error_code error)
{
	if (error) {
		logError(PFXd "accept: %s", channel, error.message().c_str());
		close();
		return;
	}

	// NOTE: i want create client class *after* connection accept,
	//       but ASIO 1.43 does not support std::move() for sockets.
	//       Need find way how to limit channel alloc.
	//if (channes_available() <= 0) {
	//	ROS_ERROR_NAMED("mavconn", "tcp-l:accept_cb: all channels in use, drop connection");
	//	client_sock.close();
	//	return;
	//}

	lock_guard lock(mutex);
	acceptor_client->client_connected(channel);
	acceptor_client->message_received.connect(boost::bind(&MAVConnTCPServer::recv_message, this, _1, _2, _3));
	acceptor_client->port_closed.connect(boost::bind(&MAVConnTCPServer::client_closed, this,
				boost::weak_ptr<MAVConnTCPClient>(acceptor_client)));

	client_list.push_back(acceptor_client);
	do_accept();
}

void MAVConnTCPServer::client_closed(boost::weak_ptr<MAVConnTCPClient> weak_instp)
{
	if (auto instp = weak_instp.lock()) {
		bool locked = mutex.try_lock();
		logInform(PFXd "Client connection closed, channel: %d, address: %s",
				channel, instp->channel, to_string_ss(instp->server_ep).c_str());

		client_list.remove(instp);

		if (locked)
			mutex.unlock();
	}
}

void MAVConnTCPServer::recv_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	/* retranslate message */
	message_received(message, sysid, compid);
}

}; // namespace mavconn
