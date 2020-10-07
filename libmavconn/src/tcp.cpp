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
 * Copyright 2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <cassert>

#include <mavconn/console_bridge_compat.h>
#include <mavconn/thread_utils.h>
#include <mavconn/tcp.h>

// Ensure the correct io_service() is called based on boost version
#if BOOST_VERSION >= 107000
#define GET_IO_SERVICE(s) ((boost::asio::io_context&)(s).get_executor().context())
#else
#define GET_IO_SERVICE(s) ((s).get_io_service())
#endif

namespace mavconn {

using boost::system::error_code;
using boost::asio::io_service;
using boost::asio::ip::tcp;
using boost::asio::buffer;
using utils::to_string_ss;
using mavlink::mavlink_message_t;
using mavlink::mavlink_status_t;

#define PFX	"mavconn: tcp"
#define PFXd	PFX "%zu: "


static bool resolve_address_tcp(io_service &io, size_t chan, std::string host, unsigned short port, tcp::endpoint &ep)
{
	bool result = false;
	tcp::resolver resolver(io);
	error_code ec;

	tcp::resolver::query query(host, "");

	auto fn = [&](const tcp::endpoint & q_ep) {
		ep = q_ep;
		ep.port(port);
		result = true;
		CONSOLE_BRIDGE_logDebug(PFXd "host %s resolved as %s", chan, host.c_str(), to_string_ss(ep).c_str());
	};

#if BOOST_ASIO_VERSION >= 101200
	for (auto q_ep : resolver.resolve(query, ec)) fn(q_ep);
#else
	std::for_each(resolver.resolve(query, ec), tcp::resolver::iterator(), fn);
#endif

	if (ec) {
		CONSOLE_BRIDGE_logWarn(PFXd "resolve error: %s", chan, ec.message().c_str());
		result = false;
	}

	return result;
}


/* -*- TCP client variant -*- */

MAVConnTCPClient::MAVConnTCPClient(uint8_t system_id, uint8_t component_id,
		std::string server_host, unsigned short server_port) :
	MAVConnInterface(system_id, component_id),
	io_service(),
	io_work(new io_service::work(io_service)),
	socket(io_service),
	is_destroying(false),
	tx_in_progress(false),
	tx_q {},
	rx_buf {}
{
	if (!resolve_address_tcp(io_service, conn_id, server_host, server_port, server_ep))
		throw DeviceError("tcp: resolve", "Bind address resolve failed");

	CONSOLE_BRIDGE_logInform(PFXd "Server address: %s", conn_id, to_string_ss(server_ep).c_str());

	try {
		socket.open(tcp::v4());
		socket.connect(server_ep);
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("tcp", err);
	}

	// NOTE: shared_from_this() should not be used in constructors

	// give some work to io_service before start
	io_service.post(std::bind(&MAVConnTCPClient::do_recv, this));

	// run io_service for async io
	io_thread = std::thread([this] () {
				utils::set_this_thread_name("mtcp%zu", conn_id);
				io_service.run();
			});
}

MAVConnTCPClient::MAVConnTCPClient(uint8_t system_id, uint8_t component_id,
		boost::asio::io_service &server_io) :
	MAVConnInterface(system_id, component_id),
	socket(server_io),
	tx_in_progress(false),
	tx_q {},
	rx_buf {}
{
	// waiting when server call client_connected()
}

void MAVConnTCPClient::client_connected(size_t server_channel)
{
	CONSOLE_BRIDGE_logInform(PFXd "Got client, id: %zu, address: %s",
			server_channel, conn_id, to_string_ss(server_ep).c_str());

	// start recv
	GET_IO_SERVICE(socket).post(std::bind(&MAVConnTCPClient::do_recv, shared_from_this()));
}

MAVConnTCPClient::~MAVConnTCPClient()
{
	is_destroying = true;
	close();
}

void MAVConnTCPClient::close()
{
	lock_guard lock(mutex);
	if (!is_open())
		return;

	socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send);
	socket.cancel();
	socket.close();

	io_work.reset();
	io_service.stop();

	if (io_thread.joinable())
		io_thread.join();

	io_service.reset();

	if (port_closed_cb)
		port_closed_cb();
}

void MAVConnTCPClient::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnTCPClient::send_bytes: TX queue overflow");

		tx_q.emplace_back(bytes, length);
	}
	GET_IO_SERVICE(socket).post(std::bind(&MAVConnTCPClient::do_send, shared_from_this(), true));
}

void MAVConnTCPClient::send_message(const mavlink_message_t *message)
{
	assert(message != nullptr);

	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	log_send(PFX, message);

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnTCPClient::send_message: TX queue overflow");

		tx_q.emplace_back(message);
	}
	GET_IO_SERVICE(socket).post(std::bind(&MAVConnTCPClient::do_send, shared_from_this(), true));
}

void MAVConnTCPClient::send_message(const mavlink::Message &message, const uint8_t source_compid)
{
	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	log_send_obj(PFX, message);

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnTCPClient::send_message: TX queue overflow");

		tx_q.emplace_back(message, get_status_p(), sys_id, source_compid);
	}
	GET_IO_SERVICE(socket).post(std::bind(&MAVConnTCPClient::do_send, shared_from_this(), true));
}

void MAVConnTCPClient::do_recv()
{
	if (is_destroying) {
		return;
	}
	auto sthis = shared_from_this();
	socket.async_receive(
			buffer(rx_buf),
			[sthis] (error_code error, size_t bytes_transferred) {
				if (error) {
					CONSOLE_BRIDGE_logError(PFXd "receive: %s", sthis->conn_id, error.message().c_str());
					sthis->close();
					return;
				}

				sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
				sthis->do_recv();
			});
}

void MAVConnTCPClient::do_send(bool check_tx_state)
{
	if (check_tx_state && tx_in_progress)
		return;

	lock_guard lock(mutex);
	if (tx_q.empty())
		return;

	tx_in_progress = true;
	auto sthis = shared_from_this();
	auto &buf_ref = tx_q.front();
	socket.async_send(
			buffer(buf_ref.dpos(), buf_ref.nbytes()),
			[sthis, &buf_ref] (error_code error, size_t bytes_transferred) {
				assert(bytes_transferred <= buf_ref.len);

				if (error) {
					CONSOLE_BRIDGE_logError(PFXd "send: %s", sthis->conn_id, error.message().c_str());
					sthis->close();
					return;
				}

				sthis->iostat_tx_add(bytes_transferred);
				lock_guard lock(sthis->mutex);

				if (sthis->tx_q.empty()) {
					sthis->tx_in_progress = false;
					return;
				}

				buf_ref.pos += bytes_transferred;
				if (buf_ref.nbytes() == 0) {
					sthis->tx_q.pop_front();
				}

				if (!sthis->tx_q.empty())
					sthis->do_send(false);
				else
					sthis->tx_in_progress = false;
			});
}


/* -*- TCP server variant -*- */

MAVConnTCPServer::MAVConnTCPServer(uint8_t system_id, uint8_t component_id,
		std::string server_host, unsigned short server_port) :
	MAVConnInterface(system_id, component_id),
	io_service(),
	acceptor(io_service),
	is_destroying(false)
{
	if (!resolve_address_tcp(io_service, conn_id, server_host, server_port, bind_ep))
		throw DeviceError("tcp-l: resolve", "Bind address resolve failed");

	CONSOLE_BRIDGE_logInform(PFXd "Bind address: %s", conn_id, to_string_ss(bind_ep).c_str());

	try {
		acceptor.open(tcp::v4());
		acceptor.set_option(tcp::acceptor::reuse_address(true));
		acceptor.bind(bind_ep);
		acceptor.listen();
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("tcp-l", err);
	}

	// give some work to io_service before start
	io_service.post(std::bind(&MAVConnTCPServer::do_accept, this));

	// run io_service for async io
	io_thread = std::thread([this] () {
				utils::set_this_thread_name("mtcps%zu", conn_id);
				io_service.run();
			});
}

MAVConnTCPServer::~MAVConnTCPServer()
{
	is_destroying = true;
	close();
}

void MAVConnTCPServer::close()
{
	lock_guard lock(mutex);
	if (!is_open())
		return;

	CONSOLE_BRIDGE_logInform(PFXd "Terminating server. "
			"All connections will be closed.", conn_id);

	io_service.stop();
	acceptor.close();

	if (io_thread.joinable())
		io_thread.join();

	if (port_closed_cb)
		port_closed_cb();
}

mavlink_status_t MAVConnTCPServer::get_status()
{
	mavlink_status_t status {};

	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		auto inst_status = instp->get_status();

		// [[[cog:
		// for f in ('packet_rx_success_count', 'packet_rx_drop_count', 'buffer_overrun', 'parse_error'):
		//     cog.outl("status.{f:23s} += inst_status.{f};".format(**locals()))
		// ]]]
		status.packet_rx_success_count += inst_status.packet_rx_success_count;
		status.packet_rx_drop_count    += inst_status.packet_rx_drop_count;
		status.buffer_overrun          += inst_status.buffer_overrun;
		status.parse_error             += inst_status.parse_error;
		// [[[end]]] (checksum: a6186246ed026f1cf2b4ffc7407e893b)

		/* seq counters always 0 for this connection type */
	}

	return status;
}

MAVConnInterface::IOStat MAVConnTCPServer::get_iostat()
{
	MAVConnInterface::IOStat iostat {};

	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		auto inst_iostat = instp->get_iostat();

		// [[[cog:
		// for p in ('tx', 'rx'):
		//     for f in ('total_bytes', 'speed'):
		//         cog.outl("iostat.{p}_{f:11s} += inst_iostat.{p}_{f};".format(**locals()))
		// ]]]
		iostat.tx_total_bytes += inst_iostat.tx_total_bytes;
		iostat.tx_speed       += inst_iostat.tx_speed;
		iostat.rx_total_bytes += inst_iostat.rx_total_bytes;
		iostat.rx_speed       += inst_iostat.rx_speed;
		// [[[end]]] (checksum: fb4fe06794471d9b068ce0c129ee7673)
	}

	return iostat;
}

void MAVConnTCPServer::send_bytes(const uint8_t *bytes, size_t length)
{
	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		instp->send_bytes(bytes, length);
	}
}

void MAVConnTCPServer::send_message(const mavlink_message_t *message)
{
	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		instp->send_message(message);
	}
}

void MAVConnTCPServer::send_message(const mavlink::Message &message, const uint8_t source_compid)
{
	lock_guard lock(mutex);
	for (auto &instp : client_list) {
		instp->send_message(message, source_compid);
	}
}

void MAVConnTCPServer::do_accept()
{
	if (is_destroying) {
		return;
	}
	auto sthis = shared_from_this();
	auto acceptor_client = std::make_shared<MAVConnTCPClient>(sys_id, comp_id, io_service);
	acceptor.async_accept(
			acceptor_client->socket,
			acceptor_client->server_ep,
			[sthis, acceptor_client] (error_code error) {
				if (error) {
					CONSOLE_BRIDGE_logError(PFXd "accept: %s", sthis->conn_id, error.message().c_str());
					sthis->close();
					return;
				}

				lock_guard lock(sthis->mutex);

				std::weak_ptr<MAVConnTCPClient> weak_client{acceptor_client};
				acceptor_client->client_connected(sthis->conn_id);
				acceptor_client->message_received_cb = std::bind(&MAVConnTCPServer::recv_message, sthis, std::placeholders::_1, std::placeholders::_2);
				acceptor_client->port_closed_cb = [weak_client, sthis] () { sthis->client_closed(weak_client); };

				sthis->client_list.push_back(acceptor_client);
				sthis->do_accept();
			});
}

void MAVConnTCPServer::client_closed(std::weak_ptr<MAVConnTCPClient> weak_instp)
{
	if (auto instp = weak_instp.lock()) {
		bool locked = mutex.try_lock();
		CONSOLE_BRIDGE_logInform(PFXd "Client connection closed, id: %p, address: %s",
				conn_id, instp.get(), to_string_ss(instp->server_ep).c_str());

		client_list.remove(instp);

		if (locked)
			mutex.unlock();
	}
}

void MAVConnTCPServer::recv_message(const mavlink_message_t *message, const Framing framing)
{
	if (message_received_cb)
		message_received_cb(message, framing);
}
}	// namespace mavconn
