/**
 * @brief MAVConn UDP link class
 * @file udp.cpp
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

#include <cassert>

#include <mavconn/console_bridge_compat.h>
#include <mavconn/thread_utils.h>
#include <mavconn/udp.h>

namespace mavconn {

using boost::system::error_code;
using boost::asio::io_service;
using boost::asio::ip::udp;
using boost::asio::buffer;
using utils::to_string_ss;
using utils::operator"" _KiB;
using mavlink::mavlink_message_t;

#define PFX	"mavconn: udp"
#define PFXd	PFX "%zu: "


static bool resolve_address_udp(io_service &io, size_t chan, std::string host, unsigned short port, udp::endpoint &ep)
{
	bool result = false;
	udp::resolver resolver(io);
	error_code ec;

	udp::resolver::query query(host, "");

	auto fn = [&](const udp::endpoint & q_ep) {
		ep = q_ep;
		ep.port(port);
		result = true;
		CONSOLE_BRIDGE_logDebug(PFXd "host %s resolved as %s", chan, host.c_str(), to_string_ss(ep).c_str());
	};

#if BOOST_ASIO_VERSION >= 101200
	for (auto q_ep : resolver.resolve(query, ec)) fn(q_ep);
#else
	std::for_each(resolver.resolve(query, ec), udp::resolver::iterator(), fn);
#endif

	if (ec) {
		CONSOLE_BRIDGE_logWarn(PFXd "resolve error: %s", chan, ec.message().c_str());
		result = false;
	}

	return result;
}


MAVConnUDP::MAVConnUDP(uint8_t system_id, uint8_t component_id,
		std::string bind_host, unsigned short bind_port,
		std::string remote_host, unsigned short remote_port) :
	MAVConnInterface(system_id, component_id),
	io_service(),
	io_work(new io_service::work(io_service)),
	permanent_broadcast(false),
	remote_exists(false),
	socket(io_service),
	tx_in_progress(false),
	tx_q {},
	rx_buf {}
{
	using udps = boost::asio::ip::udp::socket;

	if (!resolve_address_udp(io_service, conn_id, bind_host, bind_port, bind_ep))
		throw DeviceError("udp: resolve", "Bind address resolve failed");

	CONSOLE_BRIDGE_logInform(PFXd "Bind address: %s", conn_id, to_string_ss(bind_ep).c_str());

	if (remote_host != "") {
		if (remote_host != BROADCAST_REMOTE_HOST && remote_host != PERMANENT_BROADCAST_REMOTE_HOST)
			remote_exists = resolve_address_udp(io_service, conn_id, remote_host, remote_port, remote_ep);
		else {
			remote_exists = true;
			remote_ep = udp::endpoint(boost::asio::ip::address_v4::broadcast(), remote_port);
		}

		if (remote_exists)
			CONSOLE_BRIDGE_logInform(PFXd "Remote address: %s", conn_id, to_string_ss(remote_ep).c_str());
		else
			CONSOLE_BRIDGE_logWarn(PFXd "Remote address resolve failed.", conn_id);
	}

	try {
		socket.open(udp::v4());

		// set buffer opt. size from QGC
		socket.set_option(udps::reuse_address(true));
		socket.set_option(udps::send_buffer_size(256_KiB));
		socket.set_option(udps::receive_buffer_size(512_KiB));

		socket.bind(bind_ep);

		if (remote_host == BROADCAST_REMOTE_HOST) {
			socket.set_option(udps::broadcast(true));
		} else if (remote_host == PERMANENT_BROADCAST_REMOTE_HOST) {
			socket.set_option(udps::broadcast(true));
			permanent_broadcast = true;
		}
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("udp", err);
	}

	// NOTE: shared_from_this() should not be used in constructors

	// give some work to io_service before start
	io_service.post(std::bind(&MAVConnUDP::do_recvfrom, this));

	// run io_service for async io
	io_thread = std::thread([this] () {
				utils::set_this_thread_name("mudp%zu", conn_id);
				io_service.run();
			});
}

MAVConnUDP::~MAVConnUDP()
{
	close();
}

void MAVConnUDP::close()
{
	lock_guard lock(mutex);
	if (!is_open())
		return;

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

void MAVConnUDP::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	if (!remote_exists) {
		CONSOLE_BRIDGE_logDebug(PFXd "send: Remote not known, message dropped.", conn_id);
		return;
	}

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnUDP::send_bytes: TX queue overflow");

		tx_q.emplace_back(bytes, length);
	}
	io_service.post(std::bind(&MAVConnUDP::do_sendto, shared_from_this(), true));
}

void MAVConnUDP::send_message(const mavlink_message_t *message)
{
	assert(message != nullptr);

	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	if (!remote_exists) {
		CONSOLE_BRIDGE_logDebug(PFXd "send: Remote not known, message dropped.", conn_id);
		return;
	}

	log_send(PFX, message);

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnUDP::send_message: TX queue overflow");

		tx_q.emplace_back(message);
	}
	io_service.post(std::bind(&MAVConnUDP::do_sendto, shared_from_this(), true));
}

void MAVConnUDP::send_message(const mavlink::Message &message, const uint8_t source_compid)
{
	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	if (!remote_exists) {
		CONSOLE_BRIDGE_logDebug(PFXd "send: Remote not known, message dropped.", conn_id);
		return;
	}

	log_send_obj(PFX, message);

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnUDP::send_message: TX queue overflow");

		tx_q.emplace_back(message, get_status_p(), sys_id, source_compid);
	}
	io_service.post(std::bind(&MAVConnUDP::do_sendto, shared_from_this(), true));
}

void MAVConnUDP::do_recvfrom()
{
	auto sthis = shared_from_this();
	socket.async_receive_from(
			buffer(rx_buf),
			permanent_broadcast ? recv_ep : remote_ep,
			[sthis] (error_code error, size_t bytes_transferred) {
				if (error) {
					CONSOLE_BRIDGE_logError(PFXd "receive: %s", sthis->conn_id, error.message().c_str());
					sthis->close();
					return;
				}

				if (!sthis->permanent_broadcast && sthis->remote_ep != sthis->last_remote_ep) {
					CONSOLE_BRIDGE_logInform(PFXd "Remote address: %s", sthis->conn_id, to_string_ss(sthis->remote_ep).c_str());
					sthis->remote_exists = true;
					sthis->last_remote_ep = sthis->remote_ep;
				}

				sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
				sthis->do_recvfrom();
			});
}

void MAVConnUDP::do_sendto(bool check_tx_state)
{
	if (check_tx_state && tx_in_progress)
		return;

	lock_guard lock(mutex);
	if (tx_q.empty())
		return;

	tx_in_progress = true;
	auto sthis = shared_from_this();
	auto &buf_ref = tx_q.front();
	socket.async_send_to(
			buffer(buf_ref.dpos(), buf_ref.nbytes()),
			remote_ep,
			[sthis, &buf_ref] (error_code error, size_t bytes_transferred) {
				assert(bytes_transferred <= buf_ref.len);

				if (error == boost::asio::error::network_unreachable) {
					CONSOLE_BRIDGE_logWarn(PFXd "sendto: %s, retrying", sthis->conn_id, error.message().c_str());
					// do not return, try to resend
				}
				else if (error) {
					CONSOLE_BRIDGE_logError(PFXd "sendto: %s", sthis->conn_id, error.message().c_str());
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
					sthis->do_sendto(false);
				else
					sthis->tx_in_progress = false;
			});
}

std::string MAVConnUDP::get_remote_endpoint() const
{
	return to_string_ss(remote_ep);
}

}	// namespace mavconn
