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
#include <console_bridge/console.h>

#include <mavconn/thread_utils.h>
#include <mavconn/udp.h>

namespace mavconn {
using boost::system::error_code;
using boost::asio::io_service;
using boost::asio::ip::udp;
using boost::asio::buffer;
using utils::to_string_ss;
using mavlink::mavlink_message_t;

#define PFX	"mavconn: udp"
#define PFXd	PFX "%p: "


static bool resolve_address_udp(io_service &io, void *chan, std::string host, unsigned short port, udp::endpoint &ep)
{
	bool result = false;
	udp::resolver resolver(io);
	error_code ec;

	udp::resolver::query query(host, "");
	std::for_each(resolver.resolve(query, ec), udp::resolver::iterator(),
			[&](const udp::endpoint & q_ep) {
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


MAVConnUDP::MAVConnUDP(uint8_t system_id, uint8_t component_id,
		std::string bind_host, unsigned short bind_port,
		std::string remote_host, unsigned short remote_port) :
	MAVConnInterface(system_id, component_id),
	remote_exists(false),
	tx_in_progress(false),
	io_service(),
	io_work(new io_service::work(io_service)),
	socket(io_service)
{
	if (!resolve_address_udp(io_service, this, bind_host, bind_port, bind_ep))
		throw DeviceError("udp: resolve", "Bind address resolve failed");

	logInform(PFXd "Bind address: %s", this, to_string_ss(bind_ep).c_str());

	if (remote_host != "") {
		remote_exists = resolve_address_udp(io_service, this, remote_host, remote_port, remote_ep);

		if (remote_exists)
			logInform(PFXd "Remote address: %s", this, to_string_ss(remote_ep).c_str());
		else
			logWarn(PFXd "Remote address resolve failed.", this);
	}

	try {
		socket.open(udp::v4());
		socket.bind(bind_ep);
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("udp", err);
	}

	// give some work to io_service before start
	io_service.post(std::bind(&MAVConnUDP::do_recvfrom, this));

	// run io_service for async io
	std::thread t([&] () {
				utils::set_this_thread_name("mudp%p", this);
				io_service.run();
			});
	io_thread.swap(t);
}

MAVConnUDP::~MAVConnUDP() {
	close();
}

void MAVConnUDP::close() {
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

	port_closed.emit();
}

void MAVConnUDP::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!is_open()) {
		logError(PFXd "send: channel closed!", this);
		return;
	}

	if (!remote_exists) {
		logDebug(PFXd "send: Remote not known, message dropped.", this);
		return;
	}

	MsgBuffer *buf = new MsgBuffer(bytes, length);
	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
	}
	io_service.post(std::bind(&MAVConnUDP::do_sendto, this, true));
}

void MAVConnUDP::send_message(const mavlink_message_t *message)
{
	assert(message != nullptr);

	if (!is_open()) {
		logError(PFXd "send: channel closed!", this);
		return;
	}

	if (!remote_exists) {
		logDebug(PFXd "send: Remote not known, message dropped.", this);
		return;
	}

	log_send(PFX, message);

	MsgBuffer *buf = new_msgbuffer(message);
	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
	}
	io_service.post(std::bind(&MAVConnUDP::do_sendto, this, true));
}

void MAVConnUDP::send_message(const mavlink::Message &message)
{
	if (!is_open()) {
		logError(PFXd "send: channel closed!", this);
		return;
	}

	if (!remote_exists) {
		logDebug(PFXd "send: Remote not known, message dropped.", this);
		return;
	}

	log_send_obj(PFX, message);

	// XXX decide later: locked or not
	MsgBuffer *buf = new_msgbuffer(message);
	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
	}
	io_service.post(std::bind(&MAVConnUDP::do_sendto, this, true));
}

void MAVConnUDP::do_recvfrom()
{
	socket.async_receive_from(
			buffer(rx_buf, sizeof(rx_buf)),
			remote_ep,
			[&] (error_code error, size_t bytes_transferred) {
				if (error) {
					logError(PFXd "receive: %s", this, error.message().c_str());
					close();
					return;
				}

				if (remote_ep != last_remote_ep) {
					logInform(PFXd "Remote address: %s", this, to_string_ss(remote_ep).c_str());
					remote_exists = true;
					last_remote_ep = remote_ep;
				}

				parse_buffer(PFX, rx_buf, sizeof(rx_buf), bytes_transferred);
				do_recvfrom();
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
	MsgBuffer *buf = tx_q.front();
	socket.async_send_to(
			buffer(buf->dpos(), buf->nbytes()),
			remote_ep,
			[&] (error_code error, size_t bytes_transferred) {
				if (error == boost::asio::error::network_unreachable) {
					logWarn(PFXd "sendto: %s, retrying", this, error.message().c_str());
					// do not return, try to resend
				}
				else if (error) {
					logError(PFXd "sendto: %s", this, error.message().c_str());
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
					do_sendto(false);
				else
					tx_in_progress = false;
			});
}
}	// namespace mavconn
