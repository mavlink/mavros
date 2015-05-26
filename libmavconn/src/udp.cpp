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
 * Copyright 2013,2014,2015 Vladimir Ermakov, All rights reserved.
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
using mavutils::to_string_ss;
typedef std::lock_guard<std::recursive_mutex> lock_guard;

#define PFXd	"mavconn: udp%d: "


static bool resolve_address_udp(io_service &io, int chan, std::string host, unsigned short port, udp::endpoint &ep)
{
	bool result = false;
	udp::resolver resolver(io);
	error_code ec;

	udp::resolver::query query(host, "");
	std::for_each(resolver.resolve(query, ec), udp::resolver::iterator(),
		[&](const udp::endpoint &q_ep) {
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
	if (!resolve_address_udp(io_service, channel, bind_host, bind_port, bind_ep))
		throw DeviceError("udp: resolve", "Bind address resolve failed");

	logInform(PFXd "Bind address: %s", channel, to_string_ss(bind_ep).c_str());

	if (remote_host != "") {
		remote_exists = resolve_address_udp(io_service, channel, remote_host, remote_port, remote_ep);

		if (remote_exists)
			logInform(PFXd "Remote address: %s", channel, to_string_ss(remote_ep).c_str());
		else
			logWarn(PFXd "Remote address resolve failed.", channel);
	}

	try {
		socket.open(udp::v4());
		socket.bind(bind_ep);
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("udp", err);
	}

	// give some work to io_service before start
	io_service.post(boost::bind(&MAVConnUDP::do_recvfrom, this));

	// run io_service for async io
	std::thread t(boost::bind(&io_service::run, &this->io_service));
	mavutils::set_thread_name(t, "MAVConnUDP%d", channel);
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

	/* emit */ port_closed();
}

void MAVConnUDP::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!is_open()) {
		logError(PFXd "send: channel closed!", channel);
		return;
	}

	if (!remote_exists) {
		logDebug(PFXd "send: Remote not known, message dropped.", channel);
		return;
	}

	MsgBuffer *buf = new MsgBuffer(bytes, length);
	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
	}
	io_service.post(boost::bind(&MAVConnUDP::do_sendto, this, true));
}

void MAVConnUDP::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	assert(message != nullptr);

	if (!is_open()) {
		logError(PFXd "send: channel closed!", channel);
		return;
	}

	if (!remote_exists) {
		logDebug(PFXd "send: Remote not known, message dropped.", channel);
		return;
	}

	logDebug(PFXd "send: Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d Seq: %d",
			channel, message->msgid, message->len, sysid, compid, message->seq);

	MsgBuffer *buf = new_msgbuffer(message, sysid, compid);
	{
		lock_guard lock(mutex);
		tx_q.push_back(buf);
	}
	io_service.post(boost::bind(&MAVConnUDP::do_sendto, this, true));
}

void MAVConnUDP::do_recvfrom()
{
	socket.async_receive_from(
			buffer(rx_buf, sizeof(rx_buf)),
			remote_ep,
			boost::bind(&MAVConnUDP::async_receive_end,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
}

void MAVConnUDP::async_receive_end(error_code error, size_t bytes_transferred)
{
	mavlink_message_t message;
	mavlink_status_t status;

	if (error) {
		logError(PFXd "receive: %s", channel, error.message().c_str());
		close();
		return;
	}

	if (remote_ep != last_remote_ep) {
		logInform(PFXd "Remote address: %s", channel, to_string_ss(remote_ep).c_str());
		remote_exists = true;
		last_remote_ep = remote_ep;
	}

	iostat_rx_add(bytes_transferred);
	for (size_t i = 0; i < bytes_transferred; i++) {
		if (mavlink_parse_char(channel, rx_buf[i], &message, &status)) {
			logDebug(PFXd "recv: Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d Seq: %d",
					channel, message.msgid, message.len, message.sysid, message.compid, message.seq);

			/* emit */ message_received(&message, message.sysid, message.compid);
		}
	}

	do_recvfrom();
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
			boost::bind(&MAVConnUDP::async_sendto_end,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
}

void MAVConnUDP::async_sendto_end(error_code error, size_t bytes_transferred)
{
	if (error == boost::asio::error::network_unreachable) {
		logWarn(PFXd "sendto: %s, retrying", channel, error.message().c_str());
		// do not return, try to resend
	}
	else if (error) {
		logError(PFXd "sendto: %s", channel, error.message().c_str());
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
}

}; // namespace mavconn
