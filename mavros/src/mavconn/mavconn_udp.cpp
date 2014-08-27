/**
 * @brief MAVConn UDP link class
 * @file mavconn_udp.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
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
#include <mavros/mavconn_udp.h>
#include <ros/console.h>
#include <ros/assert.h>

namespace mavconn {
using boost::system::error_code;
using boost::asio::io_service;
using boost::asio::ip::udp;
using boost::asio::buffer;
typedef std::lock_guard<std::recursive_mutex> lock_guard;


static bool resolve_address_udp(io_service &io, std::string host, unsigned short port, udp::endpoint &ep)
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
			ROS_DEBUG_STREAM_NAMED("mavconn", "udp: host " << host << " resolved as " << ep);
		});

	if (ec) {
		ROS_WARN_STREAM_NAMED("mavconn", "udp: resolve error: " << ec.message());
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
	if (!resolve_address_udp(io_service, bind_host, bind_port, bind_ep))
		throw DeviceError("udp: resolve", "Bind address resolve failed");

	ROS_INFO_STREAM_NAMED("mavconn", "udp" << channel << ": Bind address: " << bind_ep);

	if (remote_host != "") {
		remote_exists = resolve_address_udp(io_service, remote_host, remote_port, remote_ep);

		if (remote_exists)
			ROS_INFO_STREAM_NAMED("mavconn", "udp" << channel << ": Remote address: " << remote_ep);
		else
			ROS_WARN_NAMED("mavconn", "udp%d: Remote address resolve failed.", channel);
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
	std::for_each(tx_q.begin(), tx_q.end(),
			[](MsgBuffer *p) { delete p; });
	tx_q.clear();

	if (io_thread.joinable())
		io_thread.join();

	/* emit */ port_closed();
}

void MAVConnUDP::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!is_open()) {
		ROS_ERROR_THROTTLE_NAMED(10, "mavconn", "udp%d:send: channel closed!", channel);
		return;
	}

	if (!remote_exists) {
		ROS_DEBUG_NAMED("mavconn", "udp%d:send:: Remote not known, message dropped.", channel);
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
	ROS_ASSERT(message != nullptr);

	if (!is_open()) {
		ROS_ERROR_THROTTLE_NAMED(10, "mavconn", "udp%d:send: channel closed!", channel);
		return;
	}

	if (!remote_exists) {
		ROS_DEBUG_NAMED("mavconn", "udp%d:send: Remote not known, message dropped.", channel);
		return;
	}

	ROS_DEBUG_NAMED("mavconn", "udp%d:send: Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d",
			channel, message->msgid, message->len, sysid, compid);

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
		ROS_ERROR_STREAM_NAMED("mavconn", "udp" << channel << ":receive: " << error.message());
		close();
		return;
	}

	if (remote_ep != last_remote_ep) {
		ROS_INFO_STREAM_NAMED("mavconn", "udp" << channel << ": Remote address: " << remote_ep);
		remote_exists = true;
		last_remote_ep = remote_ep;
	}

	for (ssize_t i = 0; i < bytes_transferred; i++) {
		if (mavlink_parse_char(channel, rx_buf[i], &message, &status)) {
			ROS_DEBUG_NAMED("mavconn", "udp%d:recv: Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d",
					channel, message.msgid, message.len, message.sysid, message.compid);

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
	if (error) {
		ROS_ERROR_STREAM_NAMED("mavconn", "udp" << channel << ":sendto: " << error.message());
		close();
		return;
	}

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
