/**
 * @file mavconn_udp.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
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

#include <mavros/mavconn_udp.h>
#include <ros/console.h>
#include <ros/assert.h>

#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>

namespace mavconn {

static bool resolve_address_udp(std::string host, unsigned short port, sockaddr_in &sin)
{
	memset(&sin, 0, sizeof(sin));

	// part of libros transport_udp.cpp
	sin.sin_family = AF_INET;
	if (inet_addr(host.c_str()) == INADDR_NONE) {
		// try resolve hostname
		struct addrinfo* addr;
		struct addrinfo hints;
		memset(&hints, 0, sizeof(hints));
		hints.ai_family = AF_UNSPEC;

		if (getaddrinfo(host.c_str(), NULL, &hints, &addr) != 0) {
			ROS_ERROR_NAMED("mavconn", "UDP: couldn't resolve host [%s]", host.c_str());
			return false;
		}

		bool found = false;
		struct addrinfo* it = addr;
		for (; it; it = it->ai_next) {
			if (it->ai_family == AF_INET) {
				memcpy(&sin, it->ai_addr, it->ai_addrlen);
				sin.sin_family = it->ai_family;
				sin.sin_port = htons(port);

				found = true;
				break;
			}
		}

		freeaddrinfo(addr);

		if (!found) {
			ROS_ERROR_NAMED("mavconn", "UDP: Couldn't find an AF_INET address for [%s]\n", host.c_str());
			return false;
		}

		ROS_DEBUG_NAMED("mavconn", "UDP: Resolved host [%s] to [%s]", host.c_str(), inet_ntoa(sin.sin_addr));
	}
	else {
		// alredy IPv4 addr
		sin.sin_addr.s_addr = inet_addr(host.c_str());
		sin.sin_port = htons(port);
	}

	return true;
}

MAVConnUDP::MAVConnUDP(uint8_t system_id, uint8_t component_id,
		std::string server_addr, unsigned short server_port,
		std::string listner_addr, unsigned short listner_port) :
	MAVConnInterface(system_id, component_id),
	sockfd(-1),
	remote_exists(false)
{
	memset(&remote_addr, 0, sizeof(remote_addr));
	memset(&last_remote_addr, 0, sizeof(last_remote_addr));

	if (!resolve_address_udp(server_addr, server_port, bind_addr))
		throw DeviceError("udp: resolve", "Bind address resolve failed");

	ROS_INFO_NAMED("mavconn", "udp: Bind address: %s:%d",
			inet_ntoa(bind_addr.sin_addr), ntohs(bind_addr.sin_port));

	if (listner_addr != "") {
			remote_exists = resolve_address_udp(listner_addr, listner_port, remote_addr);

			if (remote_exists)
				ROS_INFO_NAMED("mavconn", "udp: GCS address: %s:%d",
						inet_ntoa(remote_addr.sin_addr),
						ntohs(remote_addr.sin_port));
			else
				ROS_WARN_NAMED("mavconn", "udp: GCS address resolve failed.");
	}

	sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0)
		throw DeviceError("udp: socket", errno);

	if (::bind(sockfd, (sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
		throw DeviceError("udp: bind", errno);

	// NOTE: not shure that this needed, broke connection of GCS if it starts afret node
	//if (remote_exists) {
	//	if (::connect(sockfd, (sockaddr *)&remote_addr, sizeof(remote_addr)))
	//		throw DeviceError("udp: connect", "Connect error.");
	//}

	if (fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL, 0) | O_NONBLOCK) == -1)
		throw DeviceError("udp: fcntl", errno);

	// run io for async io
	io.set<MAVConnUDP, &MAVConnUDP::event_cb>(this);
	io.start(sockfd, ev::READ);
	start_default_loop();
}

MAVConnUDP::~MAVConnUDP() {
	close();
}

void MAVConnUDP::close() {
	if (sockfd < 0)
		return;

	io.stop();
	::close(sockfd); sockfd = -1;

	/* emit */ port_closed();
}

void MAVConnUDP::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!remote_exists) {
		ROS_DEBUG_NAMED("mavconn", "udp::send_bytes: Remote not known, message dropped.");
		return;
	}

	MsgBuffer *buf = new MsgBuffer(bytes, length);
	{
		boost::recursive_mutex::scoped_lock lock(mutex);
		tx_q.push_back(buf);
		io.set(ev::READ | ev::WRITE);
	}
}

void MAVConnUDP::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	ROS_ASSERT(message != nullptr);
	MsgBuffer *buf = nullptr;

	if (!remote_exists) {
		ROS_DEBUG_NAMED("mavconn", "udp::send_message: Remote not known, message dropped.");
		return;
	}

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

	ROS_DEBUG_NAMED("mavconn", "udp::send_message: Message-ID: %d [%zu bytes] Sys-Id: %d Comp-Id: %d",
			message->msgid, buf->nbytes(), sysid, compid);

	{
		boost::recursive_mutex::scoped_lock lock(mutex);
		tx_q.push_back(buf);
		io.set(ev::READ | ev::WRITE);
	}
}

void MAVConnUDP::event_cb(ev::io &watcher, int revents)
{
	if (ev::ERROR & revents) {
		ROS_ERROR_NAMED("mavconn", "event_cb::revents: 0x%08x", revents);
		close();
		return;
	}

	if (ev::READ & revents)
		read_cb(watcher);

	if (ev::WRITE & revents)
		write_cb(watcher);
}

void MAVConnUDP::read_cb(ev::io &watcher)
{
	mavlink_message_t message;
	mavlink_status_t status;
	uint8_t rx_buf[MsgBuffer::MAX_SIZE];
	socklen_t remote_len = sizeof(remote_addr);

	ssize_t nread = ::recvfrom(watcher.fd, rx_buf, sizeof(rx_buf), 0,
			(sockaddr *)&remote_addr, &remote_len);
	if (nread < 1) {
		ROS_ERROR_NAMED("mavconn", "udp::read_cb: %s", strerror(errno));
		close();
		return;
	}

	if (memcmp(&remote_addr, &last_remote_addr, sizeof(remote_addr)) != 0) {
		ROS_INFO_NAMED("mavconn", "udp: GCS address: %s:%d",
				inet_ntoa(remote_addr.sin_addr),
				ntohs(remote_addr.sin_port));

		remote_exists = true;
		last_remote_addr = remote_addr;
	}

	for (ssize_t i = 0; i < nread; i++) {
		if (mavlink_parse_char(channel, rx_buf[i], &message, &status)) {
			ROS_DEBUG_NAMED("mavconn", "udp::read_cb: recv Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d",
					message.msgid, message.len, message.sysid, message.compid);

			/* emit */ message_received(&message, message.sysid, message.compid);
		}
	}
}

void MAVConnUDP::write_cb(ev::io &watcher)
{
	boost::recursive_mutex::scoped_lock lock(mutex);

	if (tx_q.empty()) {
		io.set(ev::READ);
		return;
	}

	MsgBuffer *buf = tx_q.front();
	ssize_t written = ::sendto(watcher.fd, buf->dpos(), buf->nbytes(), 0,
			(sockaddr *)&remote_addr, sizeof(remote_addr));
	if (written < 0) {
		ROS_ERROR_NAMED("mavconn", "udp::write_cb: %s", strerror(errno));
		close();
		return;
	}

	buf->pos += written;
	if (buf->nbytes() == 0) {
		tx_q.pop_front();
		delete buf;
	}

	if (tx_q.empty())
		io.set(ev::READ);
	else
		io.set(ev::READ | ev::WRITE);
}

}; // namespace mavconn
