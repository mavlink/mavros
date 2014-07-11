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
#include <mavros/utils.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <algorithm>

using namespace mavconn;
using namespace boost::asio::ip;

MAVConnUDP::MAVConnUDP(uint8_t system_id, uint8_t component_id,
		std::string server_addr, unsigned short server_port,
		std::string listner_addr, unsigned short listner_port) :
	MAVConnInterface(system_id, component_id),
	io_service(),
	io_work(new asio::io_service::work(io_service)),
	socket(io_service),
	sender_exists(false),
	tx_buf_size(0),
	tx_buf_max_size(0),
	tx_in_process(false)
{
	udp::resolver resolver(io_service);

	udp::resolver::query server_query(server_addr, "");
	for(udp::resolver::iterator i = resolver.resolve(server_query);
			i != udp::resolver::iterator();
			++i) {

		server_endpoint = *i;
		server_endpoint.port(server_port);
		ROS_INFO_STREAM_NAMED("mavconn", "udp: Bind address: " << server_endpoint);
	}

	if (listner_addr != "") {
		udp::resolver::query listner_query(listner_addr, "");
		for(udp::resolver::iterator i = resolver.resolve(listner_query);
				i != udp::resolver::iterator();
				++i) {

			sender_endpoint = *i;
			sender_endpoint.port(listner_port);
			prev_sender_endpoint = sender_endpoint;
			sender_exists = true;
			ROS_INFO_STREAM_NAMED("mavconn", "udp: GCS address: " << sender_endpoint);
		}
	}

	socket.open(udp::v4());
	socket.bind(server_endpoint);

	// reserve some space in tx queue
	tx_q.reserve(TX_EXTENT * 2);

	// give some work to io_service before start
	io_service.post(boost::bind(&MAVConnUDP::do_read, this));

	// run io_service for async io
	boost::thread t(boost::bind(&boost::asio::io_service::run, &this->io_service));
	mavutils::set_thread_name(t, "MAVConnUDP%d", channel);
	io_thread.swap(t);
}

MAVConnUDP::~MAVConnUDP()
{
	io_work.reset();
	io_service.stop();
}

void MAVConnUDP::send_bytes(const uint8_t *bytes, size_t length)
{
	{
		boost::recursive_mutex::scoped_lock lock(mutex);
		tx_q.insert(tx_q.end(), bytes, bytes + length);
	}
	io_service.post(boost::bind(&MAVConnUDP::do_write, this));
}

void MAVConnUDP::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	ROS_ASSERT(message != NULL);
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN + 2];
	size_t length;

	/* if sysid/compid pair not match we need explicit finalize
	 * else just copy to buffer */
	if (message->sysid != sysid || message->compid != compid) {
		mavlink_message_t msg = *message;

#if MAVLINK_CRC_EXTRA
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len, mavlink_crcs[msg.msgid]);
#else
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len);
#endif

		length = mavlink_msg_to_send_buffer(buffer, &msg);
	}
	else
		length = mavlink_msg_to_send_buffer(buffer, message);

	ROS_DEBUG_NAMED("mavconn", "udp::send_message: Message-ID: %d [%zu bytes] Sys-Id: %d Comp-Id: %d",
			message->msgid, length, sysid, compid);
	send_bytes(buffer, length);
}

void MAVConnUDP::do_read(void)
{
	socket.async_receive_from(
			asio::buffer(rx_buf, sizeof(rx_buf)),
			sender_endpoint,
			boost::bind(&MAVConnUDP::async_read_end,
				this,
				asio::placeholders::error,
				asio::placeholders::bytes_transferred));
}

void MAVConnUDP::async_read_end(boost::system::error_code error, size_t bytes_transfered)
{
	if (error) {
		if (socket.is_open()) {
			socket.close();
			port_closed();
			ROS_ERROR_NAMED("mavconn", "udp::async_read_end: error! port closed.");
		}
	} else {
		mavlink_message_t message;
		mavlink_status_t status;

		if (sender_endpoint != prev_sender_endpoint) {
			ROS_INFO_STREAM_NAMED("mavconn", "udp: GCS address: " << sender_endpoint);
			prev_sender_endpoint = sender_endpoint;
			sender_exists = true;
		}

		for (size_t i = 0; i < bytes_transfered; i++) {
			if (mavlink_parse_char(channel, rx_buf[i], &message, &status)) {
				ROS_DEBUG_NAMED("mavconn", "udp::async_read_end: recv Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d",
						message.msgid, message.len, message.sysid, message.compid);

				/* emit */ message_received(&message, message.sysid, message.compid);
			}
		}

		do_read();
	}
}

void MAVConnUDP::copy_and_async_write(void)
{
	// should called with locked mutex from io_service thread

	tx_buf_size = tx_q.size();
	// mark transmission in progress
	tx_in_process = true;

	if (tx_buf_max_size > TX_DELSIZE ||
			tx_buf_size >= tx_buf_max_size) {

		// Set buff eq. or gt than tx_buf_size
		tx_buf_max_size = (tx_buf_size % TX_EXTENT == 0)? tx_buf_size :
			(tx_buf_size / TX_EXTENT + 1) * TX_EXTENT;

		tx_buf.reset(new uint8_t[tx_buf_max_size]);
	}

	std::copy(tx_q.begin(), tx_q.end(), tx_buf.get());
	tx_q.clear();

	socket.async_send_to(
			asio::buffer(tx_buf.get(), tx_buf_size),
			sender_endpoint,
			boost::bind(&MAVConnUDP::async_write_end,
				this,
				asio::placeholders::error));
}

void MAVConnUDP::do_write(void)
{
	if (!sender_exists) {
		ROS_DEBUG_THROTTLE_NAMED(30, "mavconn", "udp::do_write: sender do not exists!");
		return;
	}

	// if write not in progress
	if (!tx_in_process) {
		boost::recursive_mutex::scoped_lock lock(mutex);
		copy_and_async_write();
	}
}

void MAVConnUDP::async_write_end(boost::system::error_code error)
{
	if (!error) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (tx_q.empty()) {
			tx_in_process = false;
			tx_buf_size = 0;
			return;
		}

		copy_and_async_write();
	} else {
		if (socket.is_open()) {
			socket.close();
			port_closed();
			ROS_ERROR_NAMED("mavconn", "udp::async_write_end: error! port closed.");
		}
	}
}

