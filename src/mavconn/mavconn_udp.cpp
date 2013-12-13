/**
 * @file mavconn_udp.cpp
 * @author Vladimit Ermkov <voon341@gmail.com>
 */
/*
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

#include "mavconn_udp.h"
#include <ros/console.h>
#include <ros/assert.h>

using namespace mavconn;

MAVConnUDP::MAVConnUDP(uint8_t system_id, uint8_t component_id,
		std::string server_addr, unsigned server_port) :
	io_service(),
	io_work(new boost::asio::io_service::work(io_service)),
	socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), server_port)),
	sender_exists(false)
{
	sys_id = system_id;
	comp_id = component_id;
	channel = new_channel();
	ROS_ASSERT_MSG(channel >= 0, "channel allocation failure");

	// give some work to io_service before start
	io_service.post(boost::bind(&MAVConnUDP::do_read, this));

	// run io_service for async io
	boost::thread t(boost::bind(&boost::asio::io_service::run, &this->io_service));
	io_thread.swap(t);
}

MAVConnUDP::~MAVConnUDP()
{
	io_work.reset();
	io_service.stop();
	delete_channel(channel);
}

void MAVConnUDP::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	ROS_ASSERT(message != NULL);
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN + 2];
	mavlink_message_t msg = *message;

#if MAVLINK_CRC_EXTRA
	mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len, mavlink_crcs[msg.msgid]);
#else
	mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len);
#endif
	size_t length = mavlink_msg_to_send_buffer(buffer, &msg);

	ROS_DEBUG_NAMED("mavconn", "udp::send_message: Message-ID: %d [%zu bytes]", message->msgid, length);

	{
		boost::recursive_mutex::scoped_lock lock(mutex);
		tx_q.insert(tx_q.end(), buffer, buffer + length);
	}
	io_service.post(boost::bind(&MAVConnUDP::do_write, this));
}

void MAVConnUDP::do_read(void)
{
	socket.async_receive_from(
			boost::asio::buffer(rx_buf, sizeof(rx_buf)),
			sender_endpoint,
			boost::bind(&MAVConnUDP::async_read_end,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
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

		sender_exists = true;

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

void MAVConnUDP::do_write(void)
{
	if (sender_exists) {
		ROS_DEBUG_NAMED("mavconn", "udp::do_write: sender do not exists!");
		return;
	}

	// if write not in progress
	if (tx_buf == 0) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		tx_buf_size = tx_q.size();
		tx_buf.reset(new uint8_t[tx_buf_size]);
		std::copy(tx_q.begin(), tx_q.end(), tx_buf.get());
		tx_q.clear();

		socket.async_send_to(
				boost::asio::buffer(tx_buf.get(), tx_buf_size),
				sender_endpoint,
				boost::bind(&MAVConnUDP::async_write_end,
					this,
					boost::asio::placeholders::error));
	}
}

void MAVConnUDP::async_write_end(boost::system::error_code error)
{
	if (!error) {
		boost::recursive_mutex::scoped_lock lock(mutex);

		if (tx_q.empty()) {
			tx_buf.reset();
			tx_buf_size = 0;
			return;
		}

		tx_buf_size = tx_q.size();
		tx_buf.reset(new uint8_t[tx_buf_size]);
		std::copy(tx_q.begin(), tx_q.end(), tx_buf.get());
		tx_q.clear();

		socket.async_send_to(
				boost::asio::buffer(tx_buf.get(), tx_buf_size),
				sender_endpoint,
				boost::bind(&MAVConnUDP::async_write_end,
					this,
					boost::asio::placeholders::error));
	} else {
		if (socket.is_open()) {
			socket.close();
			port_closed();
			ROS_ERROR_NAMED("mavconn", "udp::async_write_end: error! port closed.");
		}
	}
}

