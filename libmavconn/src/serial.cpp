/**
 * @brief MAVConn Serial link class
 * @file serial.cpp
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
#include <mavconn/serial.h>

namespace mavconn {
using boost::system::error_code;
using boost::asio::io_service;
using boost::asio::serial_port_base;
using boost::asio::buffer;
typedef std::lock_guard<std::recursive_mutex> lock_guard;

#define PFXd	"mavconn: serial%d: "


MAVConnSerial::MAVConnSerial(uint8_t system_id, uint8_t component_id,
		std::string device, unsigned baudrate) :
	MAVConnInterface(system_id, component_id),
	tx_in_progress(false),
	io_service(),
	serial_dev(io_service)
{
	logInform(PFXd "device: %s @ %d bps", channel, device.c_str(), baudrate);

	try {
		serial_dev.open(device);

		// Sent baudrate, and 8N1 mode
		serial_dev.set_option(serial_port_base::baud_rate(baudrate));
		serial_dev.set_option(serial_port_base::character_size(8));
		serial_dev.set_option(serial_port_base::parity(serial_port_base::parity::none));
		serial_dev.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
		serial_dev.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("serial", err);
	}

	// give some work to io_service before start
	io_service.post(boost::bind(&MAVConnSerial::do_read, this));

	// run io_service for async io
	std::thread t(boost::bind(&io_service::run, &this->io_service));
	mavutils::set_thread_name(t, "MAVConnSerial%d", channel);
	io_thread.swap(t);
}

MAVConnSerial::~MAVConnSerial() {
	close();
}

void MAVConnSerial::close() {
	lock_guard lock(mutex);
	if (!is_open())
		return;

	io_service.stop();
	serial_dev.close();

	// clear tx queue
	for (auto &p : tx_q)
		delete p;
	tx_q.clear();

	if (io_thread.joinable())
		io_thread.join();

	/* emit */ port_closed();
}

void MAVConnSerial::send_bytes(const uint8_t *bytes, size_t length)
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
	io_service.post(boost::bind(&MAVConnSerial::do_write, this, true));
}

void MAVConnSerial::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
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
	io_service.post(boost::bind(&MAVConnSerial::do_write, this, true));
}

void MAVConnSerial::do_read(void)
{
	serial_dev.async_read_some(
			buffer(rx_buf, sizeof(rx_buf)),
			boost::bind(&MAVConnSerial::async_read_end,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
}

void MAVConnSerial::async_read_end(error_code error, size_t bytes_transferred)
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

	do_read();
}

void MAVConnSerial::do_write(bool check_tx_state)
{
	if (check_tx_state && tx_in_progress)
		return;

	lock_guard lock(mutex);
	if (tx_q.empty())
		return;

	tx_in_progress = true;
	MsgBuffer *buf = tx_q.front();
	serial_dev.async_write_some(
			buffer(buf->dpos(), buf->nbytes()),
			boost::bind(&MAVConnSerial::async_write_end,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
}

void MAVConnSerial::async_write_end(error_code error, size_t bytes_transferred)
{
	if (error) {
		logError(PFXd "write: %s", channel, error.message().c_str());
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
		do_write(false);
	else
		tx_in_progress = false;
}

}; // namespace mavconn
