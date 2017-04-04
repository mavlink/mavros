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
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
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
using boost::asio::buffer;
using mavlink::mavlink_message_t;


#define PFX	"mavconn: serial"
#define PFXd	PFX "%zu: "


MAVConnSerial::MAVConnSerial(uint8_t system_id, uint8_t component_id,
		std::string device, unsigned baudrate, bool hwflow) :
	MAVConnInterface(system_id, component_id),
	tx_in_progress(false),
	tx_q {},
	rx_buf {},
	io_service(),
	serial_dev(io_service)
{
	using SPB = boost::asio::serial_port_base;

	logInform(PFXd "device: %s @ %d bps", conn_id, device.c_str(), baudrate);

	try {
		serial_dev.open(device);

		// Set baudrate and 8N1 mode
		serial_dev.set_option(SPB::baud_rate(baudrate));
		serial_dev.set_option(SPB::character_size(8));
		serial_dev.set_option(SPB::parity(SPB::parity::none));
		serial_dev.set_option(SPB::stop_bits(SPB::stop_bits::one));
		serial_dev.set_option(SPB::flow_control( (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
	}
	catch (boost::system::system_error &err) {
		throw DeviceError("serial", err);
	}

	// NOTE: shared_from_this() should not be used in constructors

	// give some work to io_service before start
	io_service.post(std::bind(&MAVConnSerial::do_read, this));

	// run io_service for async io
	io_thread = std::thread([this] () {
				utils::set_this_thread_name("mserial%zu", conn_id);
				io_service.run();
			});
}

MAVConnSerial::~MAVConnSerial()
{
	close();
}

void MAVConnSerial::close()
{
	lock_guard lock(mutex);
	if (!is_open())
		return;

	io_service.stop();
	serial_dev.close();

	if (io_thread.joinable())
		io_thread.join();

	if (port_closed_cb)
		port_closed_cb();
}

void MAVConnSerial::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!is_open()) {
		logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnSerial::send_bytes: TX queue overflow");

		tx_q.emplace_back(bytes, length);
	}
	io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

void MAVConnSerial::send_message(const mavlink_message_t *message)
{
	assert(message != nullptr);

	if (!is_open()) {
		logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	log_send(PFX, message);

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnSerial::send_message: TX queue overflow");

		tx_q.emplace_back(message);
	}
	io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

void MAVConnSerial::send_message(const mavlink::Message &message)
{
	if (!is_open()) {
		logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	log_send_obj(PFX, message);

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnSerial::send_message: TX queue overflow");

		tx_q.emplace_back(message, get_status_p(), sys_id, comp_id);
	}
	io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

void MAVConnSerial::do_read(void)
{
	auto sthis = shared_from_this();
	serial_dev.async_read_some(
			buffer(rx_buf),
			[sthis] (error_code error, size_t bytes_transferred) {
				if (error) {
					logError(PFXd "receive: %s", sthis->conn_id, error.message().c_str());
					sthis->close();
					return;
				}

				sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
				sthis->do_read();
			});
}

void MAVConnSerial::do_write(bool check_tx_state)
{
	if (check_tx_state && tx_in_progress)
		return;

	lock_guard lock(mutex);
	if (tx_q.empty())
		return;

	tx_in_progress = true;
	auto sthis = shared_from_this();
	auto &buf_ref = tx_q.front();
	serial_dev.async_write_some(
			buffer(buf_ref.dpos(), buf_ref.nbytes()),
			[sthis, &buf_ref] (error_code error, size_t bytes_transferred) {
				assert(bytes_transferred <= buf_ref.len);

				if (error) {
					logError(PFXd "write: %s", sthis->conn_id, error.message().c_str());
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
					sthis->do_write(false);
				else
					sthis->tx_in_progress = false;
			});
}
}	// namespace mavconn
