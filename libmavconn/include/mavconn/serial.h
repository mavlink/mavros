/**
 * @brief MAVConn Serial link class
 * @file serial.h
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

#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <mavconn/interface.h>
#include <mavconn/msgbuffer.h>

namespace mavconn {
/**
 * @brief Serial interface
 */
class MAVConnSerial : public MAVConnInterface,
	public std::enable_shared_from_this<MAVConnSerial> {
public:
	static constexpr auto DEFAULT_DEVICE = "/dev/ttyACM0";
	static constexpr auto DEFAULT_BAUDRATE = 57600;

	/**
	 * Open and run serial link.
	 *
	 * @param[in] device    TTY device path
	 * @param[in] baudrate  serial baudrate
	 */
	MAVConnSerial(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string device = DEFAULT_DEVICE, unsigned baudrate = DEFAULT_BAUDRATE, bool hwflow = false);
	virtual ~MAVConnSerial();

	void close() override;

	void send_message(const mavlink::mavlink_message_t *message) override;
	void send_message(const mavlink::Message &message, const uint8_t source_compid) override;
	void send_bytes(const uint8_t *bytes, size_t length) override;

	inline bool is_open() override {
		return serial_dev.is_open();
	}

private:
	boost::asio::io_service io_service;
	std::thread io_thread;
	boost::asio::serial_port serial_dev;

	std::atomic<bool> tx_in_progress;
	std::deque<MsgBuffer> tx_q;
	std::array<uint8_t, MsgBuffer::MAX_SIZE> rx_buf;
	std::recursive_mutex mutex;

	void do_read();
	void do_write(bool check_tx_state);
};
}	// namespace mavconn
