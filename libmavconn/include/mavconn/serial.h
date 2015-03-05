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
 * Copyright 2013,2014,2015 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <list>
#include <atomic>
#include <boost/asio.hpp>
#include <mavconn/interface.h>
#include <mavconn/msgbuffer.h>

namespace mavconn {

/**
 * @brief Serial interface
 */
class MAVConnSerial : public MAVConnInterface {
public:
	/**
	 * Open and run serial link.
	 *
	 * @param[in] device    TTY device path
	 * @param[in] baudrate  serial baudrate
	 */
	MAVConnSerial(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string device = "/dev/ttyACM0", unsigned baudrate = 57600);
	~MAVConnSerial();

	void close();

	using MAVConnInterface::send_message;
	void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
	void send_bytes(const uint8_t *bytes, size_t length);

	inline bool is_open() { return serial_dev.is_open(); };

private:
	boost::asio::io_service io_service;
	std::thread io_thread;
	boost::asio::serial_port serial_dev;

	std::atomic<bool> tx_in_progress;
	std::list<MsgBuffer*> tx_q;
	uint8_t rx_buf[MsgBuffer::MAX_SIZE];
	std::recursive_mutex mutex;

	void do_read();
	void async_read_end(boost::system::error_code, size_t bytes_transferred);
	void do_write(bool check_tx_state);
	void async_write_end(boost::system::error_code, size_t bytes_transferred);
};

}; // namespace mavconn

