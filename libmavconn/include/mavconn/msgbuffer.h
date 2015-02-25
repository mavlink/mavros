/**
 * @brief MAVConn message buffer class (internal)
 * @file msgbuffer.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2014,2015 Vladimir Ermakov, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */

#pragma once

#include <cassert>
#include <mavconn/mavlink_dialect.h>

namespace mavconn {

/**
 * @brief Message buffer for internal use in libmavconn
 */
struct MsgBuffer {
	//! Maximum buffer size with padding for CRC bytes (263 + 2 + align padding)
	static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 2 + 7;
	uint8_t data[MAX_SIZE];
	ssize_t len;
	ssize_t pos;

	MsgBuffer() :
		pos(0),
		len(0)
	{ }

	/**
	 * @brief Buffer constructor from mavlink_message_t
	 */
	explicit MsgBuffer(const mavlink_message_t *msg) :
		pos(0)
	{
		len = mavlink_msg_to_send_buffer(data, msg);
		// paranoic check, it must be less than MAVLINK_MAX_PACKET_LEN
		assert(len < MAX_SIZE);
	}

	/**
	 * @brief Buffer constructor for send_bytes()
	 * @param[in] nbytes should be less than MAX_SIZE
	 */
	MsgBuffer(const uint8_t *bytes, ssize_t nbytes) :
		pos(0),
		len(nbytes)
	{
		assert(0 < nbytes && nbytes < MAX_SIZE);
		memcpy(data, bytes, nbytes);
	}

	virtual ~MsgBuffer() {
		pos = 0;
		len = 0;
	}

	uint8_t *dpos() {
		return data + pos;
	}

	ssize_t nbytes() {
		return len - pos;
	}
};

}; // namespace mavconn

