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
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
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

