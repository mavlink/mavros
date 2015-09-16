/**
 * @brief Mavlink convert utils
 * @file
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <algorithm>
#include <mavros_msgs/Mavlink.h>
#include <mavconn/mavlink_dialect.h>

namespace mavros_msgs {
namespace mavlink {

/**
 * @brief Convert mavros_msgs/Mavlink message to mavlink_message_t
 *
 * @param[in]  rmsg	mavros_msgs/Mavlink message
 * @param[out] mmsg	mavlink_message_t struct
 * @return true if success
 */
inline bool convert(const mavros_msgs::Mavlink &rmsg, mavlink_message_t &mmsg)
{
	if (rmsg.payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0])) {
		return false;
	}

	mmsg.magic = MAVLINK_STX;
	mmsg.len = rmsg.len;
	mmsg.seq = rmsg.seq;
	mmsg.sysid = rmsg.sysid;
	mmsg.compid = rmsg.compid;
	mmsg.msgid = rmsg.msgid;
	mmsg.checksum = rmsg.checksum;
	std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);
	return true;
};

/**
 * @brief Convert mavlink_message_t to mavros/Mavlink
 *
 * @param[in]  mmsg	mavlink_message_t struct
 * @param[out] rmsg	mavros_msgs/Mavlink message
 * @return true, this convertion can't fail
 */
inline bool convert(const mavlink_message_t &mmsg, mavros_msgs::Mavlink &rmsg)
{
	const size_t payload64_len = (mmsg.len + 7) / 8;

	// XXX: remove this after #286
	rmsg.is_valid = true;

	rmsg.len = mmsg.len;
	rmsg.seq = mmsg.seq;
	rmsg.sysid = mmsg.sysid;
	rmsg.compid = mmsg.compid;
	rmsg.msgid = mmsg.msgid;
	rmsg.checksum = mmsg.checksum;
	rmsg.payload64 = std::move(mavros_msgs::Mavlink::_payload64_type(mmsg.payload64, mmsg.payload64 + payload64_len));

	return true;
};

};	// namespace mavlink
};	// namespace mavros_msgs
