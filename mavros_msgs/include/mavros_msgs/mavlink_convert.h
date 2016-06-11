/**
 * @brief Mavlink convert utils
 * @file
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2015,2016 Vladimir Ermakov.
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

using ::mavlink::mavlink_message_t;

/**
 * @brief Convert mavros_msgs/Mavlink message to mavlink_message_t
 *
 * @note signature vector should be empty for unsigned OR
 *       MAVLINK_SIGNATURE_BLOCK size for signed messages
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

	if (!rmsg.signature.empty() && rmsg.signature.size() != sizeof(mmsg.signature)) {
		return false;
	}

	mmsg.magic = mmsg.magic;
	mmsg.len = rmsg.len;
	mmsg.incompat_flags = rmsg.incompat_flags;
	mmsg.compat_flags = rmsg.compat_flags;
	mmsg.seq = rmsg.seq;
	mmsg.sysid = rmsg.sysid;
	mmsg.compid = rmsg.compid;
	mmsg.msgid = rmsg.msgid;
	mmsg.checksum = rmsg.checksum;
	std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);
	std::copy(rmsg.signature.begin(), rmsg.signature.end(), mmsg.signature);

	return true;
}

/**
 * @brief Convert mavlink_message_t to mavros/Mavlink
 *
 * @param[in]  mmsg	mavlink_message_t struct
 * @param[out] rmsg	mavros_msgs/Mavlink message
 * @param[in]  framing_status  framing parse result (OK, BAD_CRC or BAD_SIGNATURE)
 * @return true, this convertion can't fail
 */
inline bool convert(const mavlink_message_t &mmsg, mavros_msgs::Mavlink &rmsg, uint8_t framing_status = mavros_msgs::Mavlink::FRAMING_OK)
{
	const size_t payload64_len = (mmsg.len + 7) / 8;

	rmsg.framing_status = framing_status;

	rmsg.magic = mmsg.magic;
	rmsg.len = mmsg.len;
	rmsg.incompat_flags = mmsg.incompat_flags;
	rmsg.compat_flags = mmsg.compat_flags;
	rmsg.seq = mmsg.seq;
	rmsg.sysid = mmsg.sysid;
	rmsg.compid = mmsg.compid;
	rmsg.msgid = mmsg.msgid;
	rmsg.checksum = mmsg.checksum;
	rmsg.payload64 = std::move(mavros_msgs::Mavlink::_payload64_type(mmsg.payload64, mmsg.payload64 + payload64_len));

	// copy signature block only if message is signed
	if (mmsg.incompat_flags & MAVLINK_IFLAG_SIGNED)
		rmsg.signature = std::move(mavros_msgs::Mavlink::_signature_type(mmsg.signature, mmsg.signature + sizeof(mmsg.signature)));
	else
		rmsg.signature.clear();

	return true;
}

}	// namespace mavlink
}	// namespace mavros_msgs
