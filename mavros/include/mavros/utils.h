/**
 * @brief some useful utils
 * @file utils.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavutils
 * @{
 *  @brief Some useful utils
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <algorithm>
#include <mavconn/thread_utils.h>

#include <mavros/Mavlink.h>
#include <mavconn/mavlink_dialect.h>


namespace mavutils {
/**
 * @brief Copy mavros/Mavlink.msg message data to mavlink_message_t
 */
inline bool copy_ros_to_mavlink(const mavros::Mavlink::ConstPtr &rmsg, mavlink_message_t &mmsg)
{
	if (rmsg->payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0])) {
		return false;
	}

	mmsg.msgid = rmsg->msgid;
	mmsg.len = rmsg->len;
	std::copy(rmsg->payload64.begin(), rmsg->payload64.end(), mmsg.payload64);
	return true;
};

/**
 * @brief Copy mavlink_message_t to mavros/Mavlink.msg
 */
inline void copy_mavlink_to_ros(const mavlink_message_t *mmsg, mavros::MavlinkPtr &rmsg)
{
	rmsg->len = mmsg->len;
	rmsg->seq = mmsg->seq;
	rmsg->sysid = mmsg->sysid;
	rmsg->compid = mmsg->compid;
	rmsg->msgid = mmsg->msgid;

	rmsg->payload64.reserve((mmsg->len + 7) / 8);
	for (size_t i = 0; i < (mmsg->len + 7) / 8; i++)
		rmsg->payload64.push_back(mmsg->payload64[i]);
};
};	// namespace mavutils
