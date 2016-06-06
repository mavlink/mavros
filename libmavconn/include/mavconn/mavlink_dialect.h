/**
 * @brief MAVConn mavlink.h selector
 * @file mavlink_dialect.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

/* Do not use default inlined mavlink functions!
 * Issue #269
 */
#define MAVLINK_GET_CHANNEL_STATUS
#define MAVLINK_GET_CHANNEL_BUFFER
#define MAVLINK_GET_MSG_ENTRY

// include common types before dialect only needed for these overridden functions
#include <mavlink/v2.0/mavlink_types.h>

// definition in mavlink_helpers.cpp
extern "C" {
	mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
	mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
	mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid);
}

// XXX use empy to generate this list.
namespace mavconn {

namespace ardupilotmega {
#include <mavconn/mavlink_undef.h>
#include <mavlink/v2.0/ardupilotmega/mavlink.h>
}

namespace common {
#include <mavconn/mavlink_undef.h>
#include <mavlink/v2.0/common/mavlink.h>
}

} // namespace mavconn
