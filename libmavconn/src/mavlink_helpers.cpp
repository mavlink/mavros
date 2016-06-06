/**
 * @brief MAVLink helpers
 * @file mavlink_helpers.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * This file defines replace for some helper function to prevent problem #269.
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2015 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavconn/interface.h>

using mavconn::MAVConnInterface;

// --- XXX ---

namespace _ardupilotmega {
#include <mavconn/mavlink_undef.h>
#include <mavlink/v2.0/ardupilotmega/mavlink.h>
}

static const mavlink_msg_entry_t ardupilot_msg_entries[] = MAVLINK_MESSAGE_CRCS;

namespace _common {
#include <mavconn/mavlink_undef.h>
#include <mavlink/v2.0/common/mavlink.h>
}

static const mavlink_msg_entry_t common_msg_entries[] = MAVLINK_MESSAGE_CRCS;

// --- XXX ---

/**
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
	assert(chan < MAVLINK_COMM_NUM_BUFFERS);
	return &MAVConnInterface::channel_status[chan];
}

/**
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{
	assert(chan < MAVLINK_COMM_NUM_BUFFERS);
	return &MAVConnInterface::channel_buffer[chan];
}

/**
 * Internal function to give access to message information such as additional crc byte.
 */
mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid)
{
	return NULL;
}
