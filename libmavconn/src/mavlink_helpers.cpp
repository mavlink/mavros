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

#include <cassert>
#include <mavconn/mavlink_dialect.h>

/* I moved those arrays outside of functions because
 * module arrays easier to find with gdb
 */
static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];

/**
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
	assert(chan < MAVLINK_COMM_NUM_BUFFERS);
	return &m_mavlink_status[chan];
}

/**
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{
	assert(chan < MAVLINK_COMM_NUM_BUFFERS);
	return &m_mavlink_buffer[chan];
}

