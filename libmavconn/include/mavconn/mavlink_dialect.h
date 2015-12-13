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
 * Copyright 2014,2015 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#if !defined(MAVLINK_DIALECT)
# define MAVLINK_DIALECT common
# warning "No MAVLINK_DIALECT specified. fallback to " MAVLINK_DIALECT
#endif

/* Do not use default inlined mavlink functions!
 * Issue #269
 */
#define MAVLINK_GET_CHANNEL_STATUS
#define MAVLINK_GET_CHANNEL_BUFFER

// include common types before dialect only needed for these overridden functions
#include <mavlink/v1.0/mavlink_types.h>

// definition in mavlink_helpers.cpp
extern "C" mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
extern "C" mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);

/* C preprocessor could not compare strings directly
 * See: http://c-faq.com/cpp/ifstrcmp.html
 */
#define _DIALECT(name)		_DIALECT_CONCAT(name)
#define _DIALECT_CONCAT(name)	_DIALECT_ ## name
#define _DIALECT_ardupilotmega	1
#define _DIALECT_autoquad	2
#define _DIALECT_common		3
#define _DIALECT_matrixpilot	4
#define _DIALECT_minimal	5
#define _DIALECT_pixhawk	6	/* removed */
#define _DIALECT_slugs		7
#define _DIALECT_test		8
#define _DIALECT_ualberta	9
#define _DIALECT_sensesoar	10	/* removed */
#define _DIALECT_ASLUAV		11
#define _DIALECT_paparazzi	12

#  if _DIALECT(MAVLINK_DIALECT) == _DIALECT_ardupilotmega
#  include <mavlink/v1.0/ardupilotmega/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_autoquad
#  include <mavlink/v1.0/autoquad/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_common
#  include <mavlink/v1.0/common/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_matrixpilot
#  include <mavlink/v1.0/matrixpilot/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_minimal
#  include <mavlink/v1.0/minimal/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_pixhawk
#  include <mavlink/v1.0/pixhawk/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_slugs
#  include <mavlink/v1.0/slugs/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_test
#  include <mavlink/v1.0/test/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_ualberta
#  include <mavlink/v1.0/ualberta/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_sensesoar
#  include <mavlink/v1.0/sensesoar/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_ASLUAV
#  include <mavlink/v1.0/ASLUAV/mavlink.h>
#elif _DIALECT(MAVLINK_DIALECT) == _DIALECT_paparazzi
#  include <mavlink/v1.0/paparazzi/mavlink.h>
#else
#  error "Unknown MAVLINK_DIALECT"
#endif

