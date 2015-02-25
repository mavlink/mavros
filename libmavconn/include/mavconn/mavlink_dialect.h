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

#if !defined(MAVLINK_DIALECT)
# define MAVLINK_DIALECT common
# warning "No MAVLINK_DIALECT specified. fallback to " MAVLINK_DIALECT
#endif

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
#define _DIALECT_pixhawk	6
#define _DIALECT_slugs		7
#define _DIALECT_test		8
#define _DIALECT_ualberta	9
#define _DIALECT_sensesoar	10
#define _DIALECT_ASLUAV		11

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
#else
#  error "Unknown MAVLINK_DIALECT"
#endif

