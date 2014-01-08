/**
 * @brief MAVConn mavlink.h selector
 * @file mavconn_mavlink.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#pragma once

#if !defined(MAVLINK_DIALECT)
# define MAVLINK_DIALECT common
# warning "No MAVLINK_DIALECT specified. fallback to " MAVLINK_DIALECT
#endif

#  if MAVLINK_DIALECT == ardupilotmega
#  include <mavlink/v1.0/ardupilotmega/mavlink.h>
#elif MAVLINK_DIALECT == autoquad
#  include <mavlink/v1.0/autoquad/mavlink.h>
#elif MAVLINK_DIALECT == common
#  include <mavlink/v1.0/common/mavlink.h>
#elif MAVLINK_DIALECT == matrixpilot
#  include <mavlink/v1.0/matrixpilot/mavlink.h>
#elif MAVLINK_DIALECT == minimal
#  include <mavlink/v1.0/minimal/mavlink.h>
#elif MAVLINK_DIALECT == pixhawk
#  include <mavlink/v1.0/pixhawk/mavlink.h>
#elif MAVLINK_DIALECT == slugs
#  include <mavlink/v1.0/slugs/mavlink.h>
#elif MAVLINK_DIALECT == test
#  include <mavlink/v1.0/test/mavlink.h>
#elif MAVLINK_DIALECT == ualberta
#  include <mavlink/v1.0/ualberta/mavlink.h>
#elif MAVLINK_DIALECT == sensesoar
#  include <mavlink/v1.0/sensesoar/mavlink.h>
#else
#  error "Unknown MAVLINK_DIALECT " MAVLINK_DIALECT
#endif

