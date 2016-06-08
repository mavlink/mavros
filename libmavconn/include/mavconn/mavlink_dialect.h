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

// XXX fix mavlink C++ to define this
#define MAVLINK_HELPER static inline

// XXX use empy to generate include list

#include <mavlink/v2.0/common/common.hpp>
#include <mavlink/v2.0/ardupilotmega/ardupilotmega.hpp>
