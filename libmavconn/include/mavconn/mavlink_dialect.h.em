/**
 * @@brief MAVConn mavlink.h selector
 * @@file mavlink_dialect.h
 * @@author Vladimir Ermakov <vooon341@@gmail.com>
 *
 * @@addtogroup mavconn
 * @@{
 */
/*
 * libmavconn
 * Copyright 2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
@#
@# EmPy template of dialect include file
@#

#pragma once

// AUTOMATIC GENERATED FILE!
// from include/mavconn/mavlink_dialect.h.em

#define MAVLINK_START_SIGN_STREAM(link_id)
#define MAVLINK_END_SIGN_STREAM(link_id)

@[for dialect in MAVLINK_V20_DIALECTS]#include <mavlink/v2.0/@(dialect)/@(dialect).hpp>
@[end for]
