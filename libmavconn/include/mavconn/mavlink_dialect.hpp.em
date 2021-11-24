//
// libmavconn
// Copyright 2014,2015,2016,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @@brief MAVConn mavlink.h selector
 * @@file mavlink_dialect.hpp
 * @@author Vladimir Ermakov <vooon341@@gmail.com>
 *
 * @@addtogroup mavconn
 * @@{
 */
@#
@# EmPy template of dialect include file
@#

#pragma once
#ifndef MAVCONN__MAVLINK_DIALECT_HPP_
#define MAVCONN__MAVLINK_DIALECT_HPP_

// AUTOMATIC GENERATED FILE!
// from include/mavconn/mavlink_dialect.hpp.em

namespace mavlink
{
#ifndef MAVLINK_VERSION
#include <mavlink/config.h>
constexpr auto version = MAVLINK_VERSION;
#undef MAVLINK_VERSION
#else
constexpr auto version = "unknown";
#endif
}  // namespace mavlink

#define MAVLINK_START_SIGN_STREAM(link_id)
#define MAVLINK_END_SIGN_STREAM(link_id)

// NOTE(vooon): ignore warning
// warning: ISO C++ prohibits anonymous structs
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

@[for dialect in MAVLINK_V20_DIALECTS]#include <mavlink/v2.0/@(dialect)/@(dialect).hpp>
@[end for]

#pragma GCC diagnostic pop

#endif  // MAVCONN__MAVLINK_DIALECT_HPP_
