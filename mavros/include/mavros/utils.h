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
 * Copyright 2014,2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <mavconn/thread_utils.h>
#include <mavros_msgs/mavlink_convert.h>

namespace mavros {
namespace utils {

using mavconn::utils::format;

/**
 * Helper to get enum value from strongly typed enum (enum class).
 */
template<typename _T>
constexpr typename std::underlying_type<_T>::type enum_value(_T e)
{
	return static_cast<typename std::underlying_type<_T>::type>(e);
}

}	// namespace utils
}	// namespace mavros
