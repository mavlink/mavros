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

#include <Eigen/Geometry>
#include <mavconn/thread_utils.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavconn/mavlink_dialect.h>

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

/**
 * @brief Retrieve alias of the orientation received by MAVLink msg.
 */
std::string to_string(mavlink::common::MAV_SENSOR_ORIENTATION e);

std::string to_string(mavlink::common::MAV_AUTOPILOT e);
std::string to_string(mavlink::common::MAV_TYPE e);
std::string to_string(mavlink::common::MAV_STATE e);

/**
 * Helper to call to_string() for enum _T
 */
template<typename _T>
std::string to_string_enum(int e)
{
	return to_string(static_cast<_T>(e));
}

/**
 * @brief Function to match the received orientation received by MAVLink msg
 *        and the rotation of the sensor relative to the FCU.
 */
Eigen::Quaterniond sensor_orientation_matching(mavlink::common::MAV_SENSOR_ORIENTATION orientation);

/**
 * @brief Retrieve sensor orientation number from alias name.
 */
int sensor_orientation_from_str(const std::string &sensor_orientation);


}	// namespace utils
}	// namespace mavros
