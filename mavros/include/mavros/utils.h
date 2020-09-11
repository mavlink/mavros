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

#include <ros/console.h>

// OS X compat: missing error codes
#ifdef __APPLE__
#define EBADE 50	/* Invalid exchange */
#define EBADFD 81	/* File descriptor in bad state */
#define EBADRQC 54	/* Invalid request code */
#define EBADSLT 55	/* Invalid slot */
#endif

namespace mavros {
namespace utils {
using mavconn::utils::format;

/**
 * Possible modes of timesync operation
 *
 * Used by UAS class, but it can't be defined inside because enum is used in utils.
 */
enum class timesync_mode {
	NONE = 0,	//!< Disabled
	MAVLINK,	//!< Via TIMESYNC message
	ONBOARD,
	PASSTHROUGH,
};

/**
 * Helper to get enum value from strongly typed enum (enum class).
 */
template<typename _T>
constexpr typename std::underlying_type<_T>::type enum_value(_T e)
{
	return static_cast<typename std::underlying_type<_T>::type>(e);
}

/**
 * Get string repr for timesync_mode
 */
std::string to_string(timesync_mode e);

/**
 * @brief Retrieve alias of the orientation received by MAVLink msg.
 */
std::string to_string(mavlink::common::MAV_SENSOR_ORIENTATION e);

std::string to_string(mavlink::minimal::MAV_AUTOPILOT e);
std::string to_string(mavlink::minimal::MAV_TYPE e);
std::string to_string(mavlink::minimal::MAV_STATE e);
std::string to_string(mavlink::minimal::MAV_COMPONENT e);
std::string to_string(mavlink::common::MAV_ESTIMATOR_TYPE e);
std::string to_string(mavlink::common::ADSB_ALTITUDE_TYPE e);
std::string to_string(mavlink::common::ADSB_EMITTER_TYPE e);
std::string to_string(mavlink::common::MAV_MISSION_RESULT e);
std::string to_string(mavlink::common::MAV_FRAME e);
std::string to_string(mavlink::common::MAV_DISTANCE_SENSOR e);
std::string to_string(mavlink::common::LANDING_TARGET_TYPE e);
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

/**
 * @brief Retrieve timesync mode from name
 */
timesync_mode timesync_mode_from_str(const std::string &mode);

/**
 * @brief Retreive MAV_FRAME from name
 */
mavlink::common::MAV_FRAME mav_frame_from_str(const std::string &mav_frame);

/**
 * @brief Retreive MAV_TYPE from name
 */
mavlink::minimal::MAV_TYPE mav_type_from_str(const std::string &mav_type);

/**
 * @brief Retrieve landing target type from alias name
 */
mavlink::common::LANDING_TARGET_TYPE landing_target_type_from_str(const std::string &landing_target_type);

}	// namespace utils
}	// namespace mavros
