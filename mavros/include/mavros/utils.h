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
 * Copyright 2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <mavconn/thread_utils.h>
#include <mavros_msgs/mavlink_convert.h>

// redeclare message at deprecated location
namespace mavros {
typedef mavros_msgs::Mavlink Mavlink __attribute__((deprecated));
typedef mavros_msgs::MavlinkPtr MavlinkPtr __attribute__((deprecated));
};	// namespace mavros

namespace mavutils {
/**
 * @brief Copy mavros/Mavlink.msg message data to mavlink_message_t
 * @deprecated Please use mavros_msgs::mavlink::convert() instead.
 */
inline bool copy_ros_to_mavlink(const mavros_msgs::Mavlink::ConstPtr &rmsg, mavlink_message_t &mmsg) __attribute__((deprecated));
bool copy_ros_to_mavlink(const mavros_msgs::Mavlink::ConstPtr &rmsg, mavlink_message_t &mmsg)
{
	return mavros_msgs::mavlink::convert(*rmsg, mmsg);
};

/**
 * @brief Copy mavlink_message_t to mavros/Mavlink.msg
 * @deprecated Please use mavros_msgs::mavlink::convert() instead.
 */
inline void copy_mavlink_to_ros(const mavlink_message_t *mmsg, mavros_msgs::MavlinkPtr &rmsg) __attribute__((deprecated));
void copy_mavlink_to_ros(const mavlink_message_t *mmsg, mavros_msgs::MavlinkPtr &rmsg)
{
	mavros_msgs::mavlink::convert(*mmsg, *rmsg);
};
};	// namespace mavutils
