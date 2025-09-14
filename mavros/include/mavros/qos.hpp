/*
 * Copyright 2025 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief additional QoS configs
 * @file qos.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavutils
 * @{
 *  @brief Additional QoS configs
 */

#pragma once

#ifndef MAVROS__QOS_HPP_
#define MAVROS__QOS_HPP_

#include "rclcpp/rclcpp.hpp"

namespace mavros
{

/**
 * QoS for latched in-frequently updated data, like GPS origin or mission list.
 *
 * - History: Keep last,
 * - Depth: 10,
 * - Reliability: Reliable,
 * - Durability: Transient Local,
 * - Deadline: Default,
 * - Lifespan: Default,
 * - Liveliness: System default,
 * - Liveliness lease duration: default,
 * - Avoid ros namespace conventions: false
 */
static inline rclcpp::QoS LatchedStateQoS()
{
  return rclcpp::QoS(10).keep_last(1).reliable().transient_local();
}

}       // namespace mavros

#endif  // MAVROS__QOS_HPP_
