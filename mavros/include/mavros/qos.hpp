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

/**
 * QoS for state data that is latched and periodically republished, like
 * connection/mode state or the home position.
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
static inline rclcpp::QoS StateQoS()
{
  return rclcpp::QoS(10).transient_local();
}

//! Publisher options that opt out of intra-process comms.
//
// Latched (transient_local) publishers are not compatible with intra-process
// delivery; rclcpp throws "intraprocess communication allowed only with
// volatile durability" when such a publisher is created on an intra-process
// node (e.g. on humble). Plugin nodes enable intra-process comms, so latched
// publishers must disable it per-publisher.
static inline rclcpp::PublisherOptions NonIntraProcessPublisherOptions()
{
  rclcpp::PublisherOptions opts;
  opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  return opts;
}

//! Subscription options that opt out of intra-process comms (see above).
static inline rclcpp::SubscriptionOptions NonIntraProcessSubscriptionOptions()
{
  rclcpp::SubscriptionOptions opts;
  opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  return opts;
}

}       // namespace mavros

#endif  // MAVROS__QOS_HPP_
