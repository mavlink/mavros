/**
 * @brief MAVROS UAS manager
 * @file uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014,2017,2021 Vladimir Ermakov, M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include "mavros/mavros_uas.hpp"

using namespace mavros;         // NOLINT
using namespace mavros::uas;    // NOLINT


/* -*- time syncronise functions -*- */

static inline rclcpp::Time ros_time_from_ns(const uint64_t stamp_ns)
{
  return rclcpp::Time(
    stamp_ns / 1000000000UL,                            // t_sec
    stamp_ns % 1000000000UL);                           // t_nsec
}

rclcpp::Time UAS::synchronise_stamp(uint32_t time_boot_ms)
{
  // copy offset from atomic var
  uint64_t offset_ns = time_offset;

  if (offset_ns > 0 || tsync_mode == timesync_mode::PASSTHROUGH) {
    uint64_t stamp_ns = static_cast<uint64_t>(time_boot_ms) * 1000000UL + offset_ns;
    return ros_time_from_ns(stamp_ns);
  } else {
    return this->now();
  }
}

rclcpp::Time UAS::synchronise_stamp(uint64_t time_usec)
{
  uint64_t offset_ns = time_offset;

  if (offset_ns > 0 || tsync_mode == timesync_mode::PASSTHROUGH) {
    uint64_t stamp_ns = time_usec * 1000UL + offset_ns;
    return ros_time_from_ns(stamp_ns);
  } else {
    return this->now();
  }
}
