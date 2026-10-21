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

#include <cinttypes>

#include "mavros/mavros_uas.hpp"

using namespace mavros;         // NOLINT
using namespace mavros::uas;    // NOLINT


/* -*- time synchronise functions -*- */

// builtin_interfaces/msg/Time stores seconds as int32,
// so only stamps in [0, 2^31) s can be published.
static constexpr int64_t kMaxStampNs = (INT64_C(1) << 31) * INT64_C(1000000000);

static inline bool is_stamp_publishable(const int64_t stamp_ns)
{
  return stamp_ns >= 0 && stamp_ns < kMaxStampNs;
}

static inline rclcpp::Time ros_time_from_ns(const int64_t stamp_ns)
{
  return rclcpp::Time(stamp_ns);
}

rclcpp::Time UAS::synchronise_stamp(uint32_t time_boot_ms)
{
  // copy offset from atomic var
  int64_t offset_ns = time_offset;

  if (offset_ns != 0 || tsync_mode == timesync_mode::PASSTHROUGH) {
    int64_t stamp_ns = static_cast<int64_t>(time_boot_ms) * 1000000L + offset_ns;
    if (is_stamp_publishable(stamp_ns)) {
      return ros_time_from_ns(stamp_ns);
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TM: synchronised stamp %" PRId64 " ns out of range, using current time", stamp_ns);
  }

  return this->now();
}

rclcpp::Time UAS::synchronise_stamp(uint64_t time_usec)
{
  int64_t offset_ns = time_offset;

  if (offset_ns != 0 || tsync_mode == timesync_mode::PASSTHROUGH) {
    // bound time_usec first, a corrupt value would overflow the conversion to ns
    if (time_usec < static_cast<uint64_t>(kMaxStampNs / 1000)) {
      int64_t stamp_ns = static_cast<int64_t>(time_usec) * 1000L + offset_ns;
      if (is_stamp_publishable(stamp_ns)) {
        return ros_time_from_ns(stamp_ns);
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TM: synchronised stamp out of range (time_usec %" PRIu64 ", offset %" PRId64
      " ns), using current time",
      time_usec, offset_ns);
  }

  return this->now();
}
