/*
 * Copyright 2026 Daniil Mordanov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <cmath>
#include <cstdint>

#include <Eigen/Core>

#include "mavros/frame_tf.hpp"

namespace mavros
{
namespace extra_plugins
{
namespace fake_gps
{

/**
 * @brief Derive local NED velocity from two ECEF positions.
 *
 * @param current_ecef Current ECEF position [m].
 * @param previous_ecef Previous ECEF position [m].
 * @param dt Elapsed time [s].
 * @param map_origin Geodetic map origin [latitude, longitude, altitude].
 * @return Velocity in the local North-East-Down frame [m/s].
 */
inline Eigen::Vector3d calculate_velocity_ned(
  const Eigen::Vector3d & current_ecef,
  const Eigen::Vector3d & previous_ecef,
  const double dt,
  const Eigen::Vector3d & map_origin)
{
  if (dt <= 0.0) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d velocity_ecef = (current_ecef - previous_ecef) / dt;
  const Eigen::Vector3d velocity_enu = ftf::transform_frame_ecef_enu(
    velocity_ecef, map_origin);
  return ftf::transform_frame_enu_ned(velocity_enu);
}

/**
 * @brief Compute course over ground from a NED velocity vector.
 *
 * @return Course clockwise from north in centidegrees in the range [0, 35999].
 */
inline uint16_t course_over_ground_cdeg(const Eigen::Vector3d & velocity_ned)
{
  const double north = velocity_ned.x();
  const double east = velocity_ned.y();

  if (north == 0.0 && east == 0.0) {
    return 0;
  }

  double course_deg = std::atan2(east, north) * 180.0 / M_PI;
  if (course_deg < 0.0) {
    course_deg += 360.0;
  }

  return static_cast<uint16_t>(std::lround(course_deg * 100.0)) % 36000;
}

}  // namespace fake_gps
}  // namespace extra_plugins
}  // namespace mavros
