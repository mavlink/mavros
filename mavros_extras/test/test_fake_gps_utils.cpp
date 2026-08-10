/*
 * Copyright 2026 Daniil Mordanov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "fake_gps_utils.hpp"

namespace fake_gps = mavros::extra_plugins::fake_gps;

static constexpr double epsilon = 1e-9;

TEST(FakeGpsUtils, ConvertsEcefVelocityToNed)
{
  const Eigen::Vector3d map_origin(0.0, 0.0, 0.0);
  const Eigen::Vector3d previous_ecef = Eigen::Vector3d::Zero();

  const auto north = fake_gps::calculate_velocity_ned(
    Eigen::Vector3d(0.0, 0.0, 2.0), previous_ecef, 2.0, map_origin);
  EXPECT_NEAR(north.x(), 1.0, epsilon);
  EXPECT_NEAR(north.y(), 0.0, epsilon);
  EXPECT_NEAR(north.z(), 0.0, epsilon);

  const auto east = fake_gps::calculate_velocity_ned(
    Eigen::Vector3d(0.0, 2.0, 0.0), previous_ecef, 2.0, map_origin);
  EXPECT_NEAR(east.x(), 0.0, epsilon);
  EXPECT_NEAR(east.y(), 1.0, epsilon);
  EXPECT_NEAR(east.z(), 0.0, epsilon);

  const auto up = fake_gps::calculate_velocity_ned(
    Eigen::Vector3d(2.0, 0.0, 0.0), previous_ecef, 2.0, map_origin);
  EXPECT_NEAR(up.x(), 0.0, epsilon);
  EXPECT_NEAR(up.y(), 0.0, epsilon);
  EXPECT_NEAR(up.z(), -1.0, epsilon);
}

TEST(FakeGpsUtils, RejectsNonIncreasingTimestamps)
{
  const Eigen::Vector3d current_ecef(1.0, 2.0, 3.0);
  const Eigen::Vector3d previous_ecef = Eigen::Vector3d::Zero();
  const Eigen::Vector3d map_origin(0.0, 0.0, 0.0);

  EXPECT_TRUE(
    fake_gps::calculate_velocity_ned(
      current_ecef, previous_ecef, 0.0, map_origin).isZero());
  EXPECT_TRUE(
    fake_gps::calculate_velocity_ned(
      current_ecef, previous_ecef, -1.0, map_origin).isZero());
}

TEST(FakeGpsUtils, ComputesCourseClockwiseFromNorth)
{
  EXPECT_EQ(
    fake_gps::course_over_ground_cdeg(Eigen::Vector3d(0.0, 0.0, 0.0)), 0);
  EXPECT_EQ(
    fake_gps::course_over_ground_cdeg(Eigen::Vector3d(1.0, 0.0, 0.0)), 0);
  EXPECT_EQ(
    fake_gps::course_over_ground_cdeg(Eigen::Vector3d(1.0, 1.0, 0.0)), 4500);
  EXPECT_EQ(
    fake_gps::course_over_ground_cdeg(Eigen::Vector3d(0.0, 1.0, 0.0)), 9000);
  EXPECT_EQ(
    fake_gps::course_over_ground_cdeg(Eigen::Vector3d(-1.0, 0.0, 0.0)), 18000);
  EXPECT_EQ(
    fake_gps::course_over_ground_cdeg(Eigen::Vector3d(0.0, -1.0, 0.0)), 27000);
}
