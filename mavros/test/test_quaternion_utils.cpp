//
// mavros
// Copyright 2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Test libmavros quaternion utilities
 */

#include <gtest/gtest.h>
#include <cmath>

#ifdef USE_OLD_TF2_ROS
# include <tf2/LinearMath/Quaternion.h>
# include <tf2/LinearMath/Matrix3x3.h>
#else
# include <tf2/LinearMath/Quaternion.hpp>
# include <tf2/LinearMath/Matrix3x3.hpp>
#endif

#include <rclcpp/rclcpp.hpp>
#include <mavros/frame_tf.hpp>

using namespace mavros;     // NOLINT

static const double epsilon = 1e-9;
static const double epsilon_f = 1e-6;
static const double deg_to_rad = M_PI / 180.0;
// gMock has ability to define array matcher, but there problems with that.
// so trying good old for loop

#define ASSERT_QUATERNION(q1, q2, epsilon) \
  ASSERT_NEAR(q1.w(), q2.w(), epsilon); \
  ASSERT_NEAR(q1.x(), q2.x(), epsilon); \
  ASSERT_NEAR(q1.y(), q2.y(), epsilon); \
  ASSERT_NEAR(q1.z(), q2.z(), epsilon)

#define EXPECT_QUATERNION(q1, q2, epsilon) \
  EXPECT_NEAR(q1.w(), q2.w(), epsilon); \
  EXPECT_NEAR(q1.x(), q2.x(), epsilon); \
  EXPECT_NEAR(q1.y(), q2.y(), epsilon); \
  EXPECT_NEAR(q1.z(), q2.z(), epsilon)

/* -*- quaternion_from_rpy / getYaw -*- */

TEST(FRAME_TF, quaternion_from_rpy__check_compatibility)
{
  auto eigen_q = ftf::quaternion_from_rpy(1.0, 2.0, 3.0);
  auto eigen_neg_q = ftf::quaternion_from_rpy(-1.0, -2.0, -3.0);
  tf2::Quaternion bt_q; bt_q.setRPY(1.0, 2.0, 3.0);
  tf2::Quaternion bt_neg_q; bt_neg_q.setRPY(-1.0, -2.0, -3.0);

  EXPECT_QUATERNION(bt_q, eigen_q, epsilon);
  EXPECT_QUATERNION(bt_neg_q, eigen_neg_q, epsilon);
}

TEST(FRAME_TF, quaternion_from_rpy__paranoic_check)
{
  auto q1 = ftf::quaternion_from_rpy(1.0, 2.0, 3.0);
  auto q2 = ftf::quaternion_from_rpy(Eigen::Vector3d(1.0, 2.0, 3.0));

  EXPECT_QUATERNION(q1, q2, epsilon);
}

TEST(FRAME_TF, quaternion_to_rpy__123)
{
  // Angles kept within the ZYX unique domain (|pitch| < pi/2) so that the
  // round-trip rpy->q->rpy recovers the original values exactly.
  auto q = ftf::quaternion_from_rpy(1.0, 0.5, 3.0);
  auto rpy = ftf::quaternion_to_rpy(q);

  EXPECT_NEAR(1.0, rpy.x(), epsilon);
  EXPECT_NEAR(0.5, rpy.y(), epsilon);
  EXPECT_NEAR(3.0, rpy.z(), epsilon);
}

// Regression test: quaternion_to_rpy must return the canonical yaw in (-pi, pi]
// for all headings, not clamp to [0, pi] as Eigen::eulerAngles(2,1,0) does.
TEST(FRAME_TF, quaternion_to_rpy__yaw_full_circle)
{
  const double roll = 0.1, pitch = 0.2;

  for (int yaw_deg = -179; yaw_deg <= 180; ++yaw_deg) {
    const double yaw = yaw_deg * deg_to_rad;

    std::stringstream ss;
    ss << "yaw=" << yaw_deg << "deg";
    SCOPED_TRACE(ss.str());

    auto q = ftf::quaternion_from_rpy(roll, pitch, yaw);
    auto rpy = ftf::quaternion_to_rpy(q);

    // Canonical yaw recovered by atan2 lives in (-pi, pi].
    // Allow for floating-point round-trip; the exact recovered value must
    // equal the input since both sit in (-pi, pi].
    EXPECT_NEAR(roll, rpy.x(), epsilon);
    EXPECT_NEAR(pitch, rpy.y(), epsilon);
    EXPECT_NEAR(yaw, rpy.z(), epsilon);
  }
}

TEST(FRAME_TF, quaternion_to_rpy__pm_pi)
{
  // this test try large count of different angles

  // in degrees
  const ssize_t min = -180;
  const ssize_t max = 180;
  const ssize_t step = 15;

  const auto test_orientation = ftf::quaternion_from_rpy(1.0, 2.0, 3.0);

  for (ssize_t roll = min; roll <= max; roll += step) {
    for (ssize_t pitch = min; pitch <= max; pitch += step) {
      for (ssize_t yaw = min; yaw <= max; yaw += step) {
        Eigen::Vector3d expected_deg(roll, pitch, yaw);
        Eigen::Vector3d expected = expected_deg * deg_to_rad;

        std::stringstream ss;

        ss << "DEG(" << expected_deg.x() << ", " << expected_deg.y() << ", " << expected_deg.z() <<
          ")  ";
        ss << "RAD(" << expected.x() << ", " << expected.y() << ", " << expected.z() << ")";

        SCOPED_TRACE(ss.str());

        // rpy->q->rpy->q
        auto q1 = ftf::quaternion_from_rpy(expected);
        auto rpy = ftf::quaternion_to_rpy(q1);
        auto q2 = ftf::quaternion_from_rpy(rpy);

        // Direct angle comparison is still undefined near gimbal-lock (pitch ≈ ±90°)
        // because ZYX atan2/asin decomposition is not unique there.
        // The yaw-clamping bug (eulerAngles clamping to [0,pi]) is covered
        // separately by quaternion_to_rpy__yaw_full_circle above.
        // EXPECT_NEAR(expected.x(), rpy.x(), epsilon);
        // EXPECT_NEAR(expected.y(), rpy.y(), epsilon);
        // EXPECT_NEAR(expected.z(), rpy.z(), epsilon);

        // instead of direct comparison we rotate other quaternion and then compare results
        auto tq1 = q1 * test_orientation * q1.inverse();
        auto tq2 = q2 * test_orientation * q2.inverse();

        EXPECT_QUATERNION(tq1, tq2, epsilon);
      }
    }
  }
}

// ftf::quaternion_to_rpy() is not compatible with tf2::Matrix3x3(q).getRPY()

TEST(FRAME_TF, quaternion_get_yaw__123)
{
  // with pitch >= pi/2 got incorrect result.
  // so 1, 2, 3 rad (57, 115, 172 deg) replaced with 60, 89, 172 deg
  auto q = ftf::quaternion_from_rpy(60.0 * deg_to_rad, 89.0 * deg_to_rad, 3.0);

  EXPECT_NEAR(3.0, ftf::quaternion_get_yaw(q), epsilon);
}

TEST(FRAME_TF, quaternion_get_yaw__pm_pi)
{
  // in degrees
  const ssize_t min = -180;
  const ssize_t max = 180;
  const ssize_t step = 1;

  for (ssize_t yaw = min; yaw <= max; yaw += step) {
    Eigen::Vector3d expected_deg(1.0, 2.0, yaw);
    Eigen::Vector3d expected = expected_deg * deg_to_rad;

    std::stringstream ss;

    ss << "DEG(" << expected_deg.x() << ", " << expected_deg.y() << ", " << expected_deg.z() <<
      ")  ";
    ss << "RAD(" << expected.x() << ", " << expected.y() << ", " << expected.z() << ")";

    SCOPED_TRACE(ss.str());

    auto q1 = ftf::quaternion_from_rpy(expected);
    double q1_yaw = ftf::quaternion_get_yaw(q1);

    EXPECT_NEAR(expected.z(), q1_yaw, epsilon);
  }
}

/* -*- mavlink util -*- */

static double angle_delta(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

TEST(FRAME_TF, quaternion_to_rpy_zyx__roundtrip)
{
  // Pitch stays clear of ±90 deg, where roll and yaw are not unique.
  for (int roll = -180; roll <= 180; roll += 15) {
    for (int pitch = -75; pitch <= 75; pitch += 15) {
      for (int yaw = -180; yaw <= 180; yaw += 15) {
        Eigen::Vector3d expected(
          roll * deg_to_rad, pitch * deg_to_rad, yaw * deg_to_rad);

        auto q = ftf::quaternion_from_rpy(expected);
        auto rpy = ftf::quaternion_to_rpy_zyx(q);
        auto rpy_neg = ftf::quaternion_to_rpy_zyx(
          Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z()));

        EXPECT_NEAR(0.0, angle_delta(expected.x(), rpy.x()), epsilon);
        EXPECT_NEAR(0.0, angle_delta(expected.y(), rpy.y()), epsilon);
        EXPECT_NEAR(0.0, angle_delta(expected.z(), rpy.z()), epsilon);
        EXPECT_NEAR(0.0, angle_delta(rpy.x(), rpy_neg.x()), epsilon);
        EXPECT_NEAR(0.0, angle_delta(rpy.y(), rpy_neg.y()), epsilon);
        EXPECT_NEAR(0.0, angle_delta(rpy.z(), rpy_neg.z()), epsilon);
        EXPECT_NEAR(0.0, angle_delta(rpy.z(), ftf::quaternion_get_yaw(q)), epsilon);
      }
    }
  }
}

static void expect_same_rotation(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  const double d = std::abs(a.normalized().dot(b.normalized()));
  EXPECT_NEAR(1.0, d, epsilon);
}

TEST(FRAME_TF, quaternion_to_rpy_zyx__gimbal_lock)
{
  // At pitch = ±90 deg roll and yaw are not unique. Pitch is clamped, and the
  // recovered triple must still be the same rotation.
  auto q_up = ftf::quaternion_from_rpy(0.2, M_PI_2, -1.0);
  auto rpy_up = ftf::quaternion_to_rpy_zyx(q_up);
  EXPECT_NEAR(M_PI_2, rpy_up.y(), 1e-8);
  expect_same_rotation(q_up, ftf::quaternion_from_rpy(rpy_up));

  auto q_down = ftf::quaternion_from_rpy(-0.4, -M_PI_2, 1.1);
  auto rpy_down = ftf::quaternion_to_rpy_zyx(q_down);
  EXPECT_NEAR(-M_PI_2, rpy_down.y(), 1e-8);
  expect_same_rotation(q_down, ftf::quaternion_from_rpy(rpy_down));
}

TEST(FRAME_TF, quaternion_to_rpy_zyx__enu_is_not_fcu)
{
  const double roll = 0.3;
  const double pitch = -0.4;
  const double yaw = 1.2;
  auto ned = ftf::quaternion_from_rpy(roll, pitch, yaw);
  auto enu = ftf::transform_orientation_aircraft_baselink(
    ftf::transform_orientation_ned_enu(ned));

  auto from_ned = ftf::quaternion_to_rpy_zyx(ned);
  auto from_enu = ftf::quaternion_to_rpy_zyx(enu);

  EXPECT_NEAR(roll, from_ned.x(), epsilon);
  EXPECT_NEAR(pitch, from_ned.y(), epsilon);
  EXPECT_NEAR(yaw, from_ned.z(), epsilon);
  EXPECT_GT((from_enu - from_ned).norm(), 0.1);
}

TEST(FRAME_TF, quaternion_to_rpy_zyx__zero_quaternion)
{
  auto rpy = ftf::quaternion_to_rpy_zyx(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(rpy.isZero(0.0));
}

TEST(FRAME_TF, quaternion_to_mavlink__123)
{
  auto eigen_q = ftf::quaternion_from_rpy(1.0, 2.0, 3.0);
  std::array<float, 4> mavlink_q;

  ftf::quaternion_to_mavlink(eigen_q, mavlink_q);

  EXPECT_NEAR(mavlink_q[0], eigen_q.w(), epsilon_f);
  EXPECT_NEAR(mavlink_q[1], eigen_q.x(), epsilon_f);
  EXPECT_NEAR(mavlink_q[2], eigen_q.y(), epsilon_f);
  EXPECT_NEAR(mavlink_q[3], eigen_q.z(), epsilon_f);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
