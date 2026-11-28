//
// mavros
// Copyright 2026 Vishwa (vishwagw), All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Tests for attitude covariance from ATTITUDE_QUATERNION_COV: the imu plugin
 * applies it to ~/data (also for data from ATTITUDE_QUATERNION), and
 * local_position puts it in the orientation block of ~/odom only when the FCU
 * reports it.
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "test_plugin_helpers.hpp"
#include "../src/plugins/imu.cpp"             // NOLINT(build/include)
#include "../src/plugins/local_position.cpp"  // NOLINT(build/include)

using mavros::uas::TestUAS;
using mavros::uas::TopicCapture;
using mavros::uas::UAS;
using mavlink::common::msg::ATTITUDE_QUATERNION;
using mavlink::common::msg::ATTITUDE_QUATERNION_COV;
using mavlink::common::msg::LOCAL_POSITION_NED_COV;

namespace
{

//! Diagonal of the rotational (roll, pitch, yaw) block of a row-major 6x6 covariance.
std::array<double, 3> rotation_diagonal(const std::array<double, 36> & cov)
{
  return {cov[21], cov[28], cov[35]};
}

//! True if the rotational block and its cross terms of a 6x6 covariance are all zero.
bool rotation_block_is_zero(const std::array<double, 36> & cov)
{
  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      if ((r >= 3 || c >= 3) && cov[r * 6 + c] != 0.0) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

class AttitudeCovarianceTest : public TestUAS
{
public:
  static constexpr uint8_t FCU_SYSTEM = 1;
  static constexpr uint8_t FCU_COMPONENT = 1;

  UAS::SharedPtr uas;
  std::shared_ptr<mavros::std_plugins::IMUPlugin> imu;
  std::shared_ptr<mavros::std_plugins::LocalPositionPlugin> local_position;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  std::shared_ptr<TopicCapture<sensor_msgs::msg::Imu>> imu_data;
  std::shared_ptr<TopicCapture<nav_msgs::msg::Odometry>> odom;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(FCU_SYSTEM, FCU_COMPONENT);

    imu = std::make_shared<mavros::std_plugins::IMUPlugin>(uas.get());
    local_position = std::make_shared<mavros::std_plugins::LocalPositionPlugin>(uas.get());
    register_plugin(uas, imu);
    register_plugin(uas, local_position);

    // Match the SensorDataQoS publishers.
    imu_data = mavros::uas::capture_topic<sensor_msgs::msg::Imu>(
      imu->get_node(), "~/data", subs, rclcpp::SensorDataQoS());
    odom = mavros::uas::capture_topic<nav_msgs::msg::Odometry>(
      local_position->get_node(), "~/odom", subs, rclcpp::SensorDataQoS());

    add_node(uas);
    add_node(imu->get_node());
    add_node(local_position->get_node());
    start_spin();

    ASSERT_TRUE(mavros::uas::wait_connected(subs));
  }

  void TearDown() override
  {
    stop_spin();
  }

  void inject(const mavlink::Message & m)
  {
    auto mmsg = convert_message(m);
    mmsg.sysid = FCU_SYSTEM;
    mmsg.compid = FCU_COMPONENT;
    route(uas, &mmsg, mavconn::Framing::ok);
  }

  //! Level aircraft heading North (identity aircraft -> NED).
  static ATTITUDE_QUATERNION_COV attitude_cov()
  {
    ATTITUDE_QUATERNION_COV a{};
    a.time_usec = 1000000;
    a.q = {1.0f, 0.0f, 0.0f, 0.0f};
    a.rollspeed = 0.1f;
    a.pitchspeed = 0.2f;
    a.yawspeed = 0.3f;
    // roll, pitch, yaw; row-major
    a.covariance = {
      0.01f, 0.004f, 0.0f,
      0.004f, 0.02f, 0.005f,
      0.0f, 0.005f, 0.03f,
    };
    return a;
  }

  //! The same attitude as attitude_cov(), without covariance.
  static ATTITUDE_QUATERNION attitude_quaternion()
  {
    ATTITUDE_QUATERNION q{};
    q.time_boot_ms = 1500;
    q.q1 = 1.0f;
    return q;
  }

  static LOCAL_POSITION_NED_COV local_position_cov()
  {
    LOCAL_POSITION_NED_COV p{};
    p.time_usec = 2000000;
    p.x = 1.0f;
    p.y = 2.0f;
    p.z = -3.0f;
    p.covariance.fill(0.0f);
    p.covariance[0] = 0.5f;  // x
    return p;
  }

  //! ~/data orientation covariance with the default ~orientation_stdev (1.0).
  static void expect_stdev_orientation_covariance(const std::array<double, 9> & cov)
  {
    const std::array<double, 9> expected = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    for (size_t i = 0; i < expected.size(); ++i) {
      EXPECT_DOUBLE_EQ(cov[i], expected[i]) << "index " << i;
    }
  }
};

// Aircraft -> base_link flips the pitch and yaw axes: the roll/pitch and
// roll/yaw terms change sign, the pitch/yaw term does not.
TEST_F(AttitudeCovarianceTest, imu_data_carries_the_fcu_attitude_covariance)
{
  inject(attitude_cov());

  ASSERT_TRUE(imu_data->wait_for(1));
  const auto m = imu_data->snapshot().front();
  const std::array<double, 9> expected = {
    0.01, -0.004, 0.0,
    -0.004, 0.02, 0.005,
    0.0, 0.005, 0.03,
  };
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(m.orientation_covariance[i], expected[i], 1e-7) << "index " << i;
  }

  // Rates are converted as for ATTITUDE_QUATERNION.
  EXPECT_NEAR(m.angular_velocity.x, 0.1, 1e-6);
  EXPECT_NEAR(m.angular_velocity.y, -0.2, 1e-6);
  EXPECT_NEAR(m.angular_velocity.z, -0.3, 1e-6);
}

// NaN in the first element means unknown: fall back to ~orientation_stdev.
TEST_F(AttitudeCovarianceTest, nan_fcu_covariance_uses_orientation_stdev)
{
  auto a = attitude_cov();
  a.covariance[0] = std::numeric_limits<float>::quiet_NaN();
  inject(a);

  ASSERT_TRUE(imu_data->wait_for(1));
  expect_stdev_orientation_covariance(imu_data->snapshot().front().orientation_covariance);
}

// An all-zero (or otherwise non-positive diagonal) covariance is unknown too.
TEST_F(AttitudeCovarianceTest, zero_fcu_covariance_uses_orientation_stdev)
{
  auto a = attitude_cov();
  a.covariance.fill(0.0f);
  inject(a);

  ASSERT_TRUE(imu_data->wait_for(1));
  expect_stdev_orientation_covariance(imu_data->snapshot().front().orientation_covariance);
}

// When ATTITUDE_QUATERNION is streamed, it keeps publishing ~/data with the cached
// FCU covariance, and ATTITUDE_QUATERNION_COV no longer publishes on its own.
TEST_F(AttitudeCovarianceTest, fcu_covariance_is_applied_to_attitude_quaternion_data)
{
  inject(attitude_cov());
  ASSERT_TRUE(imu_data->wait_for(1));

  inject(attitude_quaternion());
  ASSERT_TRUE(imu_data->wait_for(2));
  EXPECT_NEAR(imu_data->snapshot().back().orientation_covariance[0], 0.01, 1e-7);
  EXPECT_NEAR(imu_data->snapshot().back().orientation_covariance[1], -0.004, 1e-7);

  inject(attitude_cov());
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(imu_data->snapshot().size(), 2U);

  inject(attitude_quaternion());
  ASSERT_TRUE(imu_data->wait_for(3));
  EXPECT_NEAR(imu_data->snapshot().back().orientation_covariance[0], 0.01, 1e-7);
}

// Heading North, base_link roll is about the world North (y) axis, so the roll and
// pitch variances swap places in the world-frame pose covariance.
TEST_F(AttitudeCovarianceTest, odom_gets_the_fcu_orientation_covariance)
{
  inject(attitude_cov());
  ASSERT_TRUE(imu_data->wait_for(1));

  inject(local_position_cov());
  ASSERT_TRUE(odom->wait_for(1));
  const auto o = odom->snapshot().back();

  // Position covariance is unchanged.
  EXPECT_FLOAT_EQ(o.pose.covariance[0], 0.5);

  const auto pose_rot = rotation_diagonal(o.pose.covariance);
  EXPECT_NEAR(pose_rot[0], 0.02, 1e-7);
  EXPECT_NEAR(pose_rot[1], 0.01, 1e-7);
  EXPECT_NEAR(pose_rot[2], 0.03, 1e-7);
  // world x/y (base_link pitch/roll) term: -0.004 in base_link, sign kept by the yaw
  EXPECT_NEAR(o.pose.covariance[3 * 6 + 4], 0.004, 1e-7);
  EXPECT_NEAR(o.pose.covariance[4 * 6 + 3], 0.004, 1e-7);

  // Position/rotation cross terms stay zero.
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 3; c < 6; ++c) {
      EXPECT_DOUBLE_EQ(o.pose.covariance[r * 6 + c], 0.0);
      EXPECT_DOUBLE_EQ(o.pose.covariance[c * 6 + r], 0.0);
    }
  }

  // No angular velocity covariance in MAVLink attitude messages.
  EXPECT_TRUE(rotation_block_is_zero(o.twist.covariance));
}

// With stock parameters and no ATTITUDE_QUATERNION_COV, odometry is unchanged:
// the ~orientation_stdev estimate in ~/data does not reach ~/odom.
TEST_F(AttitudeCovarianceTest, odom_orientation_covariance_stays_zero_without_the_cov_message)
{
  inject(attitude_quaternion());
  ASSERT_TRUE(imu_data->wait_for(1));
  EXPECT_DOUBLE_EQ(imu_data->snapshot().front().orientation_covariance[0], 1.0);

  inject(local_position_cov());
  ASSERT_TRUE(odom->wait_for(1));
  const auto o = odom->snapshot().front();
  EXPECT_TRUE(rotation_block_is_zero(o.pose.covariance));
  EXPECT_TRUE(rotation_block_is_zero(o.twist.covariance));
}

// An unknown covariance after a valid one clears the odometry block again.
TEST_F(AttitudeCovarianceTest, unknown_fcu_covariance_clears_the_odom_block)
{
  inject(attitude_cov());
  ASSERT_TRUE(imu_data->wait_for(1));

  auto a = attitude_cov();
  a.covariance[0] = std::numeric_limits<float>::quiet_NaN();
  inject(a);
  ASSERT_TRUE(imu_data->wait_for(2));

  inject(local_position_cov());
  ASSERT_TRUE(odom->wait_for(1));
  EXPECT_TRUE(rotation_block_is_zero(odom->snapshot().front().pose.covariance));
}

int main(int argc, char ** argv)
{
  TestUAS::Init();
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
