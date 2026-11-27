//
// mavros
// Copyright 2026 Vishwa (vishwagw), All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Tests for the vision_pose plugin: a pose in ENU/base_link from ~/pose or
 * ~/pose_cov is sent as VISION_POSITION_ESTIMATE in NED/aircraft, with the
 * covariance rotated and packed as an upper-right triangle, and a repeated
 * stamp is dropped.
 */

#include <cmath>
#include <memory>
#include <vector>

#include "test_extras_plugin_helpers.hpp"
#include "../src/plugins/vision_pose_estimate.cpp"  // NOLINT(build/include)

using mavros::uas::TestUAS;
using mavros::uas::UAS;
using mavlink::common::msg::VISION_POSITION_ESTIMATE;

class VisionPoseTest : public TestUAS
{
public:
  static constexpr uint8_t FCU_SYSTEM = 1;
  static constexpr uint8_t FCU_COMPONENT = 1;

  UAS::SharedPtr uas;
  std::shared_ptr<mavros::extra_plugins::VisionPoseEstimatePlugin> plugin;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<CaptureBuffer> out;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(FCU_SYSTEM, FCU_COMPONENT);

    plugin = std::make_shared<mavros::extra_plugins::VisionPoseEstimatePlugin>(uas.get());
    register_plugin(uas, plugin);
    node = plugin->get_node();

    out = capture_sink(uas);
    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
    pose_cov_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/pose_cov", 10);

    add_node(uas);
    add_node(node);
    start_spin();

    ASSERT_TRUE(mavros::uas::wait_matched(pose_pub));
    ASSERT_TRUE(mavros::uas::wait_matched(pose_cov_pub));
    ASSERT_TRUE(mavros::uas::wait_connected(sink_subs_));
  }

  void TearDown() override
  {
    stop_spin();
  }

  //! Pose at (1, 2, 3) m ENU, heading @a yaw rad from East (counter-clockwise).
  static geometry_msgs::msg::Pose pose_enu(const double yaw)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = 1.0;
    p.position.y = 2.0;
    p.position.z = 3.0;
    p.orientation.z = std::sin(yaw / 2.0);
    p.orientation.w = std::cos(yaw / 2.0);
    return p;
  }

  static builtin_interfaces::msg::Time stamp(int32_t sec, uint32_t nanosec)
  {
    builtin_interfaces::msg::Time t;
    t.sec = sec;
    t.nanosec = nanosec;
    return t;
  }
};

// ENU (1, 2, 3) -> NED (2, 1, -3); heading 30 deg from East -> yaw 60 deg from North.
TEST_F(VisionPoseTest, pose_is_sent_in_ned_and_aircraft_frame)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = stamp(100, 500000000);
  ps.pose = pose_enu(M_PI / 6.0);
  pose_pub->publish(ps);

  const auto sent = mavros::uas::wait_sent<VISION_POSITION_ESTIMATE>(*out, 1);
  ASSERT_EQ(sent.size(), 1U);
  const auto & vp = sent.front();
  EXPECT_EQ(vp.usec, 100500000U);
  EXPECT_NEAR(vp.x, 2.0, 1e-5);
  EXPECT_NEAR(vp.y, 1.0, 1e-5);
  EXPECT_NEAR(vp.z, -3.0, 1e-5);
  EXPECT_NEAR(vp.roll, 0.0, 1e-5);
  EXPECT_NEAR(vp.pitch, 0.0, 1e-5);
  EXPECT_NEAR(vp.yaw, M_PI / 3.0, 1e-5);

  // ~/pose carries no covariance: it is sent as all zeros.
  for (const float c : vp.covariance) {
    EXPECT_FLOAT_EQ(c, 0.0f);
  }
}

// Covariance is reflected to NED (x <-> y, z sign) and packed column by column
// from the lower triangle: diagonal elements are at 0, 6, 11, 15, 18, 20.
TEST_F(VisionPoseTest, covariance_is_rotated_to_ned_and_packed)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pc;
  pc.header.stamp = stamp(7, 0);
  pc.pose.pose = pose_enu(0.0);
  for (size_t i = 0; i < 6; ++i) {
    pc.pose.covariance[i * 6 + i] = static_cast<double>(i + 1);  // x y z roll pitch yaw
  }
  // ENU x/z correlation becomes NED y/z with the sign flipped by z -> -z.
  pc.pose.covariance[0 * 6 + 2] = 0.7;
  pc.pose.covariance[2 * 6 + 0] = 0.7;
  pose_cov_pub->publish(pc);

  const auto sent = mavros::uas::wait_sent<VISION_POSITION_ESTIMATE>(*out, 1);
  ASSERT_EQ(sent.size(), 1U);
  const auto & cov = sent.front().covariance;
  EXPECT_FLOAT_EQ(cov[0], 2.0f);   // x (was ENU y)
  EXPECT_FLOAT_EQ(cov[6], 1.0f);   // y (was ENU x)
  EXPECT_FLOAT_EQ(cov[11], 3.0f);  // z
  EXPECT_FLOAT_EQ(cov[15], 5.0f);  // roll (was ENU pitch)
  EXPECT_FLOAT_EQ(cov[18], 4.0f);  // pitch (was ENU roll)
  EXPECT_FLOAT_EQ(cov[20], 6.0f);  // yaw
  EXPECT_FLOAT_EQ(cov[2], 0.0f);   // x/z
  EXPECT_FLOAT_EQ(cov[7], -0.7f);  // y/z

  // Heading East in ENU is yaw +90 deg in NED.
  EXPECT_NEAR(sent.front().yaw, M_PI / 2.0, 1e-5);
}

// A pose with the same stamp as the last one sent is dropped.
TEST_F(VisionPoseTest, repeated_stamp_is_dropped)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = stamp(42, 0);
  ps.pose = pose_enu(0.0);
  pose_pub->publish(ps);
  ASSERT_EQ(mavros::uas::wait_sent<VISION_POSITION_ESTIMATE>(*out, 1).size(), 1U);

  ps.pose.position.x = 9.0;
  pose_pub->publish(ps);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(mavros::uas::sent_messages<VISION_POSITION_ESTIMATE>(*out).size(), 1U);

  ps.header.stamp = stamp(42, 1000);
  pose_pub->publish(ps);
  const auto sent = mavros::uas::wait_sent<VISION_POSITION_ESTIMATE>(*out, 2);
  ASSERT_EQ(sent.size(), 2U);
  EXPECT_EQ(sent.back().usec, 42000001U);
  EXPECT_NEAR(sent.back().y, 9.0, 1e-5);  // ENU x -> NED y
}

int main(int argc, char ** argv)
{
  TestUAS::Init();
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
