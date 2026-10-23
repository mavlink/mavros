/** Check cumulative setpoint times and cancellation of the completed timer. */
// Copyright 2026 LostPatrol. See LICENSE.md for license terms.
#include <gtest/gtest.h>

#include <array>
#include <future>
#include <vector>

#include "test_plugin_helpers.hpp"
#include "../src/plugins/setpoint_trajectory.cpp"  // NOLINT(build/include)

namespace mavros
{
namespace uas
{

// One zero-start, nonuniform trajectory covers interval scheduling and completion.
TEST_F(TestUAS, trajectory_intervals_and_completion)
{
  auto uas = create_uas();
  auto plugin = std::make_shared<std_plugins::SetpointTrajectoryPlugin>(uas.get());
  auto node = plugin->get_node();
  auto pub = node->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    "~/local", rclcpp::SensorDataQoS());
  const std::array<double, 3> times{0.0, 0.4, 1.2};
  std::vector<rclcpp::Time> sent;
  std::promise<void> completed;
  auto future = completed.get_future();
  auto sub = uas->create_subscription<mavros_msgs::msg::Mavlink>(
    "~/mavlink_sink", rclcpp::SensorDataQoS(),
    [&sent, &completed, &times](mavros_msgs::msg::Mavlink::ConstSharedPtr msg) {
      if (msg->msgid == mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID) {
        sent.emplace_back(msg->header.stamp);
        if (sent.size() == times.size()) {
          completed.set_value();
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(uas);
  exec.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while ((pub->get_subscription_count() == 0 || sub->get_publisher_count() == 0) &&
    std::chrono::steady_clock::now() < deadline)
  {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(pub->get_subscription_count(), 0U);
  ASSERT_GT(sub->get_publisher_count(), 0U);

  trajectory_msgs::msg::MultiDOFJointTrajectory trajectory;
  for (const double time : times) {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(time);
    point.transforms.resize(1);
    point.transforms[0].rotation.w = 1.0;
    point.velocities.resize(1);
    point.accelerations.resize(1);
    trajectory.points.push_back(point);
  }
  pub->publish(trajectory);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, exec.spin_until_future_complete(future, 5s));
  ASSERT_EQ(times.size(), sent.size());
  for (size_t i = 1; i < times.size(); ++i) {
    EXPECT_NEAR(times[i], (sent[i] - sent[0]).seconds(), 0.2);
  }
  auto group = node->get_node_base_interface()->get_default_callback_group();
  EXPECT_EQ(nullptr, group->find_timer_ptrs_if(
      [](const rclcpp::TimerBase::SharedPtr & timer) {return !timer->is_canceled();}));
}

}  // namespace uas
}  // namespace mavros

/** Initialize ROS for the focused plugin regression. */
int main(int argc, char ** argv)
{
  mavros::uas::TestUAS::Init();
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
