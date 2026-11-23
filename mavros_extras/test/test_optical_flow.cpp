//
// mavros
// Copyright 2026 Vishwa (vishwagw), All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Tests for the optical_flow plugin: OPTICAL_FLOW from the FCU is published
 * with axes mapped from aircraft to base_link, with ground distance as a Range,
 * and OpticalFlow from ROS is sent to the FCU mapped the other way.
 */

#include <memory>
#include <vector>

#include "test_extras_plugin_helpers.hpp"
#include "../src/plugins/optical_flow.cpp"  // NOLINT(build/include)

using mavros::uas::TestUAS;
using mavros::uas::UAS;
using mavros::uas::TopicCapture;
using mavlink::common::msg::OPTICAL_FLOW;

class OpticalFlowTest : public TestUAS
{
public:
  static constexpr uint8_t FCU_SYSTEM = 1;
  static constexpr uint8_t FCU_COMPONENT = 1;

  UAS::SharedPtr uas;
  std::shared_ptr<mavros::extra_plugins::OpticalFlowPlugin> plugin;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<CaptureBuffer> out;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  std::shared_ptr<TopicCapture<mavros_msgs::msg::OpticalFlow>> flows;
  std::shared_ptr<TopicCapture<sensor_msgs::msg::Range>> ranges;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(FCU_SYSTEM, FCU_COMPONENT);

    plugin = std::make_shared<mavros::extra_plugins::OpticalFlowPlugin>(uas.get());
    register_plugin(uas, plugin);
    node = plugin->get_node();

    flows = mavros::uas::capture_topic<mavros_msgs::msg::OpticalFlow>(
      node, "~/raw/optical_flow", subs);
    ranges = mavros::uas::capture_topic<sensor_msgs::msg::Range>(
      node, "~/ground_distance", subs);
    out = capture_sink(uas);

    add_node(uas);
    add_node(node);
    start_spin();

    auto all_subs = subs;
    all_subs.insert(all_subs.end(), sink_subs_.begin(), sink_subs_.end());
    ASSERT_TRUE(mavros::uas::wait_connected(all_subs));
  }

  void TearDown() override
  {
    stop_spin();
  }

  void inject(const OPTICAL_FLOW & m, uint8_t sysid)
  {
    auto mmsg = convert_message(m);
    mmsg.sysid = sysid;
    mmsg.compid = FCU_COMPONENT;
    route(uas, &mmsg, mavconn::Framing::ok);
  }

  static OPTICAL_FLOW flow_from_fcu()
  {
    OPTICAL_FLOW m{};
    m.time_usec = 1000000;
    m.sensor_id = 3;
    m.flow_x = 10;
    m.flow_y = 20;
    m.flow_comp_m_x = 1.5f;
    m.flow_comp_m_y = -0.5f;
    m.quality = 200;
    m.ground_distance = 2.5f;
    m.flow_rate_x = 0.25f;
    m.flow_rate_y = 0.75f;
    return m;
  }
};

// Aircraft (FRD) -> base_link (FLU): x is kept, y changes sign.
TEST_F(OpticalFlowTest, flow_from_the_fcu_is_published_in_base_link)
{
  inject(flow_from_fcu(), FCU_SYSTEM);

  ASSERT_TRUE(flows->wait_for(1));
  const auto f = flows->snapshot().front();
  EXPECT_EQ(f.header.frame_id, "optical_flow");
  EXPECT_NEAR(f.flow.x, 10.0, 1e-9);
  EXPECT_NEAR(f.flow.y, -20.0, 1e-9);
  EXPECT_NEAR(f.flow.z, 0.0, 1e-9);
  EXPECT_FLOAT_EQ(f.flow_comp_m.x, 1.5);
  EXPECT_FLOAT_EQ(f.flow_comp_m.y, 0.5);
  EXPECT_FLOAT_EQ(f.flow_rate.x, 0.25);
  EXPECT_FLOAT_EQ(f.flow_rate.y, -0.75);
  EXPECT_FLOAT_EQ(f.ground_distance, 2.5f);
  EXPECT_EQ(f.quality, 200);
}

// Ground distance is also published as a Range with the configured limits.
TEST_F(OpticalFlowTest, ground_distance_is_published_as_a_range)
{
  inject(flow_from_fcu(), FCU_SYSTEM);

  ASSERT_TRUE(ranges->wait_for(1));
  const auto r = ranges->snapshot().front();
  EXPECT_EQ(r.header.frame_id, "optical_flow");
  EXPECT_EQ(r.radiation_type, sensor_msgs::msg::Range::INFRARED);
  EXPECT_FLOAT_EQ(r.range, 2.5f);
  EXPECT_FLOAT_EQ(r.field_of_view, 0.119428926f);
  EXPECT_FLOAT_EQ(r.min_range, 0.3f);
  EXPECT_FLOAT_EQ(r.max_range, 5.0f);
}

// The handler uses SystemAndOk: flow from another system is dropped.
TEST_F(OpticalFlowTest, flow_from_another_system_is_ignored)
{
  inject(flow_from_fcu(), FCU_SYSTEM + 1);
  std::this_thread::sleep_for(300ms);
  EXPECT_TRUE(flows->snapshot().empty());
  EXPECT_TRUE(ranges->snapshot().empty());

  // ...while the next message from the FCU still gets through.
  inject(flow_from_fcu(), FCU_SYSTEM);
  EXPECT_TRUE(flows->wait_for(1));
}

// base_link (FLU) -> aircraft (FRD) on the way to the FCU.
TEST_F(OpticalFlowTest, flow_from_ros_is_sent_to_the_fcu_in_aircraft_frame)
{
  auto pub = node->create_publisher<mavros_msgs::msg::OpticalFlow>("~/raw/send", 1);
  ASSERT_TRUE(mavros::uas::wait_matched(pub));

  mavros_msgs::msg::OpticalFlow f;
  f.header.stamp.sec = 12;
  f.header.stamp.nanosec = 345000000;
  f.flow.x = 10.0;
  f.flow.y = -20.0;
  f.flow_comp_m.x = 1.5;
  f.flow_comp_m.y = 0.5;
  f.flow_rate.x = 0.25;
  f.flow_rate.y = -0.75;
  f.quality = 150;
  f.ground_distance = 3.0f;
  pub->publish(f);

  const auto sent = mavros::uas::wait_sent<OPTICAL_FLOW>(*out, 1);
  ASSERT_EQ(sent.size(), 1U);
  const auto & m = sent.front();
  EXPECT_EQ(m.time_usec, 12345000U);
  EXPECT_EQ(m.sensor_id, 0);
  EXPECT_EQ(m.flow_x, 10);
  EXPECT_EQ(m.flow_y, 20);
  EXPECT_FLOAT_EQ(m.flow_comp_m_x, 1.5f);
  EXPECT_FLOAT_EQ(m.flow_comp_m_y, -0.5f);
  EXPECT_FLOAT_EQ(m.flow_rate_x, 0.25f);
  EXPECT_FLOAT_EQ(m.flow_rate_y, 0.75f);
  EXPECT_EQ(m.quality, 150);
  EXPECT_FLOAT_EQ(m.ground_distance, 3.0f);
}

int main(int argc, char ** argv)
{
  TestUAS::Init();
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
