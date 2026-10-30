//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Tests for sys_time TIMESYNC: requests and replies are addressed, and only the
 * FCU's replies to our own requests feed the offset estimate.
 */

#include <memory>
#include <mutex>
#include <vector>

#include "rosgraph_msgs/msg/clock.hpp"
#include "test_plugin_helpers.hpp"
#include "../src/plugins/sys_time.cpp"  // NOLINT(build/include)

using mavros::uas::TestUAS;
using mavros::uas::UAS;
using mavlink::common::msg::TIMESYNC;

class SysTimeTest : public TestUAS
{
public:
  static constexpr uint8_t FCU_SYSTEM = 1;
  static constexpr uint8_t FCU_COMPONENT = 1;
  // Another component of the FCU's system, e.g. a camera or companion app.
  static constexpr uint8_t OTHER_COMPONENT = 154;

  UAS::SharedPtr uas;
  std::shared_ptr<mavros::std_plugins::SystemTimePlugin> plugin;
  std::shared_ptr<CaptureBuffer> out;
  rclcpp::Node::SharedPtr client;
  rclcpp::Subscription<mavros_msgs::msg::TimesyncStatus>::SharedPtr status_sub;
  std::mutex status_mutex;
  std::vector<mavros_msgs::msg::TimesyncStatus> statuses;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(FCU_SYSTEM, FCU_COMPONENT);

    plugin = std::make_shared<mavros::std_plugins::SystemTimePlugin>(uas.get());
    register_plugin(uas, plugin);

    add_node(uas);
    add_node(plugin->get_node());
    client = make_client_node("sys_time_client");
    status_sub = client->create_subscription<mavros_msgs::msg::TimesyncStatus>(
      "/test_mavros_uas/timesync_status", rclcpp::SensorDataQoS(),
      [this](const mavros_msgs::msg::TimesyncStatus::SharedPtr m) {
        std::lock_guard<std::mutex> lock(status_mutex);
        statuses.push_back(*m);
      });
    out = capture_sink(uas);
    start_spin();
  }

  void TearDown() override
  {
    stop_spin();
  }

  void inject(const TIMESYNC & m, uint8_t sysid, uint8_t compid)
  {
    auto mmsg = convert_message(m);
    mmsg.sysid = sysid;
    mmsg.compid = compid;
    route(uas, &mmsg, mavconn::Framing::ok);
  }

  static TIMESYNC timesync(int64_t tc1, int64_t ts1, uint8_t tgt_sys = 0, uint8_t tgt_comp = 0)
  {
    TIMESYNC m{};
    m.tc1 = tc1;
    m.ts1 = ts1;
    m.target_system = tgt_sys;
    m.target_component = tgt_comp;
    return m;
  }

  static TIMESYNC decode(const mavros_msgs::msg::Mavlink & rmsg)
  {
    auto m = TestUAS::to_mavlink(rmsg);
    TIMESYNC out{};
    mavlink::MsgMap map(m);
    out.deserialize(map);
    return out;
  }

  std::vector<TIMESYNC> sent()
  {
    std::vector<TIMESYNC> ret;
    for (const auto & m : out->snapshot()) {
      if (m.msgid == TIMESYNC::MSG_ID) {
        ret.push_back(decode(m));
      }
    }
    return ret;
  }

  size_t status_count()
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    return statuses.size();
  }

  // The plugin handles an injected message before inject() returns, and its
  // replies and statuses arrive in that order. So the tests check that a message
  // was ignored by sending a valid one after it: once the valid one's result
  // arrives, a result from the ignored one would already be there before it.

  //! Wait for n TIMESYNC replies (tc1 > 0) sent by the plugin; return those sent.
  std::vector<TIMESYNC> wait_replies(size_t n)
  {
    std::vector<TIMESYNC> replies;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    do {
      replies.clear();
      for (const auto & m : sent()) {
        if (m.tc1 > 0) {
          replies.push_back(m);
        }
      }
      if (replies.size() >= n) {
        break;
      }
      std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return replies;
  }

  //! Wait for n timesync statuses; return the remote timestamp of each published.
  std::vector<uint64_t> wait_status_stamps(size_t n)
  {
    wait_status_count(n);
    std::lock_guard<std::mutex> lock(status_mutex);
    std::vector<uint64_t> stamps;
    for (const auto & st : statuses) {
      stamps.push_back(st.remote_timestamp_ns);
    }
    return stamps;
  }

  //! Wait for a request sent after the one with the given ts1.
  TIMESYNC wait_request_after(int64_t ts1)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      for (const auto & m : sent()) {
        if (m.tc1 == 0 && m.ts1 > ts1) {
          return m;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    ADD_FAILURE() << "no request after ts1 " << ts1;
    return TIMESYNC{};
  }

  //! Start requesting and return the first request sent.
  TIMESYNC first_request()
  {
    plugin->get_node()->set_parameter(rclcpp::Parameter("timesync_rate", 20.0));
    auto rmsg = wait_msgid(*out, TIMESYNC::MSG_ID);
    EXPECT_EQ(rmsg.msgid, TIMESYNC::MSG_ID);
    return decode(rmsg);
  }

  //! Publish sim time until the plugin's clock reads it.
  void set_sim_time(const rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr & pub, int sec)
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock.sec = sec;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (plugin->get_node()->now().seconds() != sec &&
      std::chrono::steady_clock::now() < deadline)
    {
      pub->publish(clock);
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_EQ(plugin->get_node()->now().seconds(), sec);
  }

  //! Wait for a request whose ts1 is below the given value.
  bool wait_request_below(int64_t ts1)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      for (const auto & m : sent()) {
        if (m.tc1 == 0 && m.ts1 < ts1) {
          return true;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  void wait_status_count(size_t n)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (status_count() < n && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }
};

TEST_F(SysTimeTest, requests_are_addressed_to_the_fcu)
{
  auto request = first_request();
  EXPECT_EQ(request.tc1, 0);
  EXPECT_GT(request.ts1, 0);
  EXPECT_EQ(request.target_system, FCU_SYSTEM);
  EXPECT_EQ(request.target_component, FCU_COMPONENT);
}

TEST_F(SysTimeTest, replies_are_addressed_to_the_requester)
{
  inject(timesync(0, 1234), FCU_SYSTEM, OTHER_COMPONENT);

  auto replies = wait_replies(1);
  ASSERT_EQ(replies.size(), 1u);
  EXPECT_GT(replies[0].tc1, 0);
  EXPECT_EQ(replies[0].ts1, 1234);
  EXPECT_EQ(replies[0].target_system, FCU_SYSTEM);
  EXPECT_EQ(replies[0].target_component, OTHER_COMPONENT);
}

TEST_F(SysTimeTest, requests_for_another_component_are_not_answered)
{
  inject(timesync(0, 1234, FCU_SYSTEM, OTHER_COMPONENT), FCU_SYSTEM, FCU_COMPONENT);
  // A broadcast request is answered; it is the only reply.
  inject(timesync(0, 5678), FCU_SYSTEM, FCU_COMPONENT);

  auto replies = wait_replies(1);
  ASSERT_EQ(replies.size(), 1u);
  EXPECT_EQ(replies[0].ts1, 5678);
}

TEST_F(SysTimeTest, only_the_fcus_reply_to_our_request_is_used)
{
  auto request = first_request();

  // A reply to someone else's request (another ts1), and our ts1 echoed by
  // another component: neither is ours.
  inject(timesync(1'000'000'000, request.ts1 + 7), FCU_SYSTEM, FCU_COMPONENT);
  inject(timesync(2'000'000'000, request.ts1), FCU_SYSTEM, OTHER_COMPONENT);
  // The FCU's reply, without target fields as ArduPilot sends it.
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);

  auto stamps = wait_status_stamps(1);
  ASSERT_EQ(stamps.size(), 1u);
  EXPECT_EQ(stamps[0], 5'000'000'000u);

  // The same reply again (e.g. a router duplicate) answers nothing pending; the
  // FCU's reply to a later request does.
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  auto later = wait_request_after(request.ts1);
  inject(timesync(7'000'000'000, later.ts1), FCU_SYSTEM, FCU_COMPONENT);

  stamps = wait_status_stamps(2);
  ASSERT_EQ(stamps.size(), 2u);
  EXPECT_EQ(stamps[1], 7'000'000'000u);
}

TEST_F(SysTimeTest, replies_addressed_to_another_component_are_ignored)
{
  auto request = first_request();
  inject(
    timesync(1'000'000'000, request.ts1, FCU_SYSTEM, OTHER_COMPONENT),
    FCU_SYSTEM, FCU_COMPONENT);
  // Had the first been used, it would have taken the pending request.
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);

  auto stamps = wait_status_stamps(1);
  ASSERT_EQ(stamps.size(), 1u);
  EXPECT_EQ(stamps[0], 5'000'000'000u);
}

TEST_F(SysTimeTest, target_component_zero_accepts_any_component)
{
  uas->set_tgt(FCU_SYSTEM, 0);
  auto request = first_request();
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  EXPECT_EQ(wait_status_stamps(1), std::vector<uint64_t>{5'000'000'000u});
}

TEST_F(SysTimeTest, sync_resumes_after_the_clock_steps_back)
{
  // With sim time a SITL restart steps our clock back; an NTP step can too.
  auto clock_pub = client->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  plugin->get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));
  set_sim_time(clock_pub, 100);
  first_request();

  set_sim_time(clock_pub, 50);
  ASSERT_TRUE(wait_request_below(100'000'000'000));
  const int64_t ts1 = 50'000'000'000;
  inject(timesync(5'000'000'000, ts1), FCU_SYSTEM, FCU_COMPONENT);
  EXPECT_EQ(wait_status_stamps(1), std::vector<uint64_t>{5'000'000'000u});
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  TestUAS::Init();
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
