//
// mavros
// Copyright 2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Test mavros uas node
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executors.hpp"
#include "mavconn/interface.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin_filter.hpp"

using ::testing::_;
using ::testing::Return;

using namespace mavros;         // NOLINT
using namespace mavros::uas;    // NOLINT
using namespace mavros::plugin; // NOLINT

using namespace mavconn; // NOLINT
using mavlink_message_t = mavlink::mavlink_message_t;
using msgid_t = mavlink::msgid_t;

namespace mavlink
{
namespace common
{
using namespace mavlink::minimal;     // NOLINT
namespace msg
{
using namespace mavlink::minimal::msg;         // NOLINT
}
}
}  // namespace mavlink

class MockUAS : public UAS
{
public:
  using SharedPtr = std::shared_ptr<MockUAS>;

  explicit MockUAS(const std::string name_)
  : UAS(name_) {}

  MOCK_METHOD1(create_plugin_instance, plugin::Plugin::SharedPtr(const std::string & pl_name));
};

class MockPlugin : public plugin::Plugin
{
public:
  using SharedPtr = std::shared_ptr<MockPlugin>;

  explicit MockPlugin(UAS::SharedPtr uas_)
  : Plugin(uas_.get()) {}

  MOCK_METHOD0(get_subscriptions, plugin::Plugin::Subscriptions(void));

  inline plugin::Plugin::SharedPtr getptr()
  {
    return std::static_pointer_cast<plugin::Plugin>(shared_from_this());
  }

  inline Subscriptions allsubs()
  {
    return {
      make_handler(mavlink::common::msg::STATUSTEXT::MSG_ID, &MockPlugin::handle_statustext_raw),
      make_handler(&MockPlugin::handle_heartbeat_anyok),
      make_handler(&MockPlugin::handle_heartbeat_systemandok),
      make_handler(&MockPlugin::handle_heartbeat_componentandok)
    };
  }

  inline Subscriptions rawsubs()
  {
    return {
      make_handler(mavlink::common::msg::STATUSTEXT::MSG_ID, &MockPlugin::handle_statustext_raw),
    };
  }

  MOCK_METHOD2(handle_statustext_raw, void(const mavlink_message_t * msg, const Framing framing));
  MOCK_METHOD3(
    handle_heartbeat_anyok,
    void(const mavlink_message_t * msg, mavlink::common::msg::HEARTBEAT & hb,
    filter::AnyOk filter));
  MOCK_METHOD3(
    handle_heartbeat_systemandok,
    void(const mavlink_message_t * msg, mavlink::common::msg::HEARTBEAT & hb,
    filter::SystemAndOk filter));
  MOCK_METHOD3(
    handle_heartbeat_componentandok,
    void(const mavlink_message_t * msg, mavlink::common::msg::HEARTBEAT & hb,
    filter::ComponentAndOk filter));
};

namespace mavros
{
namespace uas
{

class TestUAS : public ::testing::Test
{
public:
  TestUAS()
  {
    // std::cout << "init" << std::endl;
    rclcpp::init(0, nullptr);
  }

  ~TestUAS()
  {
    // NOTE(vooon): required to remove any remaining Nodes
    // std::cout << "kill" << std::endl;
    rclcpp::shutdown();
  }

  MockUAS::SharedPtr create_node()
  {
    auto uas = std::make_shared<MockUAS>("test_mavros_uas");
    uas->startup_delay_timer->cancel();
    return uas;
  }

  template<typename TimerT>
  std::function<void()> take_callback(const std::shared_ptr<TimerT> & timer)
  {
    timer->reset();
    // Humble: call() returns bool and execute_callback() takes no arguments.
    // Newer rclcpp: call() returns the call info that execute_callback() consumes.
    // Probe the execute_callback() signature, since call() exists on both.
    if constexpr (requires(std::shared_ptr<void> data) {timer->execute_callback(data);}) {
      auto data = timer->call();
      return [timer, data]() {timer->execute_callback(data);};
    } else {
      return [timer]() {timer->execute_callback();};
    }
  }

  std::function<void()> take_startup_callback(MockUAS::SharedPtr uas)
  {
    return take_callback(uas->startup_delay_timer);
  }

  void set_lists(MockUAS::SharedPtr p, UAS::StrV denylist, UAS::StrV allowlist)
  {
    p->plugin_denylist = denylist;
    p->plugin_allowlist = allowlist;
  }

  bool is_plugin_allowed(MockUAS::SharedPtr p, const std::string pl_name)
  {
    return p->is_plugin_allowed(pl_name);
  }

  void add_plugin(MockUAS::SharedPtr p, const std::string & pl_name)
  {
    p->add_plugin(pl_name);
  }

  void plugin_route(MockUAS::SharedPtr p, const mavlink_message_t * msg, const Framing framing)
  {
    p->plugin_route(msg, framing);
  }

  void recv_message(MockUAS::SharedPtr p, const mavlink_message_t & msg, const Framing framing)
  {
    auto rmsg = std::make_unique<mavros_msgs::msg::Mavlink>();
    ASSERT_TRUE(mavros_msgs::mavlink::convert(msg, *rmsg, utils::enum_value(framing)));
    p->recv_message(std::move(rmsg));
  }

  void recv_raw_ros_message(MockUAS::SharedPtr p, mavros_msgs::msg::Mavlink::UniquePtr rmsg)
  {
    p->recv_message(std::move(rmsg));
  }

  static bool is_fast_lane_msgid(uint32_t msgid)
  {
    return UAS::is_fast_lane_msgid(msgid);
  }

  mavlink_message_t convert_message(const mavlink::Message & msg, const uint16_t source = 0x0101)
  {
    mavlink_message_t ret;
    mavlink::MsgMap map(ret);
    ret.sysid = source >> 8;
    ret.compid = source & 0xFF;
    msg.serialize(map);

    return ret;
  }

  mavlink::common::msg::HEARTBEAT make_heartbeat()
  {
    using mavlink::common::MAV_AUTOPILOT;
    using mavlink::common::MAV_MODE;
    using mavlink::common::MAV_STATE;
    using mavlink::common::MAV_TYPE;

    mavlink::common::msg::HEARTBEAT hb = {};
    hb.type = static_cast<int>(MAV_TYPE::ONBOARD_CONTROLLER);
    hb.autopilot = static_cast<int>(MAV_AUTOPILOT::INVALID);
    hb.base_mode = static_cast<int>(MAV_MODE::MANUAL_ARMED);
    hb.custom_mode = 0;
    hb.system_status = static_cast<int>(MAV_STATE::ACTIVE);

    return hb;
  }
};

TEST_F(TestUAS, startup_callback_after_context_shutdown)
{
  auto uas = create_node();
  // Keep the baseline path from invoking an unconfigured mock plugin factory.
  uas->set_parameter(rclcpp::Parameter("plugin_denylist", UAS::StrV{"*"}));
  auto callback = take_startup_callback(uas);
  rclcpp::shutdown();
  EXPECT_NO_THROW(callback());
}

TEST_F(TestUAS, startup_shutdown_race_does_not_escape)
{
  auto uas = create_node();
  EXPECT_CALL(*uas, create_plugin_instance(_)).WillOnce(
    ::testing::InvokeWithoutArgs([]() -> plugin::Plugin::SharedPtr {
      rclcpp::shutdown();
      throw std::runtime_error("ROS resource creation interrupted by shutdown");
    }));
  auto callback = take_startup_callback(uas);
  EXPECT_NO_THROW(callback());
}

TEST_F(TestUAS, startup_error_with_valid_context_is_not_hidden)
{
  auto uas = create_node();
  EXPECT_CALL(*uas, create_plugin_instance(_)).WillOnce(
    ::testing::InvokeWithoutArgs([]() -> plugin::Plugin::SharedPtr {
      throw std::runtime_error("invalid startup configuration");
    }));
  auto callback = take_startup_callback(uas);
  EXPECT_THROW(callback(), std::runtime_error);
  EXPECT_TRUE(rclcpp::ok());
  // Stop the context before local node destruction, even if its thread is still starting.
  rclcpp::shutdown();
}

TEST_F(TestUAS, is_plugin_allowed)
{
  auto uas = create_node();

  // by default all allowed
  set_lists(uas, {}, {});
  EXPECT_EQ(true, is_plugin_allowed(uas, "test1"));
  EXPECT_EQ(true, is_plugin_allowed(uas, "test2"));

  // check denylist
  set_lists(uas, {"test1", "prefix*", "*suffix"}, {});
  EXPECT_EQ(false, is_plugin_allowed(uas, "test1"));
  EXPECT_EQ(true, is_plugin_allowed(uas, "test2"));
  EXPECT_EQ(false, is_plugin_allowed(uas, "prefix_test"));
  EXPECT_EQ(false, is_plugin_allowed(uas, "test_suffix"));

  // check allowlist
  set_lists(uas, {"*"}, {"test1", "prefix*", "*suffix"});
  EXPECT_EQ(true, is_plugin_allowed(uas, "test1"));
  EXPECT_EQ(false, is_plugin_allowed(uas, "test2"));
  EXPECT_EQ(true, is_plugin_allowed(uas, "prefix_test"));
  EXPECT_EQ(true, is_plugin_allowed(uas, "test_suffix"));
}

TEST_F(TestUAS, add_plugin__route_message__filter)
{
  auto uas = create_node();
  auto plugin1 = std::make_shared<MockPlugin>(uas);
  auto subs1 = plugin1->allsubs();
  auto plugin2 = std::make_shared<MockPlugin>(uas);
  auto subs2 = plugin2->rawsubs();

  // XXX(vooon): silence leak warnings: they work badly with shared_ptr
  testing::Mock::AllowLeak(&(*uas));
  testing::Mock::AllowLeak(&(*plugin1));
  testing::Mock::AllowLeak(&(*plugin2));

  mavlink::common::msg::STATUSTEXT stxt{};
  auto m_stxt = convert_message(stxt, 0x0000);

  auto hb = make_heartbeat();
  auto m_hb11 = convert_message(hb, 0x0101);
  auto m_hb12 = convert_message(hb, 0x0102);
  auto m_hb21 = convert_message(hb, 0x0201);

  EXPECT_CALL(*uas, create_plugin_instance("test1")).WillRepeatedly(Return(plugin1));
  EXPECT_CALL(*uas, create_plugin_instance("test2")).WillRepeatedly(Return(plugin2));
  EXPECT_CALL(*plugin1, get_subscriptions()).WillRepeatedly(Return(subs1));
  EXPECT_CALL(*plugin2, get_subscriptions()).WillRepeatedly(Return(subs2));

  add_plugin(uas, "test1");
  add_plugin(uas, "test2");

  EXPECT_CALL(*plugin1, handle_statustext_raw).Times(2);
  EXPECT_CALL(*plugin2, handle_statustext_raw).Times(2);
  EXPECT_CALL(*plugin1, handle_heartbeat_anyok).Times(3);
  EXPECT_CALL(*plugin1, handle_heartbeat_systemandok).Times(2);
  EXPECT_CALL(*plugin1, handle_heartbeat_componentandok).Times(1);

  plugin_route(uas, &m_stxt, Framing::ok);
  plugin_route(uas, &m_stxt, Framing::bad_crc);
  plugin_route(uas, &m_hb21, Framing::ok);          // AnyOk
  plugin_route(uas, &m_hb12, Framing::ok);          // AnyOk, SystemAndOk
  plugin_route(uas, &m_hb11, Framing::ok);          // AnyOk, SystemAndOk, ComponentAndOk
  plugin_route(uas, &m_hb11, Framing::bad_crc);     // none

  testing::Mock::VerifyAndClearExpectations(&(*uas));
  testing::Mock::VerifyAndClearExpectations(&(*plugin1));
  testing::Mock::VerifyAndClearExpectations(&(*plugin2));
}

// UAS::is_addressed_to_us: target ids are ours or broadcast (0).
TEST_F(TestUAS, is_addressed_to_us)
{
  auto uas = create_node();
  testing::Mock::AllowLeak(&(*uas));

  const uint8_t sys = uas->get_system_id();
  const uint8_t comp = uas->get_component_id();

  EXPECT_TRUE(uas->is_addressed_to_us(sys, comp));
  EXPECT_TRUE(uas->is_addressed_to_us(0, 0));
  EXPECT_TRUE(uas->is_addressed_to_us(sys, 0));
  EXPECT_TRUE(uas->is_addressed_to_us(0, comp));
  EXPECT_FALSE(uas->is_addressed_to_us(sys + 1, comp));
  EXPECT_FALSE(uas->is_addressed_to_us(sys, comp + 1));
  EXPECT_FALSE(uas->is_addressed_to_us(sys + 1, 0));
}

TEST_F(TestUAS, synchronise_stamp_negative_offset)
{
  auto uas = create_node();

  // Scenario: FCU was on boot time, so offset is approx current epoch (e.g. 1.7e9 seconds)
  int64_t offset_ns = 1700000000LL * 1000000000LL;
  uas->set_time_offset(offset_ns);

  // FCU jumps to GPS time (also approx 1.7e9 seconds)
  // Use uint64_t version of synchronise_stamp
  uint64_t time_usec = 1700000000ULL * 1000000ULL;

  // Total time = 3.4e9 seconds.
  // This overflows int32_t (max 2.14e9), becoming negative.
  // rclcpp::Time(int32_t, uint32_t) will see a negative second and throw.

  // This should not throw (once fixed)
  EXPECT_NO_THROW(
      {
        rclcpp::Time stamp = uas->synchronise_stamp(time_usec);
      });

  // ... and must not publish a stamp whose int32 seconds overflow,
  // it falls back to the current time
  builtin_interfaces::msg::Time msg = uas->synchronise_stamp(time_usec);
  EXPECT_GE(msg.sec, 0);
}

TEST_F(TestUAS, synchronise_stamp_small_negative_offset)
{
  auto uas = create_node();

  // Scenario: PX4 SITL in lockstep, the FCU clock is 5 ms ahead of ROS sim time
  uas->set_time_offset(-5000000);

  EXPECT_EQ(uas->synchronise_stamp(uint32_t{1000}).nanoseconds(), 995000000);
  EXPECT_EQ(uas->synchronise_stamp(uint64_t{1000000}).nanoseconds(), 995000000);
}

TEST_F(TestUAS, synchronise_stamp_out_of_range_falls_back_to_now)
{
  auto uas = create_node();

  // a stamp below zero (FCU just booted) and one beyond int32 seconds both fall back to now()
  uas->set_time_offset(-5000000);
  EXPECT_NEAR(uas->synchronise_stamp(uint32_t{1}).seconds(), uas->now().seconds(), 1.0);

  uas->set_time_offset(INT64_C(3000000000) * 1000000000);
  EXPECT_NEAR(uas->synchronise_stamp(uint32_t{1000}).seconds(), uas->now().seconds(), 1.0);
  EXPECT_NEAR(uas->synchronise_stamp(uint64_t{1000000}).seconds(), uas->now().seconds(), 1.0);

  // a corrupt time_usec must not overflow
  EXPECT_NEAR(
    uas->synchronise_stamp(std::numeric_limits<uint64_t>::max()).seconds(),
    uas->now().seconds(), 1.0);
}

TEST_F(TestUAS, fast_lane_bypasses_bulk_queue_and_concurrent_send)
{
  EXPECT_TRUE(is_fast_lane_msgid(mavlink::common::msg::SYSTEM_TIME::MSG_ID));
  EXPECT_TRUE(is_fast_lane_msgid(mavlink::common::msg::ATTITUDE::MSG_ID));
  EXPECT_TRUE(is_fast_lane_msgid(mavlink::common::msg::ATTITUDE_QUATERNION::MSG_ID));
  EXPECT_TRUE(is_fast_lane_msgid(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID));
  EXPECT_TRUE(is_fast_lane_msgid(mavlink::common::msg::LOCAL_POSITION_NED_COV::MSG_ID));
  EXPECT_TRUE(is_fast_lane_msgid(mavlink::common::msg::COMMAND_ACK::MSG_ID));
  EXPECT_TRUE(is_fast_lane_msgid(mavlink::common::msg::TIMESYNC::MSG_ID));
  EXPECT_TRUE(is_fast_lane_msgid(mavlink::common::msg::ODOMETRY::MSG_ID));
  EXPECT_FALSE(is_fast_lane_msgid(mavlink::common::msg::HEARTBEAT::MSG_ID));
  EXPECT_FALSE(is_fast_lane_msgid(mavlink::common::msg::STATUSTEXT::MSG_ID));
  EXPECT_FALSE(is_fast_lane_msgid(mavlink::common::msg::PARAM_VALUE::MSG_ID));

  struct LanePlugin : public plugin::Plugin
  {
    explicit LanePlugin(UAS::SharedPtr uas_)
    : Plugin(uas_.get()) {}

    Subscriptions get_subscriptions() override
    {
      return {
        make_handler(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, &LanePlugin::on_fast),
        make_handler(mavlink::common::msg::STATUSTEXT::MSG_ID, &LanePlugin::on_bulk),
      };
    }

    void on_fast(const mavlink_message_t *, const Framing)
    {
      fast_count.fetch_add(1, std::memory_order_relaxed);
    }

    void on_bulk(const mavlink_message_t *, const Framing)
    {
      if (!first_bulk_entered.exchange(true)) {
        while (!release_bulk.load(std::memory_order_acquire)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
      bulk_count.fetch_add(1, std::memory_order_relaxed);
    }

    std::atomic<size_t> fast_count{0};
    std::atomic<size_t> bulk_count{0};
    std::atomic<bool> first_bulk_entered{false};
    std::atomic<bool> release_bulk{false};
  };

  auto uas = create_node();
  auto lane_plugin = std::make_shared<LanePlugin>(uas);
  testing::Mock::AllowLeak(&(*uas));

  uas->set_parameter(rclcpp::Parameter("uas_url", "/uas_test_lane"));
  uas->set_parameter(rclcpp::Parameter("plugin_allowlist", UAS::StrV{"lane_test"}));
  uas->set_parameter(rclcpp::Parameter("rt_sched_fifo_priority", 0));
  EXPECT_CALL(*uas, create_plugin_instance("lane_test")).WillRepeatedly(Return(lane_plugin));
  auto startup_cb = take_startup_callback(uas);
  startup_cb();
  add_plugin(uas, "lane_test");

  mavlink::common::msg::STATUSTEXT stxt{};
  auto m_bulk = convert_message(stxt, 0x0101);
  mavlink::common::msg::LOCAL_POSITION_NED lpn{};
  auto m_fast = convert_message(lpn, 0x0101);

  // Enqueue one bulk message and wait until the bulk worker enters the handler and blocks.
  recv_message(uas, m_bulk, Framing::ok);
  const auto enter_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!lane_plugin->first_bulk_entered.load(std::memory_order_acquire)) {
    if (std::chrono::steady_clock::now() >= enter_deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(lane_plugin->first_bulk_entered.load());

  // Flood bulk queue beyond capacity while bulk worker is blocked, and concurrently send outbound
  // frames from multiple threads.
  std::vector<std::thread> senders;
  for (int t = 0; t < 4; ++t) {
    senders.emplace_back(
      [&]() {
        auto hb = make_heartbeat();
        for (int i = 0; i < 50; ++i) {
          uas->send_message(hb);
        }
      });
  }
  for (int i = 0; i < 1200; ++i) {
    recv_message(uas, m_bulk, Framing::ok);
  }

  // Fast-lane messages must execute immediately on the caller without waiting for bulk_rx_thd.
  for (int i = 0; i < 50; ++i) {
    recv_message(uas, m_fast, Framing::ok);
  }
  EXPECT_EQ(lane_plugin->fast_count.load(), 50U);
  EXPECT_EQ(lane_plugin->bulk_count.load(), 0U);

  // Release bulk worker and join concurrent senders.
  lane_plugin->release_bulk.store(true, std::memory_order_release);
  for (auto & th : senders) {
    th.join();
  }

  const auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (lane_plugin->bulk_count.load() < 1025U) {
    if (std::chrono::steady_clock::now() >= drain_deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  // 1 in-flight + 1024 queued max capacity.
  EXPECT_EQ(lane_plugin->bulk_count.load(), 1025U);

  // Malformed Mavlink ROS message (invalid signature size != 13) must be dropped without throwing.
  auto bad_rmsg = std::make_unique<mavros_msgs::msg::Mavlink>();
  bad_rmsg->signature.resize(5, 0);
  EXPECT_NO_THROW(recv_raw_ros_message(uas, std::move(bad_rmsg)));
  EXPECT_NO_THROW(recv_raw_ros_message(uas, nullptr));
}

TEST_F(TestUAS, stress_concurrent_fast_bulk_exception_and_state_transitions)
{
  struct StressPlugin : public plugin::Plugin
  {
    explicit StressPlugin(UAS::SharedPtr uas_)
    : Plugin(uas_.get())
    {
      enable_connection_cb();
      enable_capabilities_cb();
    }

    Subscriptions get_subscriptions() override
    {
      return {
        make_handler(mavlink::common::msg::ATTITUDE::MSG_ID, &StressPlugin::on_fast_att),
        make_handler(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, &StressPlugin::on_fast_lpn),
        make_handler(mavlink::common::msg::HEARTBEAT::MSG_ID, &StressPlugin::on_bulk_hb),
        make_handler(mavlink::common::msg::DEBUG::MSG_ID, &StressPlugin::on_throwing),
      };
    }

    void on_fast_att(const mavlink_message_t *, const Framing)
    {
      fast_att.fetch_add(1, std::memory_order_relaxed);
    }

    void on_fast_lpn(const mavlink_message_t *, const Framing)
    {
      fast_lpn.fetch_add(1, std::memory_order_relaxed);
    }

    void on_bulk_hb(const mavlink_message_t *, const Framing)
    {
      bulk_hb.fetch_add(1, std::memory_order_relaxed);
    }

    void on_throwing(const mavlink_message_t *, const Framing)
    {
      throw_calls.fetch_add(1, std::memory_order_relaxed);
      throw std::runtime_error("simulated plugin handler failure");
    }

    void connection_cb(bool) override
    {
      conn_transitions.fetch_add(1, std::memory_order_relaxed);
    }

    void capabilities_cb(uas::MAV_CAP) override
    {
      cap_transitions.fetch_add(1, std::memory_order_relaxed);
    }

    std::atomic<size_t> fast_att{0};
    std::atomic<size_t> fast_lpn{0};
    std::atomic<size_t> bulk_hb{0};
    std::atomic<size_t> throw_calls{0};
    std::atomic<size_t> conn_transitions{0};
    std::atomic<size_t> cap_transitions{0};
  };

  auto uas = create_node();
  auto sp = std::make_shared<StressPlugin>(uas);
  testing::Mock::AllowLeak(&(*uas));

  uas->set_parameter(rclcpp::Parameter("uas_url", "/uas_test_stress"));
  uas->set_parameter(rclcpp::Parameter("plugin_allowlist", UAS::StrV{"stress_test"}));
  uas->set_parameter(rclcpp::Parameter("rt_sched_fifo_priority", 0));
  EXPECT_CALL(*uas, create_plugin_instance("stress_test")).WillRepeatedly(Return(sp));
  auto startup_cb = take_startup_callback(uas);
  startup_cb();
  add_plugin(uas, "stress_test");

  mavlink::common::msg::ATTITUDE att{};
  auto m_att = convert_message(att, 0x0101);
  mavlink::common::msg::LOCAL_POSITION_NED lpn{};
  auto m_lpn = convert_message(lpn, 0x0101);
  auto hb = make_heartbeat();
  auto m_hb = convert_message(hb, 0x0101);
  mavlink::common::msg::DEBUG dbg{};
  auto m_dbg = convert_message(dbg, 0x0101);

  constexpr int kThreads = 6;
  constexpr int kIters = 300;
  std::vector<std::thread> workers;
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back(
      [&, t]() {
        for (int i = 0; i < kIters; ++i) {
          recv_message(uas, m_att, Framing::ok);
          recv_message(uas, m_lpn, Framing::ok);
          recv_message(uas, m_hb, Framing::ok);
          if ((i % 10) == 0) {
            recv_message(uas, m_dbg, Framing::ok);
          }
          if ((i % 25) == 0) {
            uas->send_message(hb, static_cast<uint8_t>(190 + t));
            uas->update_connection_status((i % 50) == 0);
            uas->update_capabilities(true, static_cast<uint64_t>(i + 1));
            (void)uas->synchronise_stamp(static_cast<uint32_t>(i * 10));
          }
        }
      });
  }

  for (auto & w : workers) {
    w.join();
  }

  EXPECT_EQ(sp->fast_att.load(), static_cast<size_t>(kThreads * kIters));
  EXPECT_EQ(sp->fast_lpn.load(), static_cast<size_t>(kThreads * kIters));

  // Wait for bulk queue to drain.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sp->throw_calls.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GT(sp->bulk_hb.load(), 0U);
  EXPECT_GT(sp->throw_calls.load(), 0U);
  EXPECT_GT(sp->conn_transitions.load(), 0U);
  EXPECT_GT(sp->cap_transitions.load(), 0U);
}

}  // namespace uas
}  // namespace mavros

int main(int argc, char ** argv)
{
  // rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
