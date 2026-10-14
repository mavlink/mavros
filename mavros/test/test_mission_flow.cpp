//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Service-based tests for the waypoint (mission) plugin ROS API.
 *
 * The plugin under test is compiled into this TU by including its source
 * (plus the MissionBase implementation), so the full service/publisher surface
 * is exercised over real ROS interfaces with a simulated FCU that answers the
 * mission protocol handshake.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include "test_plugin_helpers.hpp"
#include "../src/plugins/mission_protocol_base.cpp"  // NOLINT(build/include)
#include "../src/plugins/waypoint.cpp"               // NOLINT(build/include)

using namespace std::chrono_literals;  // NOLINT

using mavros::utils::enum_value;
using mavlink::common::MAV_CMD;
using mavlink::common::MAV_MISSION_RESULT;
using mavlink::common::MAV_MISSION_TYPE;
using mavlink::common::msg::MISSION_ACK;
using mavlink::common::msg::MISSION_COUNT;
using mavlink::common::msg::MISSION_ITEM_INT;
using mavlink::common::msg::MISSION_REQUEST_INT;
using mavlink::common::msg::MISSION_REQUEST_LIST;
using mavros_msgs::msg::Waypoint;
using mavros_msgs::srv::WaypointClear;
using mavros_msgs::srv::WaypointPull;
using mavros_msgs::srv::WaypointPush;
using mavros_msgs::srv::WaypointSetCurrent;

namespace mavros
{
namespace uas
{

class MissionFlowTest : public TestUAS
{
public:
  UAS::SharedPtr uas;
  std::shared_ptr<mavros::std_plugins::WaypointPlugin> plugin;
  std::shared_ptr<std::vector<mavros_msgs::msg::Mavlink>> out;
  rclcpp::Node::SharedPtr client;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(1, 1);

    plugin = std::make_shared<mavros::std_plugins::WaypointPlugin>(uas.get());
    register_plugin(uas, plugin);

    // Deterministic fast timeouts so failure paths do not hang.
    auto pn = plugin->get_node();
    pn->set_parameter(rclcpp::Parameter("mission_wp_timeout", 0.2));
    pn->set_parameter(rclcpp::Parameter("mission_list_timeout", 1.0));
    pn->set_parameter(rclcpp::Parameter("mission_retries", 1));
    pn->set_parameter(rclcpp::Parameter("use_mission_item_int", true));
    pn->set_parameter(rclcpp::Parameter("enable_partial_push", true));

    add_node(uas);
    add_node(plugin->get_node());
    client = make_client_node("mission_client");
    out = capture_sink(uas);
    start_spin();
  }

  void TearDown() override
  {
    stop_spin();
  }

  /* -*- FCU-side message builders -*- */

  // Another client sharing the FCU link (arbitrary ids, neither ours nor the
  // FCU's); its transfers are routed to us as well.
  static constexpr uint8_t OTHER_SYSTEM = 200;
  static constexpr uint8_t OTHER_COMPONENT = 100;

  // The FCU addresses its side of a transfer to whoever started it: us, or
  // (to_us == false) another client.
  template<class MsgT>
  void set_target(MsgT & m, bool to_us)
  {
    m.target_system = to_us ? uas->get_system_id() : OTHER_SYSTEM;
    m.target_component = to_us ? uas->get_component_id() : OTHER_COMPONENT;
  }

  void inject(const mavlink::mavlink_message_t & mmsg)
  {
    route(uas, &mmsg, mavconn::Framing::ok);
    std::this_thread::sleep_for(10ms);
  }

  mavlink::mavlink_message_t fcu_count(uint16_t count, bool to_us = true, uint8_t sysid = 1)
  {
    MISSION_COUNT m{};
    m.mission_type = enum_value(MAV_MISSION_TYPE::MISSION);
    m.count = count;
    set_target(m, to_us);
    auto mmsg = convert_message(m);
    mmsg.sysid = sysid;
    mmsg.compid = 1;
    mmsg.msgid = MISSION_COUNT::MSG_ID;
    return mmsg;
  }

  mavlink::mavlink_message_t fcu_item_int(uint16_t seq, bool to_us = true, uint8_t sysid = 1)
  {
    MISSION_ITEM_INT m{};
    m.mission_type = enum_value(MAV_MISSION_TYPE::MISSION);
    m.seq = seq;
    m.frame = enum_value(mavlink::common::MAV_FRAME::GLOBAL_RELATIVE_ALT_INT);
    m.command = enum_value(MAV_CMD::NAV_WAYPOINT);
    m.x = 10000000 * seq;
    m.y = to_us ? 20000000 : -20000000;
    m.z = 10.0f;
    set_target(m, to_us);
    auto mmsg = convert_message(m);
    mmsg.sysid = sysid;
    mmsg.compid = 1;
    mmsg.msgid = MISSION_ITEM_INT::MSG_ID;
    return mmsg;
  }

  mavlink::mavlink_message_t fcu_request_int(uint16_t seq, bool to_us = true, uint8_t sysid = 1)
  {
    MISSION_REQUEST_INT m{};
    m.mission_type = enum_value(MAV_MISSION_TYPE::MISSION);
    m.seq = seq;
    set_target(m, to_us);
    auto mmsg = convert_message(m);
    mmsg.sysid = sysid;
    mmsg.compid = 1;
    mmsg.msgid = MISSION_REQUEST_INT::MSG_ID;
    return mmsg;
  }

  mavlink::mavlink_message_t fcu_ack(
    MAV_MISSION_RESULT type, bool to_us = true, uint8_t sysid = 1)
  {
    MISSION_ACK m{};
    m.mission_type = enum_value(MAV_MISSION_TYPE::MISSION);
    m.type = enum_value(type);
    set_target(m, to_us);
    auto mmsg = convert_message(m);
    mmsg.sysid = sysid;
    mmsg.compid = 1;
    mmsg.msgid = MISSION_ACK::MSG_ID;
    return mmsg;
  }

  mavlink::mavlink_message_t fcu_current(uint16_t seq)
  {
    mavlink::common::msg::MISSION_CURRENT m{};
    m.seq = seq;
    auto mmsg = convert_message(m);
    mmsg.sysid = 1;
    mmsg.compid = 1;
    mmsg.msgid = mavlink::common::msg::MISSION_CURRENT::MSG_ID;
    return mmsg;
  }

  Waypoint make_wp(uint16_t command, float x = 0, float y = 0, float z = 0)
  {
    Waypoint wp{};
    wp.frame = enum_value(mavlink::common::MAV_FRAME::GLOBAL_RELATIVE_ALT_INT);
    wp.command = command;
    wp.is_current = false;
    wp.autocontinue = true;
    wp.x_lat = x;
    wp.y_long = y;
    wp.z_alt = z;
    return wp;
  }
};

// Full pull handshake: FCU answers with a count and all items, plugin publishes
// the waypoint list and reports success with the received count.
TEST_F(MissionFlowTest, pull_success)
{
  auto pull_client = client->create_client<WaypointPull>("/test_mavros_uas/mission/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointPull::Request>();
  std::promise<WaypointPull::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});

  auto rl = wait_msgid(*out, MISSION_REQUEST_LIST::MSG_ID);
  EXPECT_EQ(rl.msgid, MISSION_REQUEST_LIST::MSG_ID);

  inject(fcu_count(2));
  inject(fcu_item_int(0));
  inject(fcu_item_int(1));

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->wp_received, 2u);
}

// Full push handshake: plugin sends count then items in response to requests,
// FCU ack finishes the transfer.
TEST_F(MissionFlowTest, push_success)
{
  auto push_client = client->create_client<WaypointPush>("/test_mavros_uas/mission/push");
  ASSERT_TRUE(push_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointPush::Request>();
  req->start_index = 0;
  req->waypoints.push_back(make_wp(enum_value(MAV_CMD::NAV_WAYPOINT), 1, 2, 3));
  req->waypoints.push_back(make_wp(enum_value(MAV_CMD::NAV_WAYPOINT), 4, 5, 6));

  std::promise<WaypointPush::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(push_client, req, 10s));});

  auto cnt = wait_msgid(*out, MISSION_COUNT::MSG_ID);
  EXPECT_EQ(cnt.msgid, MISSION_COUNT::MSG_ID);

  inject(fcu_request_int(0));
  auto item0 = wait_msgid(*out, MISSION_ITEM_INT::MSG_ID);
  EXPECT_EQ(item0.msgid, MISSION_ITEM_INT::MSG_ID);

  inject(fcu_request_int(1));
  // wait for a second MISSION_ITEM_INT
  EXPECT_NE(wait_msgid(*out, MISSION_ITEM_INT::MSG_ID), mavros_msgs::msg::Mavlink{});

  inject(fcu_ack(MAV_MISSION_RESULT::ACCEPTED));

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->wp_transfered, 2u);
}

TEST_F(MissionFlowTest, clear_success)
{
  auto clear_client = client->create_client<WaypointClear>("/test_mavros_uas/mission/clear");
  ASSERT_TRUE(clear_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointClear::Request>();
  std::promise<WaypointClear::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(clear_client, req, 10s));});

  EXPECT_EQ(
    wait_msgid(*out, mavlink::common::msg::MISSION_CLEAR_ALL::MSG_ID).msgid,
    mavlink::common::msg::MISSION_CLEAR_ALL::MSG_ID);

  inject(fcu_ack(MAV_MISSION_RESULT::ACCEPTED));

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
}

TEST_F(MissionFlowTest, set_current_success)
{
  auto set_client =
    client->create_client<WaypointSetCurrent>("/test_mavros_uas/mission/set_current");
  ASSERT_TRUE(set_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointSetCurrent::Request>();
  req->wp_seq = 3;
  std::promise<WaypointSetCurrent::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(set_client, req, 10s));});

  EXPECT_NE(
    wait_msgid(*out, mavlink::common::msg::MISSION_SET_CURRENT::MSG_ID),
    mavros_msgs::msg::Mavlink{});

  mavlink::common::msg::MISSION_CURRENT mc{};
  mc.seq = 3;
  auto mmsg = convert_message(mc);
  mmsg.sysid = 1;
  mmsg.compid = 1;
  mmsg.msgid = mavlink::common::msg::MISSION_CURRENT::MSG_ID;
  inject(mmsg);

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
}

// Pull timeout: FCU never responds to MISSION_REQUEST_LIST.
TEST_F(MissionFlowTest, pull_timeout)
{
  auto pull_client = client->create_client<WaypointPull>("/test_mavros_uas/mission/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointPull::Request>();
  std::promise<WaypointPull::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});

  EXPECT_EQ(
    wait_msgid(*out, MISSION_REQUEST_LIST::MSG_ID).msgid, MISSION_REQUEST_LIST::MSG_ID);

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
}

// Push timeout: FCU requests nothing after the count.
TEST_F(MissionFlowTest, push_timeout)
{
  auto push_client = client->create_client<WaypointPush>("/test_mavros_uas/mission/push");
  ASSERT_TRUE(push_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointPush::Request>();
  req->waypoints.push_back(make_wp(enum_value(MAV_CMD::NAV_WAYPOINT)));

  std::promise<WaypointPush::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(push_client, req, 10s));});

  EXPECT_EQ(wait_msgid(*out, MISSION_COUNT::MSG_ID).msgid, MISSION_COUNT::MSG_ID);

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
}

// Partial push out-of-range is rejected without touching the FCU.
TEST_F(MissionFlowTest, partial_push_out_of_range)
{
  auto push_client = client->create_client<WaypointPush>("/test_mavros_uas/mission/push");
  ASSERT_TRUE(push_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointPush::Request>();
  req->start_index = 10;
  req->waypoints.push_back(make_wp(enum_value(MAV_CMD::NAV_WAYPOINT)));

  auto res = call_service(push_client, req, 5s);

  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->wp_transfered, 0u);
}

// Another client's download running alongside ours: the FCU's replies to it
// (count and items addressed to the other client) must not end up in our list.
TEST_F(MissionFlowTest, pull_ignores_other_clients_replies)
{
  auto pull_client = client->create_client<WaypointPull>("/test_mavros_uas/mission/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointPull::Request>();
  std::promise<WaypointPull::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});

  EXPECT_EQ(
    wait_msgid(*out, MISSION_REQUEST_LIST::MSG_ID).msgid, MISSION_REQUEST_LIST::MSG_ID);

  inject(fcu_count(5, false));
  inject(fcu_count(2));
  inject(fcu_item_int(0, false));
  inject(fcu_item_int(0));
  inject(fcu_item_int(1, false));
  inject(fcu_item_int(1));

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->wp_received, 2u);

  std::mutex mtx;
  mavros_msgs::msg::WaypointList::SharedPtr wpl;
  auto sub = client->create_subscription<mavros_msgs::msg::WaypointList>(
    "/test_mavros_uas/mission/waypoints", mavros::LatchedStateQoS(),
    [&](mavros_msgs::msg::WaypointList::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mtx);
      wpl = msg;
    });
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      if (wpl) {
        break;
      }
    }
    std::this_thread::sleep_for(10ms);
  }

  std::lock_guard<std::mutex> lock(mtx);
  ASSERT_NE(wpl, nullptr);
  ASSERT_EQ(wpl->waypoints.size(), 2u);
  for (const auto & wp : wpl->waypoints) {
    EXPECT_DOUBLE_EQ(wp.y_long, 2.0);
  }
}

// An ACK the FCU sends to another client's upload must not end ours.
TEST_F(MissionFlowTest, push_ignores_other_clients_ack)
{
  auto push_client = client->create_client<WaypointPush>("/test_mavros_uas/mission/push");
  ASSERT_TRUE(push_client->wait_for_service(5s));

  auto req = std::make_shared<WaypointPush::Request>();
  req->start_index = 0;
  req->waypoints.push_back(make_wp(enum_value(MAV_CMD::NAV_WAYPOINT), 1, 2, 3));
  req->waypoints.push_back(make_wp(enum_value(MAV_CMD::NAV_WAYPOINT), 4, 5, 6));

  std::promise<WaypointPush::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(push_client, req, 10s));});

  EXPECT_EQ(wait_msgid(*out, MISSION_COUNT::MSG_ID).msgid, MISSION_COUNT::MSG_ID);

  inject(fcu_request_int(0));
  EXPECT_EQ(wait_msgid(*out, MISSION_ITEM_INT::MSG_ID).msgid, MISSION_ITEM_INT::MSG_ID);

  inject(fcu_ack(MAV_MISSION_RESULT::ACCEPTED, false));

  inject(fcu_request_int(1));
  inject(fcu_ack(MAV_MISSION_RESULT::ACCEPTED));

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->wp_transfered, 2u);
}

// Watching other clients while idle is unchanged: another client's download
// still schedules a pull (pull_after_gcs).
TEST_F(MissionFlowTest, other_clients_download_schedules_pull)
{
  plugin->get_node()->set_parameter(rclcpp::Parameter("mission_reschedule_time", 0.2));

  inject(fcu_count(3, false));

  EXPECT_EQ(
    wait_msgid(*out, MISSION_REQUEST_LIST::MSG_ID, 3s).msgid, MISSION_REQUEST_LIST::MSG_ID);
}

// A MISSION_CURRENT change during a pull is published right away, with the last
// complete list; the pulled list then keeps the new current seq.
TEST_F(MissionFlowTest, current_published_during_pull)
{
  std::mutex mtx;
  mavros_msgs::msg::WaypointList::SharedPtr wpl;
  auto sub = client->create_subscription<mavros_msgs::msg::WaypointList>(
    "/test_mavros_uas/mission/waypoints", mavros::LatchedStateQoS(),
    [&](mavros_msgs::msg::WaypointList::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mtx);
      wpl = msg;
    });
  auto wait_list = [&](uint16_t current_seq, size_t size) -> bool {
      const auto deadline = std::chrono::steady_clock::now() + 5s;
      while (std::chrono::steady_clock::now() < deadline) {
        {
          std::lock_guard<std::mutex> lock(mtx);
          if (wpl && wpl->current_seq == current_seq && wpl->waypoints.size() == size) {
            return true;
          }
        }
        std::this_thread::sleep_for(10ms);
      }
      return false;
    };
  auto count_msgid = [&](uint32_t msgid) {
      size_t n = 0;
      for (const auto & m : *out) {
        n += m.msgid == msgid;
      }
      return n;
    };

  auto pull_client = client->create_client<WaypointPull>("/test_mavros_uas/mission/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));
  auto req = std::make_shared<WaypointPull::Request>();

  // first pull: two items
  {
    std::promise<WaypointPull::Response::SharedPtr> prom;
    std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});
    EXPECT_EQ(
      wait_msgid(*out, MISSION_REQUEST_LIST::MSG_ID).msgid, MISSION_REQUEST_LIST::MSG_ID);
    inject(fcu_count(2));
    inject(fcu_item_int(0));
    inject(fcu_item_int(1));
    auto res = prom.get_future().get();
    svc.join();
    ASSERT_NE(res, nullptr);
    ASSERT_TRUE(res->success);
  }
  ASSERT_TRUE(wait_list(0, 2));

  // second pull: three items, MISSION_CURRENT changes halfway through
  std::promise<WaypointPull::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (count_msgid(MISSION_REQUEST_LIST::MSG_ID) < 2 &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(count_msgid(MISSION_REQUEST_LIST::MSG_ID), 2u);
  inject(fcu_count(3));
  inject(fcu_item_int(0));

  inject(fcu_current(1));
  ASSERT_TRUE(wait_list(1, 2));
  {
    std::lock_guard<std::mutex> lock(mtx);
    EXPECT_FALSE(wpl->waypoints[0].is_current);
    EXPECT_TRUE(wpl->waypoints[1].is_current);
  }

  inject(fcu_item_int(1));
  inject(fcu_item_int(2));
  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_TRUE(wait_list(1, 3));
}

// A pull that fails part way keeps the last complete list: a later
// MISSION_CURRENT change publishes it, not the items the failed pull got.
TEST_F(MissionFlowTest, failed_pull_keeps_last_list)
{
  std::mutex mtx;
  mavros_msgs::msg::WaypointList::SharedPtr wpl;
  auto sub = client->create_subscription<mavros_msgs::msg::WaypointList>(
    "/test_mavros_uas/mission/waypoints", mavros::LatchedStateQoS(),
    [&](mavros_msgs::msg::WaypointList::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mtx);
      wpl = msg;
    });
  auto wait_list = [&](uint16_t current_seq, size_t size) -> bool {
      const auto deadline = std::chrono::steady_clock::now() + 5s;
      while (std::chrono::steady_clock::now() < deadline) {
        {
          std::lock_guard<std::mutex> lock(mtx);
          if (wpl && wpl->current_seq == current_seq && wpl->waypoints.size() == size) {
            return true;
          }
        }
        std::this_thread::sleep_for(10ms);
      }
      return false;
    };
  auto count_msgid = [&](uint32_t msgid) {
      size_t n = 0;
      for (const auto & m : *out) {
        n += m.msgid == msgid;
      }
      return n;
    };

  auto pull_client = client->create_client<WaypointPull>("/test_mavros_uas/mission/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));
  auto req = std::make_shared<WaypointPull::Request>();

  // first pull: three items
  {
    std::promise<WaypointPull::Response::SharedPtr> prom;
    std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});
    EXPECT_EQ(
      wait_msgid(*out, MISSION_REQUEST_LIST::MSG_ID).msgid, MISSION_REQUEST_LIST::MSG_ID);
    inject(fcu_count(3));
    inject(fcu_item_int(0));
    inject(fcu_item_int(1));
    inject(fcu_item_int(2));
    auto res = prom.get_future().get();
    svc.join();
    ASSERT_NE(res, nullptr);
    ASSERT_TRUE(res->success);
  }
  ASSERT_TRUE(wait_list(0, 3));

  // second pull: the FCU stops answering after the first item
  std::promise<WaypointPull::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (count_msgid(MISSION_REQUEST_LIST::MSG_ID) < 2 &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(count_msgid(MISSION_REQUEST_LIST::MSG_ID), 2u);
  inject(fcu_count(3));
  inject(fcu_item_int(0));
  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->wp_received, 1u);

  inject(fcu_current(2));
  ASSERT_TRUE(wait_list(2, 3));
  {
    std::lock_guard<std::mutex> lock(mtx);
    EXPECT_TRUE(wpl->waypoints[2].is_current);
  }
}

}   // namespace uas
}   // namespace mavros

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  mavros::uas::TestUAS::Init();
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
