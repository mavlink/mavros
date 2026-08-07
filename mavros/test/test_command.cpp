//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Service-based tests for the command plugin ROS API.
 *
 * The plugin is compiled into this TU by including its source. Each exposed
 * service is exercised over a real ROS client, the outbound COMMAND_LONG/INT is
 * verified, and a simulated FCU answers with COMMAND_ACK so the response is
 * checked. Error paths (ACK timeout, non-ACK/broadcast) are also covered.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "test_plugin_helpers.hpp"
#include "../src/plugins/command.cpp"  // NOLINT(build/include)

using namespace std::chrono_literals;  // NOLINT

using mavros::utils::enum_value;
using mavlink::common::MAV_CMD;
using mavlink::common::MAV_RESULT;
using mavlink::common::msg::COMMAND_ACK;
using mavlink::common::msg::COMMAND_INT;
using mavlink::common::msg::COMMAND_LONG;
using mavros_msgs::srv::CommandBool;
using mavros_msgs::srv::CommandHome;
using mavros_msgs::srv::CommandInt;
using mavros_msgs::srv::CommandLong;
using mavros_msgs::srv::CommandTOL;
using mavros_msgs::srv::CommandTOLLocal;
using mavros_msgs::srv::CommandTriggerControl;
using mavros_msgs::srv::CommandTriggerInterval;
using mavros_msgs::srv::CommandVtolTransition;

namespace mavros
{
namespace uas
{

class CommandFlowTest : public TestUAS
{
public:
  UAS::SharedPtr uas;
  std::shared_ptr<mavros::std_plugins::CommandPlugin> plugin;
  std::shared_ptr<std::vector<mavros_msgs::msg::Mavlink>> out;
  rclcpp::Node::SharedPtr client;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(1, 1);

    plugin = std::make_shared<mavros::std_plugins::CommandPlugin>(uas.get());
    register_plugin(uas, plugin);

    // Keep ACK timeout short for the timeout test.
    plugin->get_node()->set_parameter(rclcpp::Parameter("command_ack_timeout", 1.0));

    add_node(uas);
    add_node(plugin->get_node());
    client = make_client_node("command_client");
    out = capture_sink(uas);
    start_spin();
  }

  void TearDown() override
  {
    stop_spin();
  }

  void inject(const mavlink::mavlink_message_t & mmsg)
  {
    route(uas, &mmsg, mavconn::Framing::ok);
    std::this_thread::sleep_for(10ms);
  }

  mavlink::mavlink_message_t fcu_command_ack(uint16_t command, uint8_t result, uint8_t sysid = 1)
  {
    COMMAND_ACK ack{};
    ack.command = command;
    ack.result = result;
    auto mmsg = convert_message(ack);
    mmsg.sysid = sysid;
    mmsg.compid = 1;
    mmsg.msgid = COMMAND_ACK::MSG_ID;
    return mmsg;
  }

  template<class M>
  M decode(const mavros_msgs::msg::Mavlink & rmsg)
  {
    auto m = TestUAS::to_mavlink(rmsg);
    M out{};
    mavlink::MsgMap map(m);
    out.deserialize(map);
    return out;
  }
};

#define EXPECT_COMMAND_LONG(msg, expected_command) \
  do { \
    auto cl = wait_msgid(*out, COMMAND_LONG::MSG_ID); \
    ASSERT_EQ(cl.msgid, COMMAND_LONG::MSG_ID); \
    auto d = decode<COMMAND_LONG>(cl); \
    EXPECT_EQ(d.command, expected_command); \
    EXPECT_EQ(d.target_system, 1); \
    EXPECT_EQ(d.target_component, 1); \
    (void)(msg); \
  } while (0)

TEST_F(CommandFlowTest, arming_success)
{
  auto c = client->create_client<CommandBool>("/test_mavros_uas/cmd/arming");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandBool::Request>();
  req->value = true;
  std::promise<CommandBool::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  EXPECT_COMMAND_LONG(req, enum_value(MAV_CMD::COMPONENT_ARM_DISARM));
  inject(fcu_command_ack(enum_value(MAV_CMD::COMPONENT_ARM_DISARM),
        enum_value(MAV_RESULT::ACCEPTED)));

  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
}

TEST_F(CommandFlowTest, set_home_success)
{
  auto c = client->create_client<CommandHome>("/test_mavros_uas/cmd/set_home");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandHome::Request>();
  req->current_gps = false;
  req->yaw = 90.0f;
  req->latitude = 47.0f;
  req->longitude = 11.0f;
  req->altitude = 500.0f;
  std::promise<CommandHome::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  auto cl = wait_msgid(*out, COMMAND_LONG::MSG_ID);
  ASSERT_EQ(cl.msgid, COMMAND_LONG::MSG_ID);
  auto d = decode<COMMAND_LONG>(cl);
  EXPECT_EQ(d.command, enum_value(MAV_CMD::DO_SET_HOME));
  EXPECT_EQ(d.param1, 0.0f);
  EXPECT_EQ(d.param4, 90.0f);
  EXPECT_EQ(d.param5, 47.0f);
  EXPECT_EQ(d.param6, 11.0f);
  EXPECT_EQ(d.param7, 500.0f);

  inject(fcu_command_ack(enum_value(MAV_CMD::DO_SET_HOME), enum_value(MAV_RESULT::ACCEPTED)));
  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
}

TEST_F(CommandFlowTest, takeoff_success)
{
  auto c = client->create_client<CommandTOL>("/test_mavros_uas/cmd/takeoff");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandTOL::Request>();
  req->min_pitch = 10.0f;
  req->yaw = 45.0f;
  req->latitude = 47.0f;
  req->longitude = 11.0f;
  req->altitude = 100.0f;
  std::promise<CommandTOL::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  auto cl = wait_msgid(*out, COMMAND_LONG::MSG_ID);
  ASSERT_EQ(cl.msgid, COMMAND_LONG::MSG_ID);
  auto d = decode<COMMAND_LONG>(cl);
  EXPECT_EQ(d.command, enum_value(MAV_CMD::NAV_TAKEOFF));
  EXPECT_EQ(d.param1, 10.0f);
  EXPECT_EQ(d.param4, 45.0f);
  EXPECT_EQ(d.param5, 47.0f);
  EXPECT_EQ(d.param6, 11.0f);
  EXPECT_EQ(d.param7, 100.0f);

  inject(fcu_command_ack(enum_value(MAV_CMD::NAV_TAKEOFF), enum_value(MAV_RESULT::ACCEPTED)));
  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
}

TEST_F(CommandFlowTest, land_local_success)
{
  auto c = client->create_client<CommandTOLLocal>("/test_mavros_uas/cmd/land_local");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandTOLLocal::Request>();
  req->offset = 5.0f;
  req->rate = 2.0f;
  req->yaw = 30.0f;
  req->position.x = 1.0f;
  req->position.y = 2.0f;
  req->position.z = 3.0f;
  std::promise<CommandTOLLocal::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  auto cl = wait_msgid(*out, COMMAND_LONG::MSG_ID);
  ASSERT_EQ(cl.msgid, COMMAND_LONG::MSG_ID);
  auto d = decode<COMMAND_LONG>(cl);
  EXPECT_EQ(d.command, enum_value(MAV_CMD::NAV_LAND_LOCAL));
  EXPECT_EQ(d.param2, 5.0f);
  EXPECT_EQ(d.param3, 2.0f);
  EXPECT_EQ(d.param4, 30.0f);
  EXPECT_EQ(d.param5, 2.0f);   // position.y
  EXPECT_EQ(d.param6, 1.0f);   // position.x
  EXPECT_EQ(d.param7, 3.0f);   // position.z

  inject(fcu_command_ack(enum_value(MAV_CMD::NAV_LAND_LOCAL), enum_value(MAV_RESULT::ACCEPTED)));
  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
}

TEST_F(CommandFlowTest, trigger_control_success)
{
  auto c = client->create_client<CommandTriggerControl>("/test_mavros_uas/cmd/trigger_control");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandTriggerControl::Request>();
  req->trigger_enable = true;
  req->sequence_reset = true;
  req->trigger_pause = false;
  std::promise<CommandTriggerControl::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  EXPECT_COMMAND_LONG(req, enum_value(MAV_CMD::DO_TRIGGER_CONTROL));
  inject(fcu_command_ack(enum_value(MAV_CMD::DO_TRIGGER_CONTROL),
        enum_value(MAV_RESULT::ACCEPTED)));
  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
}

TEST_F(CommandFlowTest, trigger_interval_success)
{
  auto c = client->create_client<CommandTriggerInterval>("/test_mavros_uas/cmd/trigger_interval");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandTriggerInterval::Request>();
  req->cycle_time = 1.5f;
  req->integration_time = 0.5f;
  std::promise<CommandTriggerInterval::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  auto cl = wait_msgid(*out, COMMAND_LONG::MSG_ID);
  ASSERT_EQ(cl.msgid, COMMAND_LONG::MSG_ID);
  auto d = decode<COMMAND_LONG>(cl);
  EXPECT_EQ(d.command, enum_value(MAV_CMD::DO_SET_CAM_TRIGG_INTERVAL));
  EXPECT_EQ(d.param1, 1.5f);
  EXPECT_EQ(d.param2, 0.5f);

  inject(fcu_command_ack(enum_value(MAV_CMD::DO_SET_CAM_TRIGG_INTERVAL),
        enum_value(MAV_RESULT::ACCEPTED)));
  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
}

// vtol_transition uses confirmation=false, so it does not wait for an ACK.
TEST_F(CommandFlowTest, vtol_transition_no_ack)
{
  auto c = client->create_client<CommandVtolTransition>("/test_mavros_uas/cmd/vtol_transition");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandVtolTransition::Request>();
  req->state = 1;
  auto res = call_service(c, req, 5s);

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(wait_msgid(*out, COMMAND_LONG::MSG_ID).msgid, COMMAND_LONG::MSG_ID);
}

// Generic CommandLong with arbitrary command/params.
TEST_F(CommandFlowTest, command_long_success)
{
  auto c = client->create_client<CommandLong>("/test_mavros_uas/cmd/command");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandLong::Request>();
  req->command = enum_value(MAV_CMD::COMPONENT_ARM_DISARM);
  req->confirmation = 1;
  req->param1 = 1.0f;
  req->param2 = 2.0f;
  req->param3 = 3.0f;
  req->param4 = 4.0f;
  req->param5 = 5.0f;
  req->param6 = 6.0f;
  req->param7 = 7.0f;
  std::promise<CommandLong::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  auto cl = wait_msgid(*out, COMMAND_LONG::MSG_ID);
  ASSERT_EQ(cl.msgid, COMMAND_LONG::MSG_ID);
  auto d = decode<COMMAND_LONG>(cl);
  EXPECT_EQ(d.command, req->command);
  EXPECT_EQ(d.param1, 1.0f);
  EXPECT_EQ(d.param7, 7.0f);

  inject(fcu_command_ack(req->command, enum_value(MAV_RESULT::ACCEPTED)));
  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->result, enum_value(MAV_RESULT::ACCEPTED));
}

// Generic CommandInt produces COMMAND_INT (not COMMAND_LONG).
TEST_F(CommandFlowTest, command_int_success)
{
  auto c = client->create_client<CommandInt>("/test_mavros_uas/cmd/command_int");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandInt::Request>();
  req->command = enum_value(MAV_CMD::COMPONENT_ARM_DISARM);
  req->frame = enum_value(mavlink::common::MAV_FRAME::BODY_NED);
  req->autocontinue = 1;
  req->x = 100;
  req->y = 200;
  req->z = 3.0f;
  auto res = call_service(c, req, 5s);

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  auto ci = wait_msgid(*out, COMMAND_INT::MSG_ID);
  ASSERT_EQ(ci.msgid, COMMAND_INT::MSG_ID);
  auto d = decode<COMMAND_INT>(ci);
  EXPECT_EQ(d.command, req->command);
  EXPECT_EQ(d.x, 100);
  EXPECT_EQ(d.y, 200);
  EXPECT_EQ(d.z, 3.0f);
}

// ACK timeout: no COMMAND_ACK sent, report failure.
TEST_F(CommandFlowTest, ack_timeout)
{
  plugin->get_node()->set_parameter(rclcpp::Parameter("command_ack_timeout", 0.2));

  auto c = client->create_client<CommandBool>("/test_mavros_uas/cmd/arming");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandBool::Request>();
  req->value = true;
  std::promise<CommandBool::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  EXPECT_COMMAND_LONG(req, enum_value(MAV_CMD::COMPONENT_ARM_DISARM));

  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
}

// Broadcast command skips the ACK wait entirely.
TEST_F(CommandFlowTest, broadcast_no_ack)
{
  auto c = client->create_client<CommandLong>("/test_mavros_uas/cmd/command");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandLong::Request>();
  req->command = enum_value(MAV_CMD::COMPONENT_ARM_DISARM);
  req->broadcast = true;
  req->confirmation = 1;
  auto res = call_service(c, req, 5s);

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  auto cl = wait_msgid(*out, COMMAND_LONG::MSG_ID);
  ASSERT_EQ(cl.msgid, COMMAND_LONG::MSG_ID);
  auto d = decode<COMMAND_LONG>(cl);
  EXPECT_EQ(d.target_system, 0);   // broadcast target
  EXPECT_EQ(d.target_component, 0);
}

// Failed ACK result is surfaced in the response.
TEST_F(CommandFlowTest, ack_failed_result)
{
  auto c = client->create_client<CommandBool>("/test_mavros_uas/cmd/arming");
  ASSERT_TRUE(c->wait_for_service(5s));

  auto req = std::make_shared<CommandBool::Request>();
  req->value = true;
  std::promise<CommandBool::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(c, req, 10s));});

  EXPECT_COMMAND_LONG(req, enum_value(MAV_CMD::COMPONENT_ARM_DISARM));
  inject(fcu_command_ack(enum_value(MAV_CMD::COMPONENT_ARM_DISARM),
        enum_value(MAV_RESULT::FAILED)));

  auto res = prom.get_future().get();
  svc.join();
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->result, enum_value(MAV_RESULT::FAILED));
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
