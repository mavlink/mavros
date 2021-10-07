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

#include <memory>
#include <string>

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
  : Plugin(uas_) {}

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

}  // namespace uas
}  // namespace mavros

int main(int argc, char ** argv)
{
  // rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
