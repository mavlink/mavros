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

#include <rclcpp/executors.hpp>
#include <mavconn/interface.hpp>
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin_filter.hpp>

using ::testing::_;

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

  MockUAS(const std::string name_)
  : UAS(name_) {}

  MOCK_METHOD1(create_plugin_instance, plugin::Plugin::SharedPtr(const std::string & pl_name));
};

class MockPlugin : public plugin::Plugin
{
public:
  using SharedPtr = std::shared_ptr<MockPlugin>;

  MOCK_METHOD0(get_subscriptions, plugin::Plugin::Subscriptions(void));

  inline plugin::Plugin::SharedPtr getptr()
  {
    return std::static_pointer_cast<plugin::Plugin>(shared_from_this());
  }

#if 0
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
#endif
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

#define VERIFY_EPS() \
  testing::Mock::VerifyAndClearExpectations(&(*fcu1)); \
  testing::Mock::VerifyAndClearExpectations(&(*fcu2)); \
  testing::Mock::VerifyAndClearExpectations(&(*uas1)); \
  testing::Mock::VerifyAndClearExpectations(&(*uas2)); \
  testing::Mock::VerifyAndClearExpectations(&(*gcs1)); \
  testing::Mock::VerifyAndClearExpectations(&(*gcs2));


}  // namespace uas
}  // namespace mavros

int main(int argc, char ** argv)
{
  // rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
