//
// mavros
// Copyright 2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Test mavros router node
 */


#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <mavconn/interface.hpp>
#include <mavros/mavros_router.hpp>

using ::testing::_;

using namespace mavros::router; // NOLINT

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

using LT = Endpoint::Type;

class MockEndpoint : public Endpoint
{
public:
  using SharedPtr = std::shared_ptr<MockEndpoint>;

  MOCK_METHOD0(is_open, bool());
  MOCK_METHOD0(open, std::pair<bool, std::string>());
  MOCK_METHOD0(close, void());

  MOCK_METHOD2(
    send_message, void(const mavlink_message_t * msg,
    const Framing framing));
  MOCK_METHOD2(
    recv_message, void(const mavlink_message_t * msg,
    const Framing framing));
};

namespace mavros
{
namespace router
{

class TestRouter : public ::testing::Test
{
public:
  Router::SharedPtr create_node()
  {
    auto router = std::make_shared<Router>("test_mavros_router");

    auto make_and_add_mock_endpoint =
      [router](id_t id, const std::string & url, LT type,
        std::set<addr_t> remotes) {
        auto ep = std::make_shared<MockEndpoint>();

        ep->parent = router;
        ep->id = id;
        ep->link_type = type;
        ep->url = url;
        ep->remote_addrs = remotes;

        router->endpoints[id] = ep;
      };

    router->id_counter = 1000;
    make_and_add_mock_endpoint(1000, "mock://fcu1", LT::fcu, {0x0000, 0x0100, 0x0101});
    make_and_add_mock_endpoint(1001, "mock://fcu2", LT::fcu, {0x0000, 0x0200, 0x0201});
    make_and_add_mock_endpoint(1002, "/uas1", LT::uas, {0x0000, 0x0100, 0x01BF});
    make_and_add_mock_endpoint(1003, "/uas2", LT::uas, {0x0000, 0x0200, 0x02BF});
    make_and_add_mock_endpoint(1004, "mock://gcs1", LT::gcs, {0x0000, 0xFF00, 0xFFBE});

    return router;
  }

  MockEndpoint::SharedPtr getep(Router::SharedPtr router, id_t id)
  {
    auto ep = router->endpoints[id];
    return std::static_pointer_cast<MockEndpoint>(ep);
  }

  std::unordered_map<id_t, Endpoint::SharedPtr> & get_endpoints(Router::SharedPtr router)
  {
    return router->endpoints;
  }

  mavlink_message_t convert_message(const mavlink::Message & msg, const addr_t source = 0x0101)
  {
    mavlink_message_t ret;
    mavlink::MsgMap map(ret);
    msg.serialize(map);
    ret.sysid = source >> 8;
    ret.compid = source & 0xFF;
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

TEST_F(TestRouter, set_parameter) {
  auto router = std::make_shared<Router>("test_mavros_router");

  router->set_parameters(
    std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("fcu_urls", std::vector<std::string>{"udp://:45001@", "tcp-l://:57601"}),
        rclcpp::Parameter("gcs_urls", std::vector<std::string>{"udp://:45002@", "tcp-l://:57602"}),
        rclcpp::Parameter("uas_urls", std::vector<std::string>{"/uas1", "/uas2"})
      });

  auto expected_values = std::vector<std::pair<std::string, LT>>{
    {"tcp-l://:57601", LT::fcu},    // NOTE(vooon): std::set change order of tcp and udp
    {"udp://:45001@", LT::fcu},
    {"tcp-l://:57602", LT::gcs},
    {"udp://:45002@", LT::gcs},
    {"/uas1", LT::uas},
    {"/uas2", LT::uas},
  };

  auto endpoints = get_endpoints(router);

  EXPECT_EQ(size_t(6), endpoints.size());
  for (auto & kv : endpoints) {
    auto & ep = kv.second;
    auto tc = expected_values.at(ep->id - 1000);

    EXPECT_EQ(tc.first, ep->url);
    EXPECT_EQ(tc.second, ep->link_type);
  }
}

TEST_F(TestRouter, route_fcu_broadcast) {
  auto router = this->create_node();

  auto hb = make_heartbeat();
  auto hbmsg = convert_message(hb, 0x0101);
  auto fr = Framing::ok;

  auto fcu1 = getep(router, 1000);
  auto fcu2 = getep(router, 1001);
  auto uas1 = getep(router, 1002);
  auto uas2 = getep(router, 1003);
  auto gcs1 = getep(router, 1004);

  // EXPECT_CALL(*fcu1, send_message(_, fr));
  // EXPECT_CALL(*fcu2, send_message(_, fr));
  // EXPECT_CALL(*uas1, send_message(_, fr));
  // EXPECT_CALL(*uas2, send_message(_, fr));
  // EXPECT_CALL(*gcs1, send_message(_, fr));

  router->route_message(fcu1, &hbmsg, fr);

  testing::Mock::VerifyAndClearExpectations(fcu1);
  testing::Mock::VerifyAndClearExpectations(fcu2);
  testing::Mock::VerifyAndClearExpectations(uas1);
  testing::Mock::VerifyAndClearExpectations(uas2);
  testing::Mock::VerifyAndClearExpectations(gcs1);
}

//TEST_F(TestRouter, route_uas_broadcast) {
//  this->create_node();
//
//  auto hb = make_heartbeat();
//  auto hbmsg = convert_message(hb, 0x01BF);
//  auto fr = Framing::ok;
//
//  EXPECT_CALL(*fcu1, send_message(&hbmsg, fr)).Times(1);
//  EXPECT_CALL(*fcu2, send_message(&hbmsg, fr)).Times(1);
//  // EXPECT_CALL(*uas1, send_message(&hbmsg, fr)).Times(0);
//  // EXPECT_CALL(*uas2, send_message(&hbmsg, fr)).Times(0);
//  EXPECT_CALL(*gcs1, send_message(&hbmsg, fr)).Times(1);
//
//  router->route_message(uas1, &hbmsg, fr);
//}
//
//TEST_F(TestRouter, route_gcs_broadcast) {
//  this->create_node();
//
//  auto hb = make_heartbeat();
//  auto hbmsg = convert_message(hb, 0xFFBE);
//  auto fr = Framing::ok;
//
//  EXPECT_CALL(*fcu1, send_message(&hbmsg, fr)).Times(1);
//  EXPECT_CALL(*fcu2, send_message(&hbmsg, fr)).Times(1);
//  EXPECT_CALL(*uas1, send_message(&hbmsg, fr)).Times(1);
//  EXPECT_CALL(*uas2, send_message(&hbmsg, fr)).Times(1);
//  // EXPECT_CALL(*gcs1, send_message(&hbmsg, fr)).Times(0);
//
//  router->route_message(gcs1, &hbmsg, fr);
//}

} // namespace router
} // namespace mavros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
