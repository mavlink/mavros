// Copyright 2026 Sean Hickey.
// This file is part of MAVROS and subject to the top-level LICENSE.md.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../src/plugins/px4_events.cpp"
#include "px4_events_test_helpers.hpp"

namespace mavros
{
namespace uas
{

// Use the existing UAS test friend to exercise real MAVLink routing and filters.
class TestUAS : public ::testing::Test
{
protected:
  UAS::SharedPtr create_uas(const std::string & name)
  {
    auto vehicle = std::make_shared<UAS>(name);
    vehicle->startup_delay_timer->cancel();
    return vehicle;
  }

  void register_plugin(
    const UAS::SharedPtr & vehicle, const std::shared_ptr<plugin::Plugin> & instance)
  {
    for (const auto & subscription : instance->get_subscriptions()) {
      vehicle->plugin_subscriptions[std::get<0>(subscription)].push_back(subscription);
    }
    vehicle->loaded_plugins.push_back(instance);
  }

  void route(
    const UAS::SharedPtr & vehicle, const mavlink::Message & message,
    uint8_t system, uint8_t component)
  {
    mavlink::mavlink_message_t raw{};
    mavlink::MsgMap map(raw);
    message.serialize(map);
    raw.sysid = system;
    raw.compid = component;
    vehicle->plugin_route(&raw, mavconn::Framing::ok);
  }
};

}  // namespace uas
namespace px4_events
{

class PX4EventsPluginTestPeer
{
public:
  static std::shared_ptr<Discovery> discovery(const PX4EventsPlugin & plugin)
  {
    return plugin.event_discovery;
  }

  static bool decodes(const PX4EventsPlugin & plugin)
  {
    return plugin.event_decoder && plugin.event_decoder->decode(0x01000007, 4, {}).has_value();
  }
};

class DiscoveryTestPeer
{
public:
  static bool ready(const Discovery & discovery)
  {
    return discovery.client_->service_is_ready();
  }

  static bool active(const Discovery & discovery)
  {
    std::lock_guard<std::mutex> lock(discovery.mutex_);
    return discovery.active_;
  }
};

namespace
{
using namespace std::chrono_literals;  // NOLINT
using Command = mavros_msgs::srv::CommandLong;
using Peer = PX4EventsPluginTestPeer;

class PluginDiscoveryTest : public uas::TestUAS
{
protected:
  uas::UAS::SharedPtr vehicle;
  std::shared_ptr<PX4EventsPlugin> instance;
  rclcpp::Node::SharedPtr server;
  rclcpp::Service<Command>::SharedPtr command_service;
  std::vector<Command::Request> requests;
  // Destroy the executor before the nodes and their callback groups (including on Humble).
  rclcpp::executors::SingleThreadedExecutor executor;

  virtual std::string name() const {return "test_px4_events_auto";}

  void SetUp() override
  {
    server = std::make_shared<rclcpp::Node>(name() + "_server");
    command_service = server->create_service<Command>(
      "/" + name() + "/cmd/command",
      [this](Command::Request::SharedPtr request, Command::Response::SharedPtr response) {
        requests.push_back(*request);
        response->success = true;
        response->result = static_cast<uint8_t>(mavlink::common::MAV_RESULT::ACCEPTED);
      });
    vehicle = create_uas(name());
    vehicle->set_tgt(1, 1);
    heartbeat(mavlink::minimal::MAV_AUTOPILOT::PX4);
    instance = std::make_shared<PX4EventsPlugin>(vehicle.get());
    register_plugin(vehicle, instance);
    executor.add_node(server);
    executor.add_node(instance->get_node());
  }

  void TearDown() override
  {
    vehicle->update_connection_status(false);
    executor.cancel();
  }

  void heartbeat(mavlink::minimal::MAV_AUTOPILOT autopilot)
  {
    vehicle->update_heartbeat(
      static_cast<uint8_t>(mavlink::minimal::MAV_TYPE::QUADROTOR),
      static_cast<uint8_t>(autopilot), 0);
  }

  template<typename Predicate>
  bool spin_until(Predicate predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  void advertise(bool legacy = false, uint8_t system = 1, uint8_t component = 1)
  {
    const std::string uri = "mftp://general.json";
    if (legacy) {
      mavlink::common::msg::COMPONENT_INFORMATION message{};
      std::copy(uri.begin(), uri.end(), message.general_metadata_uri.begin());
      message.general_metadata_file_crc = 123;
      route(vehicle, message, system, component);
    } else {
      mavlink::common::msg::COMPONENT_METADATA message{};
      std::copy(uri.begin(), uri.end(), message.uri.begin());
      message.file_crc = 123;
      route(vehicle, message, system, component);
    }
  }
};

TEST_F(PluginDiscoveryTest, default_auto_requests_modern_metadata_after_px4_connection)
{
  auto discovery = Peer::discovery(*instance);
  ASSERT_NE(discovery, nullptr);
  EXPECT_EQ(instance->get_node()->get_parameter("event_decoding_mode").as_string(), "auto");
  EXPECT_FALSE(DiscoveryTestPeer::active(*discovery));
  ASSERT_TRUE(spin_until([&]() {return DiscoveryTestPeer::ready(*discovery);}));
  vehicle->update_connection_status(true);
  ASSERT_TRUE(spin_until([this]() {return !requests.empty();}));
  EXPECT_EQ(requests.front().command,
    static_cast<uint16_t>(mavlink::common::MAV_CMD::REQUEST_MESSAGE));
  EXPECT_EQ(requests.front().param1, 397.0F);
  EXPECT_EQ(requests.front().param7, 1.0F);
  EXPECT_FALSE(requests.front().broadcast);
  EXPECT_FALSE(Peer::decodes(*instance));
}

TEST_F(PluginDiscoveryTest, ignores_disconnected_and_wrong_target_advertisements)
{
  auto discovery = Peer::discovery(*instance);
  ASSERT_NE(discovery, nullptr);
  advertise();
  EXPECT_FALSE(discovery->metadata());
  vehicle->update_connection_status(true);
  advertise(false, 2, 1);
  advertise(false, 1, 2);
  EXPECT_FALSE(discovery->metadata());
  advertise();
  ASSERT_TRUE(discovery->metadata());
  EXPECT_EQ(discovery->metadata()->uri, "mftp://general.json");
  EXPECT_EQ(discovery->metadata()->crc, 123U);
  EXPECT_FALSE(DiscoveryTestPeer::active(*discovery));
  // Stage 1 discovers locations only; definitions remain inactive.
  EXPECT_FALSE(Peer::decodes(*instance));
}

TEST_F(PluginDiscoveryTest, accepts_legacy_advertisement_and_resets_on_reconnect)
{
  auto discovery = Peer::discovery(*instance);
  ASSERT_NE(discovery, nullptr);
  vehicle->update_connection_status(true);
  advertise(true);
  ASSERT_TRUE(discovery->metadata());
  vehicle->update_connection_status(false);
  EXPECT_FALSE(discovery->metadata());
  EXPECT_FALSE(DiscoveryTestPeer::active(*discovery));
  advertise();
  EXPECT_FALSE(discovery->metadata());
  vehicle->update_connection_status(true);
  EXPECT_TRUE(DiscoveryTestPeer::active(*discovery));
  advertise();
  EXPECT_TRUE(discovery->metadata());
}

TEST_F(PluginDiscoveryTest, non_px4_connection_does_not_start_discovery)
{
  auto discovery = Peer::discovery(*instance);
  ASSERT_NE(discovery, nullptr);
  heartbeat(mavlink::minimal::MAV_AUTOPILOT::ARDUPILOTMEGA);
  vehicle->update_connection_status(true);
  EXPECT_FALSE(DiscoveryTestPeer::active(*discovery));
  advertise();
  EXPECT_FALSE(discovery->metadata());
  executor.spin_some();
  EXPECT_TRUE(requests.empty());
}

class DisabledPluginDiscoveryTest : public PluginDiscoveryTest
{
  std::string name() const override {return "test_px4_events_disabled";}
};

TEST_F(DisabledPluginDiscoveryTest, disabled_mode_creates_no_discovery)
{
  EXPECT_EQ(Peer::discovery(*instance), nullptr);
  vehicle->update_connection_status(true);
  advertise();
  executor.spin_some();
  EXPECT_TRUE(requests.empty());
  EXPECT_FALSE(Peer::decodes(*instance));
}

class LocalPluginDiscoveryTest : public PluginDiscoveryTest
{
  std::string name() const override {return "test_px4_events_local";}
};

TEST_F(LocalPluginDiscoveryTest, local_definitions_survive_connection_changes_without_discovery)
{
  EXPECT_EQ(Peer::discovery(*instance), nullptr);
  EXPECT_TRUE(Peer::decodes(*instance));
  vehicle->update_connection_status(true);
  advertise();
  vehicle->update_connection_status(false);
  EXPECT_TRUE(Peer::decodes(*instance));
  executor.spin_some();
  EXPECT_TRUE(requests.empty());
}

class InvalidPluginDiscoveryTest : public PluginDiscoveryTest
{
  std::string name() const override {return "test_px4_events_invalid";}
};

TEST_F(InvalidPluginDiscoveryTest, invalid_configuration_does_not_start_discovery)
{
  EXPECT_EQ(Peer::discovery(*instance), nullptr);
  vehicle->update_connection_status(true);
  executor.spin_some();
  EXPECT_TRUE(requests.empty());
  EXPECT_FALSE(Peer::decodes(*instance));
}

}  // namespace
}  // namespace px4_events
}  // namespace mavros

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  const std::string parameters =
    "/test_px4_events_disabled/px4_events:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: disabled\n"
    "/test_px4_events_local/px4_events:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: local\n"
    "    event_metadata_file: " TEST_EVENT_METADATA "\n"
    "/test_px4_events_invalid/px4_events:\n"
    "  ros__parameters:\n"
    "    event_metadata_file: /unused-in-auto-mode\n";
  mavros::px4_events::TemporaryMetadata file(parameters, ".yaml");
  const char * args[] = {argv[0], "--ros-args", "--params-file", file.path.c_str()};
  rclcpp::init(4, args);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
