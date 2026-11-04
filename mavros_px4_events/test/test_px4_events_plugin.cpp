// Copyright 2026 Sean Hickey.
// This file is part of MAVROS and subject to the top-level LICENSE.md.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
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

  static bool decodes(PX4EventsPlugin & plugin)
  {
    std::lock_guard<std::recursive_mutex> lock(plugin.event_mutex);
    return plugin.event_decoder && plugin.event_decoder->decode(0x01000007, 4, {}).has_value();
  }

  static void release_downloads(PX4EventsPlugin & plugin)
  {
    std::lock_guard<std::recursive_mutex> lock(plugin.event_mutex);
    plugin.event_download_delay_cb(plugin.event_generation,
      plugin.event_connected_at + std::chrono::seconds(31));
  }

  static bool timer_cancelled(const PX4EventsPlugin & plugin)
  {
    return plugin.event_start_timer && plugin.event_start_timer->is_canceled();
  }

  static uint64_t generation(const PX4EventsPlugin & plugin)
  {
    return plugin.event_generation;
  }

  static void complete(
    PX4EventsPlugin & plugin, uint64_t generation, Acquisition::Result result)
  {
    plugin.event_acquisition_complete(generation, std::move(result));
  }

  static std::shared_ptr<Acquisition> acquisition(const PX4EventsPlugin & plugin)
  {
    return plugin.event_acquisition;
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

class AcquisitionTestPeer
{
public:
  static bool waiting(Acquisition & acquisition)
  {
    std::lock_guard<std::mutex> lock(acquisition.mutex_);
    return acquisition.phase_ == Acquisition::Phase::DownloadWait;
  }

  static bool idle(Acquisition & acquisition)
  {
    std::lock_guard<std::mutex> lock(acquisition.mutex_);
    return acquisition.phase_ == Acquisition::Phase::Idle && acquisition.timer_->is_canceled();
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
  // Unusable definitions cannot activate a decoder.
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

std::string integration_cache_directory;
using Open = mavros_msgs::srv::FileOpen;
using Read = mavros_msgs::srv::FileRead;
using Close = mavros_msgs::srv::FileClose;

class PluginAcquisitionTest : public PluginDiscoveryTest
{
protected:
  std::string definitions;
  std::string general;
  std::vector<std::string> opens;
  int open_error{0};
  bool hold_reads{false};
  std::shared_ptr<rmw_request_id_t> held_read;
  rclcpp::Service<Open>::SharedPtr open_service;
  rclcpp::Service<Read>::SharedPtr read_service;
  rclcpp::Service<Close>::SharedPtr close_service;

  std::string name() const override {return "test_px4_events_acquisition";}

  void SetUp() override
  {
    std::filesystem::remove_all(integration_cache_directory);
    std::ifstream file(TEST_EVENT_METADATA);
    definitions.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    general = "{\"version\":1,\"metadataTypes\":[{\"type\":4,"
      "\"uri\":\"mftp://events.json\",\"fileCrc\":" +
      std::to_string(metadata_crc32(definitions)) + "}]}";
    PluginDiscoveryTest::SetUp();
    ASSERT_FALSE(definitions.empty());
    open_service = server->create_service<Open>(
      "/" + name() + "/ftp/open",
      [this](Open::Request::SharedPtr request, Open::Response::SharedPtr response) {
        opens.push_back(request->file_path);
        response->success = open_error == 0;
        response->r_errno = open_error;
        response->size = request->file_path == "general.json" ? general.size() : definitions.size();
      });
    read_service = server->create_service<Read>(
      "/" + name() + "/ftp/read",
      [this](std::shared_ptr<rmw_request_id_t> header, Read::Request::SharedPtr request) {
        if (hold_reads) {
          held_read = header;
          return;
        }
        const auto & bytes = request->file_path == "general.json" ? general : definitions;
        const auto chunk = bytes.substr(request->offset, request->size);
        Read::Response response;
        response.success = true;
        response.data.assign(chunk.begin(), chunk.end());
        read_service->send_response(*header, response);
      });
    close_service = server->create_service<Close>(
      "/" + name() + "/ftp/close",
      [](Close::Request::SharedPtr, Close::Response::SharedPtr response) {
        response->success = true;
      });
  }

  void TearDown() override
  {
    PluginDiscoveryTest::TearDown();
    std::filesystem::remove_all(integration_cache_directory);
  }

  MetadataCache cache() const {return MetadataCache(integration_cache_directory);}

  MetadataFile general_file() const
  {
    return metadata_file("mftp://general.json", metadata_crc32(general));
  }

  MetadataFile events_file() const
  {
    return metadata_file("mftp://events.json", metadata_crc32(definitions));
  }

  void advertise_valid()
  {
    mavlink::common::msg::COMPONENT_METADATA message{};
    const auto advertised = general_file();
    std::copy(advertised.uri.begin(), advertised.uri.end(), message.uri.begin());
    message.file_crc = advertised.crc;
    route(vehicle, message, 1, 1);
  }

  void connect_and_advertise()
  {
    vehicle->update_connection_status(true);
    advertise_valid();
  }

  bool waiting_for_delay()
  {
    return AcquisitionTestPeer::waiting(*Peer::acquisition(*instance));
  }
};

TEST_F(PluginAcquisitionTest, cold_download_waits_then_activates_and_saves_both_files_once)
{
  connect_and_advertise();
  ASSERT_TRUE(spin_until([this]() {return waiting_for_delay();}));
  EXPECT_TRUE(opens.empty());
  EXPECT_FALSE(Peer::decodes(*instance));
  advertise_valid();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens, (std::vector<std::string>{"general.json", "events.json"}));
  EXPECT_TRUE(Peer::timer_cancelled(*instance));
  EXPECT_EQ(cache().load(general_file(), MetadataRole::General), general);
  EXPECT_EQ(cache().load(events_file(), MetadataRole::Events), definitions);
  advertise_valid();
  executor.spin_some();
  EXPECT_EQ(opens.size(), 2U);
}

TEST_F(PluginAcquisitionTest, complete_cache_hit_bypasses_delay_and_needs_no_ftp_services)
{
  ASSERT_TRUE(cache().save(general_file(), MetadataRole::General, general).empty());
  ASSERT_TRUE(cache().save(events_file(), MetadataRole::Events, definitions).empty());
  open_service.reset();
  read_service.reset();
  close_service.reset();
  connect_and_advertise();
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_TRUE(opens.empty());
  EXPECT_TRUE(Peer::timer_cancelled(*instance));
}

TEST_F(PluginAcquisitionTest, cached_general_downloads_only_events_after_delay)
{
  ASSERT_TRUE(cache().save(general_file(), MetadataRole::General, general).empty());
  connect_and_advertise();
  ASSERT_TRUE(spin_until([this]() {return waiting_for_delay();}));
  EXPECT_TRUE(opens.empty());
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens, (std::vector<std::string>{"events.json"}));
}

TEST_F(PluginAcquisitionTest, cached_events_downloads_only_general)
{
  ASSERT_TRUE(cache().save(events_file(), MetadataRole::Events, definitions).empty());
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens, (std::vector<std::string>{"general.json"}));
}

TEST_F(PluginAcquisitionTest, corrupt_cache_entries_fall_back_to_download)
{
  ASSERT_TRUE(cache().save(general_file(), MetadataRole::General, general).empty());
  ASSERT_TRUE(cache().save(events_file(), MetadataRole::Events, definitions).empty());
  for (const auto & entry : std::filesystem::directory_iterator(integration_cache_directory)) {
    std::ofstream(entry.path(), std::ios::trunc) << "corrupt";
  }
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens, (std::vector<std::string>{"general.json", "events.json"}));
}

TEST_F(PluginAcquisitionTest, unusable_cache_directory_does_not_prevent_activation)
{
  std::ofstream(integration_cache_directory) << "not a directory";
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens.size(), 2U);
  EXPECT_TRUE(std::filesystem::is_regular_file(integration_cache_directory));
}

TEST_F(PluginAcquisitionTest, download_failure_leaves_decoder_inactive_and_cancels_delay_timer)
{
  open_error = ENOENT;
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {
      return !opens.empty() && AcquisitionTestPeer::idle(*Peer::acquisition(*instance));
  }));
  EXPECT_FALSE(Peer::decodes(*instance));
  EXPECT_TRUE(Peer::timer_cancelled(*instance));
  EXPECT_FALSE(cache().load(general_file(), MetadataRole::General));
}

TEST_F(PluginAcquisitionTest, invalid_downloaded_metadata_does_not_activate_decoder)
{
  general = "invalid JSON";
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {
      return !opens.empty() && AcquisitionTestPeer::idle(*Peer::acquisition(*instance));
  }));
  EXPECT_FALSE(Peer::decodes(*instance));
  EXPECT_TRUE(Peer::timer_cancelled(*instance));
  EXPECT_EQ(opens, (std::vector<std::string>{"general.json"}));
}

TEST_F(PluginAcquisitionTest, advertisement_after_delay_starts_without_a_second_delay)
{
  vehicle->update_connection_status(true);
  Peer::release_downloads(*instance);
  EXPECT_TRUE(Peer::timer_cancelled(*instance));
  advertise_valid();
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens.size(), 2U);
}

TEST_F(PluginAcquisitionTest, disconnect_discards_late_reply_and_reconnect_can_acquire)
{
  hold_reads = true;
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return held_read != nullptr;}));
  vehicle->update_connection_status(false);
  EXPECT_TRUE(Peer::timer_cancelled(*instance));
  EXPECT_FALSE(Peer::decodes(*instance));
  Read::Response late;
  late.success = true;
  late.data.assign(general.begin(), general.end());
  read_service->send_response(*held_read, late);
  executor.spin_some();
  EXPECT_FALSE(Peer::decodes(*instance));
  hold_reads = false;
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  vehicle->update_connection_status(false);
  EXPECT_FALSE(Peer::decodes(*instance));
}

TEST_F(PluginAcquisitionTest, old_connection_completion_cannot_activate_new_connection)
{
  connect_and_advertise();
  const auto old_generation = Peer::generation(*instance);
  vehicle->update_connection_status(false);
  vehicle->update_connection_status(true);
  auto decoder = std::make_unique<Decoder>();
  ASSERT_TRUE(decoder->load_json(definitions).empty());
  Peer::complete(*instance, old_generation, {std::move(decoder), "", false});
  EXPECT_FALSE(Peer::decodes(*instance));
  EXPECT_FALSE(Peer::timer_cancelled(*instance));
}

class ZeroDelayPluginTest : public PluginAcquisitionTest
{
  std::string name() const override {return "test_px4_events_zero_delay";}
};

TEST_F(ZeroDelayPluginTest, zero_delay_downloads_without_explicit_gate_release)
{
  connect_and_advertise();
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens.size(), 2U);
  EXPECT_TRUE(Peer::timer_cancelled(*instance));
}

class DisabledCachePluginTest : public PluginAcquisitionTest
{
  std::string name() const override {return "test_px4_events_no_cache";}
};

TEST_F(DisabledCachePluginTest, disabled_cache_downloads_without_reading_or_writing_entries)
{
  ASSERT_TRUE(cache().save(general_file(), MetadataRole::General, general).empty());
  ASSERT_TRUE(cache().save(events_file(), MetadataRole::Events, definitions).empty());
  const auto stamp = std::filesystem::file_time_type::clock::now() - 24h;
  for (const auto & entry : std::filesystem::directory_iterator(integration_cache_directory)) {
    std::filesystem::last_write_time(entry.path(), stamp);
  }
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens.size(), 2U);
  for (const auto & entry : std::filesystem::directory_iterator(integration_cache_directory)) {
    EXPECT_EQ(std::filesystem::last_write_time(entry.path()), stamp);
  }
}

class InvalidCacheLimitPluginTest : public PluginAcquisitionTest
{
  std::string name() const override {return "test_px4_events_invalid_cache_limit";}
};

TEST_F(InvalidCacheLimitPluginTest, nonpositive_cache_limit_disables_only_persistence)
{
  connect_and_advertise();
  Peer::release_downloads(*instance);
  ASSERT_TRUE(spin_until([this]() {return Peer::decodes(*instance);}));
  EXPECT_EQ(opens.size(), 2U);
  EXPECT_FALSE(std::filesystem::exists(integration_cache_directory));
}

}  // namespace
}  // namespace px4_events
}  // namespace mavros

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mavros::px4_events::TemporaryMetadata cache_location("", ".cache-dir");
  std::filesystem::remove(cache_location.path);
  mavros::px4_events::integration_cache_directory = cache_location.path;
  const std::string parameters =
    "/test_px4_events_auto/px4_events:\n"
    "  ros__parameters:\n"
    "    event_metadata_cache_enabled: false\n"
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
  const std::string cache_parameters =
    "/test_px4_events_acquisition/px4_events:\n"
    "  ros__parameters:\n"
    "    event_metadata_cache_dir: " + cache_location.path + "\n"
    "/test_px4_events_zero_delay/px4_events:\n"
    "  ros__parameters:\n"
    "    event_metadata_download_delay: 0.0\n"
    "    event_metadata_cache_enabled: false\n"
    "/test_px4_events_no_cache/px4_events:\n"
    "  ros__parameters:\n"
    "    event_metadata_cache_enabled: false\n"
    "/test_px4_events_invalid_cache_limit/px4_events:\n"
    "  ros__parameters:\n"
    "    event_metadata_cache_max_bytes: 0\n";
  mavros::px4_events::TemporaryMetadata file(parameters + cache_parameters, ".yaml");
  const char * args[] = {argv[0], "--ros-args", "--params-file", file.path.c_str()};
  rclcpp::init(4, args);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::filesystem::remove_all(cache_location.path);
  return result;
}
