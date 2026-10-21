//
// mavros
// Copyright 2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging.h"
#include "test_plugin_helpers.hpp"
#include "px4_events_test_helpers.hpp"
#include "../src/plugins/sys_status.cpp"  // NOLINT(build/include)

namespace mavros
{
namespace std_plugins
{
class SystemStatusEventTestPeer
{
public:
  static uint64_t generation(SystemStatusPlugin & plugin)
  {
    std::lock_guard<std::recursive_mutex> lock(plugin.event_mutex);
    return plugin.event_generation;
  }

  static void elapsed(SystemStatusPlugin & plugin, unsigned seconds)
  {
    std::lock_guard<std::recursive_mutex> lock(plugin.event_mutex);
    // Advance the clock check, never override the configured delay.
    plugin.event_download_delay_cb(plugin.event_generation,
      plugin.event_connected_at + std::chrono::seconds(seconds));
  }

  static std::shared_ptr<px4_events::Decoder> decoder(SystemStatusPlugin & plugin)
  {
    std::lock_guard<std::recursive_mutex> lock(plugin.event_mutex);
    return plugin.event_decoder;
  }

  static void complete(
    SystemStatusPlugin & plugin, uint64_t generation,
    std::unique_ptr<px4_events::Decoder> decoder)
  {
    plugin.event_acquisition_complete(generation, {std::move(decoder), {}, false});
  }

  static bool startup_timer_cancelled(SystemStatusPlugin & plugin)
  {
    std::lock_guard<std::recursive_mutex> lock(plugin.event_mutex);
    return plugin.event_start_timer && plugin.event_start_timer->is_canceled();
  }
};
}  // namespace std_plugins

namespace uas
{
namespace
{
std::mutex log_mutex;
std::vector<std::pair<int, std::string>> event_logs;
// Parameter-file override isolates test storage; it does not change the
// configured cache-enabled, size-limit, or download-delay defaults.
std::filesystem::path cache_test_root;

void capture_log(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  char text[2048];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(text, sizeof(text), format, copy);
  va_end(copy);
  const std::string message(text);
  if (message.find("FCU: ") == 0 || message.find("PX4 event decoding inactive:") == 0 ||
    message.find("PX4 event decoding failed for ID ") == 0 ||
    message.find("PX4 event acquisition failed:") == 0 ||
    message.find("PX4 event metadata ") == 0 || message.find("PX4 metadata cache ") == 0)
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    event_logs.emplace_back(severity, message);
  }
}

class StatusEventTest : public TestUAS
{
protected:
  UAS::SharedPtr uas;
  std::shared_ptr<std_plugins::SystemStatusPlugin> plugin;
  rcutils_logging_output_handler_t previous_handler;
  int previous_level;

  void SetUp() override
  {
    for (const auto & entry : std::filesystem::directory_iterator(cache_test_root)) {
      std::filesystem::remove_all(entry.path());
    }
    previous_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_log);
    std::lock_guard<std::mutex> lock(log_mutex);
    event_logs.clear();
  }

  void start(const std::string & name, bool px4 = true)
  {
    uas = create_uas(name);
    uas->set_tgt(1, 1);
    uas->update_heartbeat(
      utils::enum_value(mavlink::minimal::MAV_TYPE::QUADROTOR),
      utils::enum_value(px4 ? mavlink::minimal::MAV_AUTOPILOT::PX4 :
      mavlink::minimal::MAV_AUTOPILOT::ARDUPILOTMEGA), 0);
    plugin = std::make_shared<std_plugins::SystemStatusPlugin>(uas.get());
    register_plugin(uas, plugin);
    const auto logger = plugin->get_logger().get_name();
    previous_level = rcutils_logging_get_logger_level(logger);
    ASSERT_EQ(rcutils_logging_set_logger_level(logger, RCUTILS_LOG_SEVERITY_DEBUG), RCUTILS_RET_OK);
  }

  void TearDown() override
  {
    stop_spin();
    if (plugin) {
      EXPECT_EQ(
        rcutils_logging_set_logger_level(plugin->get_logger().get_name(), previous_level),
        RCUTILS_RET_OK);
    }
    rcutils_logging_set_output_handler(previous_handler);
  }

  void inject(uint32_t id = 0x01000007, uint8_t levels = 4)
  {
    mavlink::common::msg::EVENT event{};
    event.id = id;
    event.log_levels = levels;
    event.sequence = 123;
    event.event_time_boot_ms = 456;
    event.arguments[0] = 42;
    auto message = convert_message(event);
    message.sysid = 1;
    message.compid = 1;
    route(uas, &message, mavconn::Framing::ok);
  }

  std::vector<std::pair<int, std::string>> logs()
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    return event_logs;
  }

  void expect_no_metadata_requests(const std::string & name, bool px4 = true)
  {
    auto server = std::make_shared<rclcpp::Node>(name + "_command_stub");
    auto requests = std::make_shared<std::atomic<unsigned>>(0);
    auto service = server->create_service<mavros_msgs::srv::CommandLong>(
      "/" + name + "/cmd/command",
      [requests](mavros_msgs::srv::CommandLong::Request::SharedPtr request,
      mavros_msgs::srv::CommandLong::Response::SharedPtr response) {
        if (request->param1 == mavlink::common::msg::COMPONENT_METADATA::MSG_ID ||
        request->param1 == mavlink::common::msg::COMPONENT_INFORMATION::MSG_ID)
        {
          ++*requests;
        }
        response->success = true;
      });
    start(name, px4);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    executor.add_node(plugin->get_node());
    uas->update_connection_status(true);
    const auto deadline = std::chrono::steady_clock::now() + 600ms;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    EXPECT_EQ(requests->load(), 0U);
    uas->update_connection_status(false);
  }
};

TEST_F(StatusEventTest, local_mode_logs_one_readable_line_and_preserves_raw_topic)
{
  start("test_event_local");
  auto client = make_client_node("status_event_client");
  std::promise<mavros_msgs::msg::StatusEvent> received;
  auto future = received.get_future();
  auto subscription = client->create_subscription<mavros_msgs::msg::StatusEvent>(
    "/test_event_local/status_event", rclcpp::SensorDataQoS(),
    [&received](mavros_msgs::msg::StatusEvent::ConstSharedPtr message) {
      received.set_value(*message);
    });
  add_node(plugin->get_node());
  start_spin();
  for (int i = 0; i < 100 && subscription->get_publisher_count() == 0; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(subscription->get_publisher_count(), 0u);
  inject();
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const auto raw = future.get();
  EXPECT_EQ(raw.px4_id, 7u);
  EXPECT_EQ(raw.severity, 4);
  EXPECT_EQ(raw.sequence, 123);
  EXPECT_EQ(raw.arguments[0], 42);
  for (size_t i = 1; i < raw.arguments.size(); ++i) {
    EXPECT_EQ(raw.arguments[i], 0);
  }
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(lines[0].second, "FCU: Warning next line");
}

TEST_F(StatusEventTest, ordinary_severities_follow_existing_logger_mapping)
{
  start("test_event_local");
  const std::array<int, 8> expected = {
    RCUTILS_LOG_SEVERITY_ERROR, RCUTILS_LOG_SEVERITY_ERROR,
    RCUTILS_LOG_SEVERITY_ERROR, RCUTILS_LOG_SEVERITY_ERROR,
    RCUTILS_LOG_SEVERITY_WARN, RCUTILS_LOG_SEVERITY_WARN,
    RCUTILS_LOG_SEVERITY_INFO, RCUTILS_LOG_SEVERITY_DEBUG};
  for (uint8_t severity = 0; severity < expected.size(); ++severity) {
    inject(0x01000007, severity);
  }
  const auto lines = logs();
  ASSERT_EQ(lines.size(), expected.size());
  for (size_t i = 0; i < lines.size(); ++i) {
    EXPECT_EQ(lines[i].first, expected[i]);
    EXPECT_EQ(lines[i].second, "FCU: Warning next line");
  }
}

TEST_F(StatusEventTest, disabled_protocol_events_are_debug_with_argument_details)
{
  start("test_event_local");
  inject(0x01000003, 0x38);
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_DEBUG);
  EXPECT_EQ(lines[0].second, "FCU: protocol_summary [version=42, checks=0]");
}

TEST_F(StatusEventTest, unknown_event_retains_raw_log)
{
  start("test_event_local");
  inject(0x010000fe);
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(lines[0].second.find("FCU: EVENT 254 with args -42-0-"), 0u);
}

TEST_F(StatusEventTest, disabled_by_default_retains_raw_disabled_severity_behavior)
{
  start("test_event_disabled");
  EXPECT_EQ(plugin->get_node()->get_parameter("event_decoding_mode").as_string(), "disabled");
  EXPECT_TRUE(plugin->get_node()->get_parameter("event_metadata_file").as_string().empty());
  EXPECT_DOUBLE_EQ(plugin->get_node()->get_parameter("event_metadata_download_delay").as_double(),
          30.0);
  inject(0x01000003, 8);
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(lines[0].second.find("FCU: UNK(8): EVENT 3 with args -42-0-"), 0u);
}

TEST_F(StatusEventTest, non_px4_vehicle_retains_raw_log)
{
  start("test_event_local", false);
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].second.find("FCU: EVENT 7 with args -42-0-"), 0u);
}

TEST_F(StatusEventTest, invalid_local_file_reports_once_and_does_not_stop_raw_events)
{
  start("test_event_invalid");
  inject();
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_ERROR);
  EXPECT_EQ(lines[0].second.find("PX4 event decoding inactive:"), 0u);
  EXPECT_EQ(lines[1].second.find("FCU: EVENT 7 with args -42-0-"), 0u);
  EXPECT_EQ(lines[2].second, lines[1].second);
}

TEST_F(StatusEventTest, decoding_parameters_require_restart)
{
  start("test_event_local");
  const auto node = plugin->get_node();
  EXPECT_TRUE(node->describe_parameter("event_decoding_mode").read_only);
  EXPECT_TRUE(node->describe_parameter("event_metadata_file").read_only);
  EXPECT_TRUE(node->describe_parameter("event_metadata_download_delay").read_only);
  for (const auto & name : {"event_metadata_cache_enabled", "event_metadata_cache_dir",
      "event_metadata_cache_max_bytes"})
  {
    EXPECT_TRUE(node->describe_parameter(name).read_only);
  }
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("event_decoding_mode",
          "disabled")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("event_metadata_file",
          "/other.json")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("event_metadata_download_delay",
          10.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("event_metadata_cache_enabled",
          false)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("event_metadata_cache_dir",
          "/other")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("event_metadata_cache_max_bytes",
          int64_t{1})).successful);
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].second, "FCU: Warning next line");
}

TEST_F(StatusEventTest, auto_mode_requests_metadata_and_keeps_events_raw)
{
  using Command = mavros_msgs::srv::CommandLong;
  auto server = make_client_node("event_command_stub");
  auto received = std::make_shared<std::promise<Command::Request>>();
  auto future = received->get_future();
  auto requests = std::make_shared<std::atomic<unsigned>>(0);
  auto service = server->create_service<Command>(
    "/test_event_auto/cmd/command",
    [received, requests](Command::Request::SharedPtr request,
    Command::Response::SharedPtr response) {
      if (request->param1 == mavlink::common::msg::COMPONENT_METADATA::MSG_ID) {
        if (requests->fetch_add(1) == 0) {
          received->set_value(*request);
        }
      }
      response->success = true;
      response->result = utils::enum_value(mavlink::common::MAV_RESULT::ACCEPTED);
    });
  start("test_event_auto");
  add_node(plugin->get_node());
  start_spin();
  uas->update_connection_status(true);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const auto request = future.get();
  EXPECT_FALSE(request.broadcast);
  EXPECT_EQ(request.command, utils::enum_value(mavlink::common::MAV_CMD::REQUEST_MESSAGE));
  EXPECT_EQ(request.param1, mavlink::common::msg::COMPONENT_METADATA::MSG_ID);
  EXPECT_EQ(request.param7, 1.0f);

  mavlink::common::msg::COMPONENT_METADATA advertised{};
  mavlink::set_string(advertised.uri, "mftp://etc/extras/component_general.json.xz");
  advertised.file_crc = 123;
  auto message = convert_message(advertised);
  message.sysid = 1;
  message.compid = 2;
  route(uas, &message, mavconn::Framing::ok);
  EXPECT_TRUE(logs().empty());
  message.compid = 1;
  route(uas, &message, mavconn::Framing::ok);
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 2U);
  EXPECT_EQ(
    lines[0].second, "PX4 event metadata advertised: mftp://etc/extras/component_general.json.xz");
  EXPECT_EQ(lines[1].second.find("FCU: EVENT 7 with args -42-0-"), 0U);
  uas->update_connection_status(false);
  const auto disconnected_lines = logs();
  ASSERT_EQ(disconnected_lines.size(), 3U);
  EXPECT_EQ(disconnected_lines.back().first, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(
    disconnected_lines.back().second,
    "PX4 event acquisition failed: metadata acquisition cancelled; retaining raw logging");
  route(uas, &message, mavconn::Framing::ok);
  EXPECT_EQ(logs(), disconnected_lines);
  stop_spin();
  EXPECT_EQ(requests->load(), 1U);
}

TEST_F(StatusEventTest, disabled_mode_does_not_request_metadata)
{
  expect_no_metadata_requests("test_event_disabled");
  EXPECT_FALSE(std::filesystem::exists(cache_test_root / "cache"));
}

TEST_F(StatusEventTest, auto_mode_accepts_legacy_advertisement_fields)
{
  start("test_event_auto");
  uas->update_connection_status(true);
  mavlink::common::msg::COMPONENT_INFORMATION advertised{};
  mavlink::set_string(advertised.general_metadata_uri,
          "mftp://etc/extras/component_general.json.xz");
  advertised.general_metadata_file_crc = 456;
  auto message = convert_message(advertised);
  message.sysid = 1;
  message.compid = 1;
  route(uas, &message, mavconn::Framing::ok);
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(
    lines[0].second, "PX4 event metadata advertised: mftp://etc/extras/component_general.json.xz");
  uas->update_connection_status(false);
}

TEST_F(StatusEventTest, local_mode_does_not_request_metadata)
{
  expect_no_metadata_requests("test_event_local");
  EXPECT_FALSE(std::filesystem::exists(cache_test_root / "cache"));
}

TEST_F(StatusEventTest, non_px4_auto_mode_does_not_request_metadata)
{
  expect_no_metadata_requests("test_event_auto", false);
}

TEST_F(StatusEventTest, invalid_auto_local_path_does_not_start_discovery)
{
  expect_no_metadata_requests("test_event_auto_invalid");
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 2U);
  EXPECT_EQ(lines[0].second.find("PX4 event decoding inactive:"), 0U);
  EXPECT_EQ(lines[1].second.find("FCU: EVENT 7 with args -42-0-"), 0U);
}

TEST_F(StatusEventTest, invalid_negative_download_delay_retains_raw_logging_without_discovery)
{
  expect_no_metadata_requests("test_event_auto_negative");
  inject();
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 2U);
  EXPECT_NE(lines[0].second.find("finite and nonnegative"), std::string::npos);
  EXPECT_EQ(lines[1].second.find("FCU: EVENT 7 with args -42-0-"), 0U);
}

TEST_F(StatusEventTest, invalid_nonfinite_download_delay_does_not_start_discovery)
{
  expect_no_metadata_requests("test_event_auto_infinite");
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_NE(lines[0].second.find("finite and nonnegative"), std::string::npos);
}

TEST_F(StatusEventTest, large_finite_delay_is_accepted_without_duration_overflow)
{
  start("test_event_auto_large");
  uas->update_connection_status(true);
  std_plugins::SystemStatusEventTestPeer::elapsed(*plugin, 20);
  EXPECT_TRUE(logs().empty());
  uas->update_connection_status(false);
}

TEST_F(StatusEventTest, local_definitions_survive_disconnect)
{
  start("test_event_local");
  const auto decoder = std_plugins::SystemStatusEventTestPeer::decoder(*plugin);
  uas->update_connection_status(true);
  uas->update_connection_status(false);
  EXPECT_EQ(std_plugins::SystemStatusEventTestPeer::decoder(*plugin), decoder);
  inject();
  ASSERT_EQ(logs().size(), 1U);
  EXPECT_EQ(logs()[0].second, "FCU: Warning next line");
}

class AutoStatusEventTest : public StatusEventTest
{
protected:
  using Peer = std_plugins::SystemStatusEventTestPeer;
  using Open = mavros_msgs::srv::FileOpen;
  using Read = mavros_msgs::srv::FileRead;
  using Close = mavros_msgs::srv::FileClose;
  rclcpp::Node::SharedPtr server;
  rclcpp::Service<Open>::SharedPtr open_service;
  rclcpp::Service<Read>::SharedPtr read_service;
  rclcpp::Service<Close>::SharedPtr close_service;
  rclcpp::Service<mavros_msgs::srv::CommandLong>::SharedPtr command_service;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::string general;
  std::string definitions;
  std::vector<std::string> opens;
  bool missing_file{false};
  bool hold_read{false};
  std::shared_ptr<rmw_request_id_t> held_read;

  void start_auto(const std::string & name = "test_event_auto")
  {
    std::ifstream file(TEST_EVENT_METADATA);
    definitions.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    ASSERT_FALSE(definitions.empty());
    general =
      "{\"version\":1,\"metadataTypes\":[{\"type\":4,\"uri\":\"mftp://events.json\",\"fileCrc\":" +
      std::to_string(px4_events::metadata_crc32(definitions)) + "}]}";
    server = std::make_shared<rclcpp::Node>("auto_event_ftp_stub");
    const auto prefix = "/" + name;
    open_service = server->create_service<Open>(prefix + "/ftp/open",
        [this](Open::Request::SharedPtr request, Open::Response::SharedPtr response) {
          opens.push_back(request->file_path);
          response->success = !missing_file;
          response->r_errno = missing_file ? ENOENT : 0;
          response->size = request->file_path ==
          "general.json" ? general.size() : definitions.size();
      });
    read_service = server->create_service<Read>(prefix + "/ftp/read",
        [this](std::shared_ptr<rmw_request_id_t> header, Read::Request::SharedPtr request) {
          if (hold_read) {
            held_read = header;
            return;
          }
          Read::Response response;
          response.success = true;
          const auto & bytes = request->file_path == "general.json" ? general : definitions;
          const auto chunk = bytes.substr(request->offset, request->size);
          response.data.assign(chunk.begin(), chunk.end());
          read_service->send_response(*header, response);
      });
    close_service = server->create_service<Close>(
      prefix + "/ftp/close",
      [](Close::Request::SharedPtr, Close::Response::SharedPtr response) {
        response->success = true;
      });
    command_service = server->create_service<mavros_msgs::srv::CommandLong>(
      prefix + "/cmd/command",
      [](mavros_msgs::srv::CommandLong::Request::SharedPtr,
      mavros_msgs::srv::CommandLong::Response::SharedPtr response) {
        response->success = true;
      });
    start(name);
    executor.add_node(plugin->get_node());
    executor.add_node(server);
    uas->update_connection_status(true);
  }

  void advertise(bool legacy = false)
  {
    mavlink::mavlink_message_t message;
    if (legacy) {
      mavlink::common::msg::COMPONENT_INFORMATION metadata{};
      mavlink::set_string(metadata.general_metadata_uri, "mftp://general.json");
      metadata.general_metadata_file_crc = px4_events::metadata_crc32(general);
      message = convert_message(metadata);
    } else {
      mavlink::common::msg::COMPONENT_METADATA metadata{};
      mavlink::set_string(metadata.uri, "mftp://general.json");
      metadata.file_crc = px4_events::metadata_crc32(general);
      message = convert_message(metadata);
    }
    message.sysid = 1;
    message.compid = 1;
    route(uas, &message, mavconn::Framing::ok);
  }

  bool spin_until(const std::function<bool ()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + 4s;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
    return done();
  }

  void seed_cache(bool with_general = true, bool with_events = true)
  {
    px4_events::MetadataCache cache(cache_test_root / "cache");
    if (with_general) {
      ASSERT_TRUE(cache.save(px4_events::metadata_file("mftp://general.json",
        px4_events::metadata_crc32(general)), px4_events::MetadataRole::General, general).empty());
    }
    if (with_events) {
      ASSERT_TRUE(cache.save(px4_events::metadata_file("mftp://events.json",
        px4_events::metadata_crc32(definitions)), px4_events::MetadataRole::Events,
        definitions).empty());
    }
  }

  void TearDown() override
  {
    if (plugin) {
      uas->update_connection_status(false);
      executor.remove_node(plugin->get_node());
    }
    if (server) {
      executor.remove_node(server);
    }
    StatusEventTest::TearDown();
  }
};

TEST_F(AutoStatusEventTest, default_delay_gates_ftp_then_activates_and_preserves_raw_topic)
{
  start_auto();
  EXPECT_TRUE(plugin->get_node()->get_parameter("event_metadata_cache_enabled").as_bool());
  EXPECT_EQ(plugin->get_node()->get_parameter("event_metadata_cache_max_bytes").as_int(), 16777216);
  advertise();
  Peer::elapsed(*plugin, 29);
  executor.spin_some();
  EXPECT_TRUE(opens.empty());
  inject();
  EXPECT_EQ(logs().back().second.find("FCU: EVENT 7 with args -42-0-"), 0U);
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  EXPECT_EQ(opens, (std::vector<std::string>{"general.json", "events.json"}));
  std::optional<mavros_msgs::msg::StatusEvent> raw;
  auto subscription = server->create_subscription<mavros_msgs::msg::StatusEvent>(
    "/test_event_auto/status_event", rclcpp::SensorDataQoS(),
    [&raw](mavros_msgs::msg::StatusEvent::ConstSharedPtr message) {raw = *message;});
  ASSERT_TRUE(spin_until([&subscription]() {return subscription->get_publisher_count() > 0;}));
  inject();
  ASSERT_TRUE(spin_until([&raw]() {return raw.has_value();}));
  EXPECT_EQ(raw->px4_id, 7U);
  EXPECT_EQ(raw->sequence, 123);
  EXPECT_EQ(raw->severity, 4);
  EXPECT_EQ(raw->arguments[0], 42);
  EXPECT_EQ(logs().back().second, "FCU: Warning next line");
  advertise();
  Peer::elapsed(*plugin, 40);
  executor.spin_some();
  EXPECT_EQ(opens.size(), 2U);
  const auto snapshot = Peer::decoder(*plugin);
  uas->update_connection_status(false);
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);
  EXPECT_TRUE(snapshot->decode(0x01000007, 4, {}));
  inject();
  EXPECT_EQ(logs().back().second.find("FCU: EVENT 7 with args -42-0-"), 0U);
}

TEST_F(AutoStatusEventTest, warm_cache_activates_before_download_delay_without_ftp_services)
{
  start_auto();
  seed_cache();
  open_service.reset();
  read_service.reset();
  close_service.reset();
  advertise();
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  EXPECT_TRUE(opens.empty());
  EXPECT_TRUE(Peer::startup_timer_cancelled(*plugin));
  inject();
  EXPECT_EQ(logs().back().second, "FCU: Warning next line");
}

TEST_F(AutoStatusEventTest, both_partial_hit_directions_wait_for_the_original_download_delay)
{
  start_auto();
  for (const bool general_hit : {true, false}) {
    std::filesystem::remove_all(cache_test_root / "cache");
    seed_cache(general_hit, !general_hit);
    advertise();
    Peer::elapsed(*plugin, 29);
    executor.spin_some();
    EXPECT_TRUE(opens.empty());
    EXPECT_EQ(Peer::decoder(*plugin), nullptr);
    Peer::elapsed(*plugin, 30);
    ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
    EXPECT_EQ(opens, (std::vector<std::string>{general_hit ? "events.json" : "general.json"}));
    uas->update_connection_status(false);
    opens.clear();
    uas->update_connection_status(true);
  }
}

TEST_F(AutoStatusEventTest, reconnect_revalidates_saved_metadata_without_another_download)
{
  start_auto();
  advertise();
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  ASSERT_EQ(opens.size(), 2U);
  uas->update_connection_status(false);
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);
  opens.clear();
  uas->update_connection_status(true);
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);  // Need this connection's advertisement.
  advertise();
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  EXPECT_TRUE(opens.empty());
}

TEST_F(AutoStatusEventTest, disabled_cache_ignores_existing_entries_and_does_not_save)
{
  start_auto("test_event_auto_cache_off");
  seed_cache();
  advertise();
  Peer::elapsed(*plugin, 29);
  executor.spin_some();
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);
  EXPECT_TRUE(opens.empty());
  std::filesystem::remove_all(cache_test_root / "cache");
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  EXPECT_EQ(opens.size(), 2U);
  EXPECT_FALSE(std::filesystem::exists(cache_test_root / "cache"));
}

TEST_F(AutoStatusEventTest, zero_cache_limit_reports_error_but_still_downloads_and_decodes)
{
  start_auto("test_event_auto_cache_zero");
  const auto lines = logs();
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(lines[0].first, RCUTILS_LOG_SEVERITY_ERROR);
  EXPECT_NE(lines[0].second.find("must be positive"), std::string::npos);
  advertise();
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  EXPECT_EQ(opens.size(), 2U);
  EXPECT_FALSE(std::filesystem::exists(cache_test_root / "cache"));
}

TEST_F(AutoStatusEventTest, negative_cache_limit_disables_only_persistence)
{
  start_auto("test_event_auto_cache_negative");
  advertise();
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  EXPECT_EQ(opens.size(), 2U);
  EXPECT_FALSE(std::filesystem::exists(cache_test_root / "cache"));
}

TEST_F(AutoStatusEventTest, unavailable_cache_directory_warns_but_decoding_remains_available)
{
  {
    std::ofstream blocked(cache_test_root / "blocked");
    blocked << "existing file";
  }
  start_auto("test_event_auto_cache_blocked");
  advertise();
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  const auto lines = logs();
  EXPECT_TRUE(std::any_of(lines.begin(), lines.end(), [](const auto & line) {
      return line.first == RCUTILS_LOG_SEVERITY_WARN &&
             line.second.find("PX4 metadata cache save failed:") == 0;
    }));
}

TEST_F(AutoStatusEventTest, small_positive_cache_limit_is_honored_without_disabling_decoding)
{
  start_auto("test_event_auto_cache_tiny");
  EXPECT_EQ(plugin->get_node()->get_parameter("event_metadata_cache_max_bytes").as_int(), 1);
  advertise();
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
  EXPECT_FALSE(std::filesystem::exists(cache_test_root / "cache"));
  const auto lines = logs();
  EXPECT_TRUE(std::any_of(lines.begin(), lines.end(), [](const auto & line) {
      return line.first == RCUTILS_LOG_SEVERITY_WARN &&
             line.second.find("exceeds configured cache limit") != std::string::npos;
    }));
}

TEST_F(AutoStatusEventTest, disconnect_before_scheduled_cache_lookup_prevents_activation)
{
  start_auto();
  seed_cache();
  advertise();
  uas->update_connection_status(false);
  executor.spin_some();
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);
  EXPECT_TRUE(opens.empty());
}

TEST_F(AutoStatusEventTest, late_legacy_advertisement_starts_after_elapsed_delay)
{
  start_auto();
  Peer::elapsed(*plugin, 30);
  executor.spin_some();
  EXPECT_TRUE(opens.empty());
  advertise(true);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
}

TEST_F(AutoStatusEventTest, explicitly_configured_zero_delay_starts_on_advertisement)
{
  start_auto("test_event_auto_zero");
  EXPECT_DOUBLE_EQ(plugin->get_node()->get_parameter("event_metadata_download_delay").as_double(),
          0.0);
  advertise();
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
}

TEST_F(AutoStatusEventTest, terminal_failure_stays_raw_until_next_connection)
{
  start_auto();
  missing_file = true;
  advertise();
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {
      const auto lines = logs();
      return std::any_of(lines.begin(), lines.end(), [](const auto & line) {
               return line.second.find("PX4 event acquisition failed:") == 0;
      });
    }));
  advertise();
  Peer::elapsed(*plugin, 40);
  executor.spin_some();
  EXPECT_EQ(opens.size(), 1U);
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);
  inject();
  EXPECT_EQ(logs().back().second.find("FCU: EVENT 7 with args -42-0-"), 0U);
  uas->update_connection_status(false);
  missing_file = false;
  uas->update_connection_status(true);
  advertise();
  Peer::elapsed(*plugin, 29);
  executor.spin_some();
  EXPECT_EQ(opens.size(), 1U);
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
}

TEST_F(AutoStatusEventTest, disconnect_cancels_and_rejects_old_completion_and_late_reply)
{
  start_auto();
  const auto generation = Peer::generation(*plugin);
  hold_read = true;
  advertise();
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return held_read != nullptr;}));
  uas->update_connection_status(false);
  uas->update_connection_status(true);
  auto stale = std::make_unique<px4_events::Decoder>();
  ASSERT_TRUE(stale->load_json(definitions).empty());
  Peer::complete(*plugin, generation, std::move(stale));
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);
  Read::Response late;
  late.success = true;
  late.data.assign(general.begin(), general.end());
  read_service->send_response(*held_read, late);
  hold_read = false;
  advertise();
  Peer::elapsed(*plugin, 29);
  executor.spin_some();
  EXPECT_EQ(opens.size(), 1U);
  Peer::elapsed(*plugin, 30);
  ASSERT_TRUE(spin_until([this]() {return Peer::decoder(*plugin) != nullptr;}));
}

TEST_F(AutoStatusEventTest, concurrent_completion_and_disconnect_cannot_leave_decoder_active)
{
  start_auto();
  const auto generation = Peer::generation(*plugin);
  auto decoder = std::make_unique<px4_events::Decoder>();
  ASSERT_TRUE(decoder->load_json(definitions).empty());
  std::promise<void> release;
  auto ready = release.get_future();
  std::thread completion([this, generation, decoder = std::move(decoder), &ready]() mutable {
      ready.wait();
      Peer::complete(*plugin, generation, std::move(decoder));
    });
  release.set_value();
  uas->update_connection_status(false);
  completion.join();
  EXPECT_EQ(Peer::decoder(*plugin), nullptr);
}

}  // namespace
}  // namespace uas
}  // namespace mavros

int main(int argc, char ** argv)
{
  char pattern[] = "/tmp/mavros-status-cache-XXXXXX";
  const auto directory = mkdtemp(pattern);
  if (!directory) {
    return 1;
  }
  mavros::uas::cache_test_root = directory;
  const auto root = std::string(directory);
  const std::string parameters =
    "/**/sys:\n"
    "  ros__parameters:\n"
    "    event_metadata_cache_dir: '" + root + "/cache'\n" +
    "/test_event_local/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: local\n"
    "    event_metadata_file: '" TEST_EVENT_METADATA "'\n"
    "/test_event_invalid/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: local\n"
    "    event_metadata_file: /file/that/does/not/exist\n"
    "/test_event_auto/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "/test_event_auto_invalid/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_file: /file/that/does/not/exist\n"
    "/test_event_auto_zero/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_download_delay: 0.0\n"
    "/test_event_auto_negative/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_download_delay: -1.0\n"
    "/test_event_auto_infinite/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_download_delay: .inf\n"
    "/test_event_auto_large/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_download_delay: 1.0e+300\n"
    "/test_event_auto_cache_off/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_cache_enabled: false\n"
    "/test_event_auto_cache_zero/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_cache_max_bytes: 0\n"
    "/test_event_auto_cache_negative/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_cache_max_bytes: -1\n"
    "/test_event_auto_cache_tiny/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_cache_max_bytes: 1\n"
    "/test_event_auto_cache_blocked/sys:\n"
    "  ros__parameters:\n"
    "    event_decoding_mode: auto\n"
    "    event_metadata_cache_dir: '" + root + "/blocked'\n";
  mavros::px4_events::TemporaryMetadata file(parameters, ".yaml");
  const char * ros_args[] = {"mavros-status-event-test", "--ros-args", "--params-file",
    file.path.c_str()};
  rclcpp::init(4, ros_args);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::error_code error;
  std::filesystem::remove_all(mavros::uas::cache_test_root, error);
  return result;
}
