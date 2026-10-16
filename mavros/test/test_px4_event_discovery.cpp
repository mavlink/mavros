//
// mavros
// Copyright 2026 Sean Hickey, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * @brief Test bounded PX4 metadata discovery and reconnect handling
 * @file test_px4_event_discovery.cpp
 * @author Sean Hickey
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "rcutils/logging.h"
#include "mavconn/mavlink_dialect.hpp"
#include "../src/plugins/px4_events/discovery.hpp"

namespace mavros
{
namespace px4_events
{

// Exercise the real controller with a synthetic clock, without sleeping through retries.
class DiscoveryTestPeer
{
public:
  using Clock = Discovery::Clock;
  using Request = Discovery::Request;

  static void connect(Discovery & discovery, bool connected, Clock::time_point now)
  {
    std::lock_guard<std::mutex> lock(discovery.mutex_);
    discovery.reset_locked(connected, 1, 1, now);
  }

  static std::optional<Request> poll(
    Discovery & discovery, Clock::time_point now,
    bool ready = true)
  {
    std::lock_guard<std::mutex> lock(discovery.mutex_);
    return discovery.poll_locked(now, ready);
  }

  static void result(
    Discovery & discovery, const Request & request, mavlink::common::MAV_RESULT result,
    Clock::time_point now)
  {
    std::lock_guard<std::mutex> lock(discovery.mutex_);
    discovery.result_locked(request, static_cast<uint8_t>(result), now);
  }

  static void missing_ack(Discovery & discovery, const Request & request)
  {
    auto response = std::make_shared<mavros_msgs::srv::CommandLong::Response>();
    response->success = false;
    response->result = static_cast<uint8_t>(mavlink::common::MAV_RESULT::ACCEPTED);
    std::promise<mavros_msgs::srv::CommandLong::Response::SharedPtr> promise;
    promise.set_value(response);
    discovery.response(request, promise.get_future().share());
  }
};

namespace
{
using namespace std::chrono_literals;  // NOLINT
using Peer = DiscoveryTestPeer;
using Result = mavlink::common::MAV_RESULT;
constexpr auto MODERN = mavlink::common::msg::COMPONENT_METADATA::MSG_ID;
constexpr auto LEGACY = mavlink::common::msg::COMPONENT_INFORMATION::MSG_ID;
std::atomic<unsigned> warnings{0};

void capture_log(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list *)
{
  if (severity == RCUTILS_LOG_SEVERITY_WARN &&
    std::string_view(format).find("PX4 event metadata discovery stopped:") == 0)
  {
    ++warnings;
  }
}

class Px4EventDiscoveryTest : public ::testing::Test
{
protected:
  std::shared_ptr<Discovery> discovery;
  Peer::Clock::time_point start{};
  rcutils_logging_output_handler_t previous_handler;

  void SetUp() override
  {
    previous_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_log);
    warnings = 0;
    discovery = Discovery::create(std::make_shared<rclcpp::Node>("event_discovery_test"));
    Peer::connect(*discovery, true, start);
  }

  void TearDown() override
  {
    discovery.reset();
    rcutils_logging_set_output_handler(previous_handler);
  }

  void advertise(uint8_t system = 1, uint8_t component = 1, uint32_t crc = 123)
  {
    const std::string uri = std::string("mftp://etc/extras/component_general.json.xz") + '\0';
    discovery->advertisement(system, component, uri, crc);
  }
};

TEST_F(Px4EventDiscoveryTest, RetriesModernThreeTimesWithoutSilenceFallback)
{
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    const auto now = start + 3s * attempt;
    const auto request = Peer::poll(*discovery, now);
    ASSERT_TRUE(request);
    EXPECT_EQ(request->message_id, MODERN);
    Peer::result(*discovery, *request, Result::ACCEPTED, now);
    EXPECT_FALSE(Peer::poll(*discovery, now + 2999ms));
  }
  EXPECT_FALSE(Peer::poll(*discovery, start + 9s));
  EXPECT_FALSE(Peer::poll(*discovery, start + 30s));
  EXPECT_EQ(warnings.load(), 1U);
}

TEST_F(Px4EventDiscoveryTest, ExplicitDenialStartsThreeLegacyAttempts)
{
  const auto modern = Peer::poll(*discovery, start);
  ASSERT_TRUE(modern);
  Peer::result(*discovery, *modern, Result::DENIED, start);
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    const auto now = start + 3s * attempt;
    const auto request = Peer::poll(*discovery, now);
    ASSERT_TRUE(request);
    EXPECT_EQ(request->message_id, LEGACY);
    Peer::result(*discovery, *request, Result::ACCEPTED, now);
  }
  EXPECT_FALSE(Peer::poll(*discovery, start + 9s));
  EXPECT_EQ(warnings.load(), 1U);
}

TEST_F(Px4EventDiscoveryTest, UnsupportedFallsBackButFailureStops)
{
  auto request = Peer::poll(*discovery, start);
  ASSERT_TRUE(request);
  Peer::result(*discovery, *request, Result::UNSUPPORTED, start);
  request = Peer::poll(*discovery, start);
  ASSERT_TRUE(request);
  EXPECT_EQ(request->message_id, LEGACY);
  Peer::result(*discovery, *request, Result::DENIED, start);
  EXPECT_FALSE(Peer::poll(*discovery, start + 30s));
  EXPECT_EQ(warnings.load(), 1U);

  Peer::connect(*discovery, true, start);
  request = Peer::poll(*discovery, start);
  ASSERT_TRUE(request);
  Peer::result(*discovery, *request, Result::FAILED, start);
  EXPECT_FALSE(Peer::poll(*discovery, start + 30s));
  EXPECT_EQ(warnings.load(), 2U);
}

TEST_F(Px4EventDiscoveryTest, TemporaryRejectionUsesCurrentBudget)
{
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    const auto now = start + 3s * attempt;
    const auto request = Peer::poll(*discovery, now);
    ASSERT_TRUE(request);
    EXPECT_EQ(request->message_id, MODERN);
    Peer::result(*discovery, *request, Result::TEMPORARILY_REJECTED, now);
  }
  EXPECT_FALSE(Peer::poll(*discovery, start + 9s));
}

TEST_F(Px4EventDiscoveryTest, DoesNotOverlapPendingCommandsAndBoundsResponseWait)
{
  ASSERT_TRUE(Peer::poll(*discovery, start));
  EXPECT_FALSE(Peer::poll(*discovery, start + 3s));
  EXPECT_FALSE(Peer::poll(*discovery, start + 9s));
  EXPECT_FALSE(Peer::poll(*discovery, start + 10s));
  EXPECT_FALSE(Peer::poll(*discovery, start + 20s));
  EXPECT_EQ(warnings.load(), 1U);
}

TEST_F(Px4EventDiscoveryTest, UnavailableServiceDoesNotSendOrWaitForever)
{
  EXPECT_FALSE(Peer::poll(*discovery, start, false));
  EXPECT_FALSE(Peer::poll(*discovery, start + 9s, false));
  EXPECT_EQ(warnings.load(), 0U);
  EXPECT_FALSE(Peer::poll(*discovery, start + 10s, false));
  EXPECT_FALSE(Peer::poll(*discovery, start + 20s, true));
  EXPECT_EQ(warnings.load(), 1U);
}

TEST_F(Px4EventDiscoveryTest, MissingAckResponseDoesNotTriggerLegacy)
{
  // ROS command service success=false/result=ACCEPTED is its ACK-timeout representation.
  start = Peer::Clock::now();
  Peer::connect(*discovery, true, start);
  const auto request = Peer::poll(*discovery, start);
  ASSERT_TRUE(request);
  Peer::missing_ack(*discovery, *request);
  const auto retry = Peer::poll(*discovery, start + 3s);
  ASSERT_TRUE(retry);
  EXPECT_EQ(retry->message_id, MODERN);
  Peer::result(*discovery, *request, Result::DENIED, start + 3s);
  EXPECT_FALSE(Peer::poll(*discovery, start + 3s));
  Peer::result(*discovery, *retry, Result::ACCEPTED, start + 3s);
  const auto next = Peer::poll(*discovery, start + 6s);
  ASSERT_TRUE(next);
  EXPECT_EQ(next->message_id, MODERN);
  EXPECT_EQ(warnings.load(), 0U);
}

TEST_F(Px4EventDiscoveryTest, ValidAdvertisementCompletesBeforeAckAndIgnoresOtherTargets)
{
  const auto request = Peer::poll(*discovery, start);
  ASSERT_TRUE(request);
  advertise(2, 1);
  advertise(1, 2);
  EXPECT_FALSE(discovery->metadata());
  advertise(1, 1, 0);
  ASSERT_TRUE(discovery->metadata());
  EXPECT_EQ(discovery->metadata()->crc, 0U);
  EXPECT_EQ(discovery->metadata()->ftp_path, "etc/extras/component_general.json.xz");
  Peer::result(*discovery, *request, Result::DENIED, start);
  advertise(1, 1, 999);
  EXPECT_EQ(discovery->metadata()->crc, 0U);
  EXPECT_FALSE(Peer::poll(*discovery, start + 3s));
  EXPECT_EQ(warnings.load(), 0U);
}

TEST_F(Px4EventDiscoveryTest, LateAdvertisementCanCompleteWhileUsingLegacy)
{
  const auto modern = Peer::poll(*discovery, start);
  ASSERT_TRUE(modern);
  Peer::result(*discovery, *modern, Result::DENIED, start);
  ASSERT_TRUE(Peer::poll(*discovery, start));
  advertise();
  ASSERT_TRUE(discovery->metadata());
  EXPECT_FALSE(Peer::poll(*discovery, start + 3s));
}

TEST_F(Px4EventDiscoveryTest, InvalidAdvertisementStopsOnceWithoutFallback)
{
  for (const std::string uri : {"", "https://example.com/general.json", "mftp://etc/../general"}) {
    Peer::connect(*discovery, true, start);
    discovery->advertisement(1, 1, uri + '\0', 123);
    EXPECT_FALSE(discovery->metadata());
    EXPECT_FALSE(Peer::poll(*discovery, start + 3s));
  }
  EXPECT_EQ(warnings.load(), 3U);
  Peer::connect(*discovery, true, start);
  discovery->advertisement(1, 1, std::string(100, 'x'), 123);
  EXPECT_FALSE(Peer::poll(*discovery, start));
  EXPECT_EQ(warnings.load(), 4U);
}

TEST_F(Px4EventDiscoveryTest, DisconnectClearsMetadataAndOldCallbacksCannotAffectReconnect)
{
  const auto old = Peer::poll(*discovery, start);
  ASSERT_TRUE(old);
  Peer::connect(*discovery, false, start);
  advertise();
  EXPECT_FALSE(discovery->metadata());
  EXPECT_FALSE(Peer::poll(*discovery, start + 3s));
  Peer::connect(*discovery, true, start + 4s);
  const auto current = Peer::poll(*discovery, start + 4s);
  ASSERT_TRUE(current);
  EXPECT_NE(current->generation, old->generation);
  Peer::result(*discovery, *old, Result::DENIED, start + 4s);
  EXPECT_FALSE(Peer::poll(*discovery, start + 7s));
  Peer::result(*discovery, *current, Result::ACCEPTED, start + 4s);
  const auto retry = Peer::poll(*discovery, start + 7s);
  ASSERT_TRUE(retry);
  EXPECT_EQ(retry->message_id, MODERN);
  advertise();
  ASSERT_TRUE(discovery->metadata());
  Peer::connect(*discovery, false, start + 8s);
  EXPECT_FALSE(discovery->metadata());
  EXPECT_EQ(warnings.load(), 0U);
}

}  // namespace
}  // namespace px4_events
}  // namespace mavros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
