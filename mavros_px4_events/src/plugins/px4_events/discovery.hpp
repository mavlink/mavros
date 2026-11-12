/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief PX4 general metadata advertisement discovery
 * @file discovery.hpp
 * @author Sean Hickey
 */

#ifndef PLUGINS__PX4_EVENTS__DISCOVERY_HPP_
#define PLUGINS__PX4_EVENTS__DISCOVERY_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "metadata.hpp"

namespace mavros
{
namespace px4_events
{

/** Owns only discovery; no file transfer, cache access, or decoder activation. */
class Discovery : public std::enable_shared_from_this<Discovery>
{
public:
  static std::shared_ptr<Discovery> create(const rclcpp::Node::SharedPtr & node);
  ~Discovery();

  // The plugin passes connected=true only for PX4 in automatic mode.
  void connection(bool connected, uint8_t system_id, uint8_t component_id);
  // uri includes the whole fixed-size MAVLink string field, including its terminator.
  void advertisement(uint8_t system_id, uint8_t component_id, std::string_view uri, uint32_t crc);
  std::optional<MetadataFile> metadata() const;

private:
  friend class DiscoveryTestPeer;

  using Clock = std::chrono::steady_clock;
  using Command = mavros_msgs::srv::CommandLong;
  using Client = rclcpp::Client<Command>;

  struct Request
  {
    uint64_t generation;
    uint64_t serial;
    uint32_t message_id;
  };

  static constexpr unsigned MAX_ATTEMPTS = 3;
  static constexpr auto RETRY_INTERVAL = std::chrono::seconds(3);
  static constexpr auto SERVICE_TIMEOUT = std::chrono::seconds(10);

  explicit Discovery(const rclcpp::Node::SharedPtr & node);
  void initialize();
  void tick(Clock::time_point now);
  void response(const Request & request, Client::SharedFuture future);

  // All state helpers below require mutex_. Tests use a synthetic steady clock.
  void reset_locked(bool connected, uint8_t system_id, uint8_t component_id, Clock::time_point now);
  std::optional<Request> poll_locked(Clock::time_point now, bool service_ready);
  void result_locked(const Request & request, uint8_t result, Clock::time_point now);
  bool matches_locked(const Request & request) const;
  void clear_pending_locked();
  void fail_locked(const char * reason);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  Client::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex mutex_;
  uint64_t generation_{0};
  uint64_t serial_{0};
  uint8_t system_id_{0};
  uint8_t component_id_{0};
  uint32_t message_id_{0};
  unsigned attempts_{0};
  bool active_{false};
  Clock::time_point next_request_{};
  Clock::time_point service_deadline_{};
  std::optional<Request> pending_;
  std::optional<int64_t> pending_client_id_;
  std::optional<MetadataFile> metadata_;
};

}  // namespace px4_events
}  // namespace mavros

#endif  // PLUGINS__PX4_EVENTS__DISCOVERY_HPP_
