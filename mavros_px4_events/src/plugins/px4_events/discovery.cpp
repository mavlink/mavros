/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Bounded asynchronous PX4 metadata discovery
 * @file discovery.cpp
 * @author Sean Hickey
 */

#include "discovery.hpp"

#include <exception>
#include <string>

#include "mavconn/mavlink_dialect.hpp"

namespace mavros
{
namespace px4_events
{
namespace
{
constexpr auto MODERN_MESSAGE = mavlink::common::msg::COMPONENT_METADATA::MSG_ID;
constexpr auto LEGACY_MESSAGE = mavlink::common::msg::COMPONENT_INFORMATION::MSG_ID;
using Result = mavlink::common::MAV_RESULT;
}  // namespace

std::shared_ptr<Discovery> Discovery::create(const rclcpp::Node::SharedPtr & node)
{
  auto discovery = std::shared_ptr<Discovery>(new Discovery(node));
  discovery->initialize();
  return discovery;
}

Discovery::Discovery(const rclcpp::Node::SharedPtr & node)
: node_(node)
{}

void Discovery::initialize()
{
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
#ifdef USE_OLD_RMW_QOS
  auto service_qos = rmw_qos_profile_services_default;
#else
  auto service_qos = rclcpp::ServicesQoS();
#endif
  client_ = node_->create_client<Command>(
    "cmd/command", service_qos, callback_group_);
  const std::weak_ptr<Discovery> weak = shared_from_this();
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(250), [weak]() {
        if (const auto discovery = weak.lock()) {
          discovery->tick(Clock::now());
        }
    }, callback_group_);
  timer_->cancel();
}

Discovery::~Discovery()
{
  if (timer_) {
    timer_->cancel();
  }
  if (pending_client_id_) {
    client_->remove_pending_request(*pending_client_id_);
  }
}

void Discovery::clear_pending_locked()
{
  if (pending_client_id_) {
    client_->remove_pending_request(*pending_client_id_);
  }
  pending_client_id_.reset();
  pending_.reset();
}

void Discovery::reset_locked(
  bool connected, uint8_t system_id, uint8_t component_id, Clock::time_point now)
{
  ++generation_;
  clear_pending_locked();
  metadata_.reset();
  system_id_ = system_id;
  component_id_ = component_id;
  message_id_ = MODERN_MESSAGE;
  attempts_ = 0;
  active_ = connected;
  next_request_ = now;
  service_deadline_ = now + SERVICE_TIMEOUT;
  if (active_) {
    timer_->reset();
  } else {
    timer_->cancel();
  }
}

void Discovery::connection(bool connected, uint8_t system_id, uint8_t component_id)
{
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reset_locked(connected, system_id, component_id, now);
  }
  // No 20-second download delay applies to the small advertisement exchange.
  if (connected) {
    tick(now);
  }
}

void Discovery::fail_locked(const char * reason)
{
  active_ = false;
  clear_pending_locked();
  timer_->cancel();
  RCLCPP_WARN(
    node_->get_logger(), "PX4 event metadata discovery stopped: %s; retaining raw event logging",
    reason);
}

std::optional<Discovery::Request> Discovery::poll_locked(Clock::time_point now, bool service_ready)
{
  if (!active_) {
    return std::nullopt;
  }
  if (pending_) {
    if (now >= service_deadline_) {
      // Removing a ROS pending request does not cancel the command on the server.
      // Stop rather than enqueue another command while the old one is unresolved.
      fail_locked("command service response timeout");
    }
    return std::nullopt;
  }
  if (now < next_request_) {
    return std::nullopt;
  }
  if (attempts_ == MAX_ATTEMPTS) {
    fail_locked("advertisement retry budget exhausted");
    return std::nullopt;
  }
  if (!service_ready) {
    if (now >= service_deadline_) {
      fail_locked("command service unavailable");
    }
    return std::nullopt;
  }
  ++attempts_;
  pending_ = Request{generation_, ++serial_, message_id_};
  next_request_ = now + RETRY_INTERVAL;
  service_deadline_ = now + SERVICE_TIMEOUT;
  return pending_;
}

void Discovery::tick(Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return;
  }
  try {
    const auto request = poll_locked(now, client_->service_is_ready());
    if (!request) {
      return;
    }
    auto command = std::make_shared<Command::Request>();
    command->broadcast = false;
    command->command = static_cast<uint16_t>(mavlink::common::MAV_CMD::REQUEST_MESSAGE);
    command->param1 = request->message_id;
    command->param7 = 1.0;  // Ask for the reply to be addressed to the requester.
    const std::weak_ptr<Discovery> weak = shared_from_this();
    const auto future = client_->async_send_request(
      command, [weak, token = *request](Client::SharedFuture response) {
        if (const auto discovery = weak.lock()) {
          discovery->response(token, response);
        }
      });
    pending_client_id_ = future.request_id;
  } catch (const std::exception & ex) {
    fail_locked(ex.what());
  }
}

bool Discovery::matches_locked(const Request & request) const
{
  return active_ && pending_ && request.generation == generation_ &&
         request.serial == pending_->serial;
}

void Discovery::result_locked(const Request & request, uint8_t result, Clock::time_point now)
{
  if (!matches_locked(request)) {
    return;
  }
  clear_pending_locked();
  if (result == static_cast<uint8_t>(Result::DENIED) ||
    result == static_cast<uint8_t>(Result::UNSUPPORTED))
  {
    if (message_id_ == MODERN_MESSAGE) {
      message_id_ = LEGACY_MESSAGE;
      attempts_ = 0;
      next_request_ = now;
      service_deadline_ = now + SERVICE_TIMEOUT;
    } else {
      fail_locked("legacy metadata request rejected");
    }
  } else {
    if (result != static_cast<uint8_t>(Result::ACCEPTED) &&
      result != static_cast<uint8_t>(Result::TEMPORARILY_REJECTED))
    {
      fail_locked("metadata request failed");
    } else {
      // An ACK does not contain the URI. Wait for the advertisement or next retry.
      service_deadline_ = now + SERVICE_TIMEOUT;
    }
  }
}

void Discovery::response(const Request & request, Client::SharedFuture future)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!matches_locked(request)) {
    return;
  }
  try {
    const auto response = future.get();
    if (!response->success && response->result == static_cast<uint8_t>(Result::ACCEPTED)) {
      // The command plugin leaves ACCEPTED as its default result on ACK timeout.
      // Treat unsuccessful ACCEPTED as silence, not a positive acknowledgement.
      result_locked(request, static_cast<uint8_t>(Result::TEMPORARILY_REJECTED), Clock::now());
    } else {
      result_locked(request, response->result, Clock::now());
    }
  } catch (const std::exception & ex) {
    fail_locked(ex.what());
  }
}

void Discovery::advertisement(
  uint8_t system_id, uint8_t component_id, std::string_view uri, uint32_t crc)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || system_id != system_id_ || component_id != component_id_) {
    return;
  }
  try {
    const auto terminator = uri.find('\0');
    if (terminator == std::string_view::npos) {
      fail_locked("metadata URI is not terminated");
      return;
    }
    metadata_ = metadata_file(std::string(uri.substr(0, terminator)), crc);
    active_ = false;
    clear_pending_locked();
    timer_->cancel();
    RCLCPP_DEBUG(node_->get_logger(), "PX4 event metadata advertised: %s", metadata_->uri.c_str());
  } catch (const std::exception & ex) {
    fail_locked(ex.what());
  }
}

std::optional<MetadataFile> Discovery::metadata() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return metadata_;
}

}  // namespace px4_events
}  // namespace mavros
