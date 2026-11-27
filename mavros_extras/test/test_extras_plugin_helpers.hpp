//
// mavros
// Copyright 2026 Vishwa (vishwagw), All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Helpers shared by the mavros_extras plugin tests, on top of the core
 * `TestUAS` fixture from mavros/test/test_plugin_helpers.hpp.
 */

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "test_plugin_helpers.hpp"

namespace mavros
{
namespace uas
{

//! Thread-safe store of the messages received on one ROS topic.
template<class Msg>
class TopicCapture
{
public:
  void push_back(const Msg & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msgs_.push_back(m);
  }

  std::vector<Msg> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return msgs_;
  }

  //! Wait until at least @a count messages arrived; true on success.
  bool wait_for(const size_t count, const std::chrono::milliseconds timeout = 5s) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (snapshot().size() >= count) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return snapshot().size() >= count;
  }

private:
  mutable std::mutex mutex_;
  std::vector<Msg> msgs_;
};

//! Subscribe @a node to @a topic and collect what is published there.
template<class Msg>
std::shared_ptr<TopicCapture<Msg>> capture_topic(
  const rclcpp::Node::SharedPtr & node, const std::string & topic,
  std::vector<rclcpp::SubscriptionBase::SharedPtr> & keep_alive)
{
  auto buf = std::make_shared<TopicCapture<Msg>>();
  keep_alive.push_back(
    node->create_subscription<Msg>(
      topic, 10, [buf](const typename Msg::SharedPtr m) {buf->push_back(*m);}));
  return buf;
}

//! Wait until the publisher has a matched subscription (DDS discovery).
template<class PubT>
bool wait_matched(const PubT & pub, const std::chrono::milliseconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  return pub->get_subscription_count() > 0;
}

//! Wait until every subscription has a matched publisher (DDS discovery).
inline bool wait_connected(
  const std::vector<rclcpp::SubscriptionBase::SharedPtr> & subs,
  const std::chrono::milliseconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto all_connected = [&subs]() {
      for (const auto & s : subs) {
        if (s->get_publisher_count() == 0) {
          return false;
        }
      }
      return true;
    };
  while (!all_connected() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  return all_connected();
}

//! Messages with the given msgid in a sink capture, decoded as MAVLink type M.
template<class M>
std::vector<M> sent_messages(const TestUAS::CaptureBuffer & buf)
{
  std::vector<M> ret;
  for (const auto & rmsg : buf.snapshot()) {
    if (rmsg.msgid != M::MSG_ID) {
      continue;
    }
    auto mmsg = TestUAS::to_mavlink(rmsg);
    mavlink::MsgMap map(mmsg);
    M m{};
    m.deserialize(map);
    ret.push_back(m);
  }
  return ret;
}

//! Wait until the sink capture holds at least @a count messages of type M.
template<class M>
std::vector<M> wait_sent(
  const TestUAS::CaptureBuffer & buf, const size_t count,
  const std::chrono::milliseconds timeout = 5s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto ret = sent_messages<M>(buf);
  while (ret.size() < count && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
    ret = sent_messages<M>(buf);
  }
  return ret;
}

}   // namespace uas
}   // namespace mavros
