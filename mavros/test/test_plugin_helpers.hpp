//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Shared helpers for service-based plugin tests.
 *
 * The fixture class is named `TestUAS` because `mavros::uas::UAS` declares
 * `friend class TestUAS;` (see mavros_uas.hpp), which gives it access to the
 * private plugin routing/sink internals needed to drive a plugin.
 *
 * Each test TU includes its own plugin `.cpp`, so this header is shared and
 * type-agnostic. The plugin is registered directly into the UAS routing tables
 * (bypassing the class-loader / startup timer) and outbound MAVLink is captured
 * from the `mavlink_sink` publisher.
 */

#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <algorithm>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mavconn/interface.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros_msgs/msg/mavlink.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace mavros
{
namespace uas
{

class TestUAS : public ::testing::Test
{
public:
  //! Initialize rclcpp once per test binary (idempotent). Call from main().
  static void Init()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  TestUAS() = default;

  ~TestUAS() override
  {
    stop_spin();
    exec_.reset();
    nodes_.clear();
    sink_subs_.clear();
  }

  //! Real UAS node with the outbound sink created and the startup timer stopped
  //  (so no plugins are auto-loaded and no internal executor starts).
  UAS::SharedPtr create_uas(const std::string & name = "test_mavros_uas")
  {
    auto uas = std::make_shared<UAS>(name);
    uas->startup_delay_timer->cancel();

    // Mirror connect_to_router(): create the sink so send_message() publishes
    // outbound MAVLink to a topic we can capture.
    uas->sink = uas->create_publisher<mavros_msgs::msg::Mavlink>(
      "/" + name + "/mavlink_sink", rclcpp::QoS(1000).best_effort().durability_volatile());

    return uas;
  }

  //! Register a plugin's subscriptions and instance into the UAS routing tables.
  template<class T>
  void register_plugin(UAS::SharedPtr & uas, const std::shared_ptr<T> & plugin)
  {
    for (auto & info : plugin->get_subscriptions()) {
      auto msgid = std::get<0>(info);
      auto it = uas->plugin_subscriptions.find(msgid);
      if (it == uas->plugin_subscriptions.end()) {
        uas->plugin_subscriptions[msgid] = plugin::Plugin::Subscriptions{{info}};
      } else {
        it->second.push_back(info);
      }
    }
    uas->loaded_plugins.push_back(plugin);
  }

  //! Route an inbound MAVLink message to the registered plugin handlers.
  void route(
    UAS::SharedPtr & uas, const mavlink::mavlink_message_t * mmsg,
    const mavconn::Framing framing)
  {
    uas->plugin_route(mmsg, framing);
  }

  //! Serialize a mavlink message into a mavlink_message_t.
  mavlink::mavlink_message_t convert_message(const mavlink::Message & msg)
  {
    mavlink::mavlink_message_t ret;
    mavlink::MsgMap map(ret);
    msg.serialize(map);
    return ret;
  }

  //! Subscribe to the UAS outbound sink and return a shared capture buffer.
  std::shared_ptr<std::vector<mavros_msgs::msg::Mavlink>> capture_sink(
    const UAS::SharedPtr & uas, const std::string & name = "test_mavros_uas")
  {
    auto buf = std::make_shared<std::vector<mavros_msgs::msg::Mavlink>>();
    auto mtx = std::make_shared<std::mutex>();
    sink_subs_.push_back(
      uas->create_subscription<mavros_msgs::msg::Mavlink>(
        "/" + name + "/mavlink_sink",
        rclcpp::QoS(1000).best_effort().durability_volatile(),
        [buf, mtx](const mavros_msgs::msg::Mavlink::SharedPtr m) {
          std::lock_guard<std::mutex> lk(*mtx);
          (void)lk;
          buf->push_back(*m);
        }));
    return buf;
  }

  //! Convert a captured Mavlink ROS message back to a mavlink_message_t.
  static mavlink::mavlink_message_t to_mavlink(const mavros_msgs::msg::Mavlink & rmsg)
  {
    mavlink::mavlink_message_t msg;
    mavros_msgs::mavlink::convert(rmsg, msg);
    return msg;
  }

  //! Start a MultiThreadedExecutor spinning the given nodes on a background thread.
  void start_spin()
  {
    if (spinning_) {
      return;
    }
    spinning_ = true;
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    for (auto & n : nodes_) {
      exec_->add_node(n);
    }
    spin_thread_ = std::thread([this]() {exec_->spin();});
  }

  //! Add a node to the set spun by start_spin().
  void add_node(rclcpp::Node::SharedPtr n)
  {
    nodes_.push_back(n);
  }

  void stop_spin()
  {
    if (!spinning_) {
      return;
    }
    spinning_ = false;
    if (exec_) {
      exec_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  //! Create a client node and add it to the spun set.
  rclcpp::Node::SharedPtr make_client_node(const std::string & name)
  {
    auto node = std::make_shared<rclcpp::Node>(name);
    add_node(node);
    return node;
  }

  //! Synchronous service call with a timeout; returns nullptr on timeout.
  template<class Srv>
  typename Srv::Response::SharedPtr call_service(
    const std::shared_ptr<rclcpp::Client<Srv>> & client,
    const typename Srv::Request::SharedPtr & req,
    const std::chrono::seconds timeout = 5s)
  {
    auto future = client->async_send_request(req);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  //! Wait until the captured outbound buffer contains a message with msgid.
  //  Returns the captured ROS message (first match) or nullptr on timeout.
  static mavros_msgs::msg::Mavlink wait_msgid(
    const std::vector<mavros_msgs::msg::Mavlink> & buf, const uint32_t msgid,
    const std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      for (const auto & m : buf) {
        if (m.msgid == msgid) {
          return m;
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    return mavros_msgs::msg::Mavlink{};
  }

protected:
  std::vector<rclcpp::Node::SharedPtr> nodes_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> sink_subs_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread spin_thread_;
  bool spinning_{false};
};

}   // namespace uas
}   // namespace mavros
