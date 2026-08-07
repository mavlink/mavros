// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Google Benchmark for the mavros router with real UDP endpoints.
 *
 * Measures sustained routing throughput: several FCU UDP streams feed the
 * router and the benchmark reports how many messages each UAS endpoint
 * receives per second. Run duration is controlled by Google Benchmark's
 * --benchmark_min_time (CI uses 10s).
 *
 * Skipped when MAVROS_SKIP_STRESS_TESTS=1.
 */

#include <benchmark/benchmark.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <asio.hpp>

#include "rclcpp/executors.hpp"
#include "mavconn/thread_utils.hpp"
#include "mavros/mavros_router.hpp"
#include "mavros_msgs/msg/mavlink.hpp"

using namespace std::chrono_literals;  // NOLINT
using mavconn::utils::format;

namespace mavros
{
namespace router
{

static int udp_base()
{
  if (const char * env = std::getenv("MAVROS_TEST_UDP_BASE")) {
    return std::atoi(env);
  }
  return 15500;
}

static int fcu_bind(int f) {return udp_base() + f;}
static int fcu_remote(int f) {return udp_base() + 100 + f;}

template<typename M>
static size_t build_frame(const M & msg, uint8_t sysid, uint8_t compid, uint8_t * wire)
{
  mavlink::mavlink_message_t mmsg{};
  mavlink::MsgMap mmap(mmsg);
  mmsg.sysid = sysid;
  mmsg.compid = compid;
  msg.serialize(mmap);
  const auto & mi = msg.get_message_info();
  mavlink::mavlink_status_t status{};
  mavlink::mavlink_finalize_message_buffer(
    &mmsg, sysid, compid, &status, mi.min_length, mi.length, mi.crc_extra);
  return mavlink::mavlink_msg_to_send_buffer(wire, &mmsg);
}

class BenchmarkRouter : public benchmark::Fixture
{
public:
  void SetUp(const benchmark::State &) override
  {
    rclcpp::init(0, nullptr);
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);
    router_ = std::make_shared<Router>(opts, "bench_router");

    fcu_count_ = 3;
    uas_count_ = 4;

    std::vector<std::string> fcu_urls, uas_urls;
    for (int f = 0; f < fcu_count_; f++) {
      fcu_urls.push_back(
        format("udp://127.0.0.1:%d@127.0.0.1:%d", fcu_bind(f), fcu_remote(f)));
    }
    for (int u = 0; u < uas_count_; u++) {
      uas_urls.push_back(format("/bench_uas%d", u + 1));
    }
    router_->set_parameters({
          rclcpp::Parameter("fcu_urls", fcu_urls),
          rclcpp::Parameter("gcs_urls", std::vector<std::string>{}),
          rclcpp::Parameter("uas_urls", uas_urls),
    });

    exec_->add_node(router_);
    exit_promise_ = std::make_shared<std::promise<void>>();
    exit_future_ = exit_promise_->get_future().share();
    spin_thread_ = std::thread([this]() {exec_->spin_until_future_complete(exit_future_);});
    std::this_thread::sleep_for(1s);

    // Subscribe to each UAS endpoint and teach the router their addresses.
    for (int u = 0; u < uas_count_; u++) {
      uas_recv_.push_back(std::make_shared<std::atomic<size_t>>(0));
      subs_.push_back(
        router_->create_subscription<mavros_msgs::msg::Mavlink>(
          format("/bench_uas%d/mavlink_source", u + 1), rclcpp::QoS(1000).best_effort(),
          [u, this](const mavros_msgs::msg::Mavlink::SharedPtr) {(*uas_recv_[u])++;}));
      pubs_.push_back(
        router_->create_publisher<mavros_msgs::msg::Mavlink>(
          format("/bench_uas%d/mavlink_sink", u + 1), rclcpp::QoS(1000).best_effort()));
      mavros_msgs::msg::Mavlink rmsg{};
      rmsg.header.stamp.sec = 1;
      rmsg.sysid = static_cast<uint8_t>((u + 1) * 10);
      rmsg.compid = 1;
      rmsg.msgid = 0;  // HEARTBEAT
      pubs_.back()->publish(rmsg);
    }

    // FCU UDP senders bound to the router's remote ports.
    for (int f = 0; f < fcu_count_; f++) {
      asio::ip::udp::socket s(io_);
      s.open(asio::ip::udp::v4());
      s.set_option(asio::ip::udp::socket::reuse_address(true));
      s.bind(
        asio::ip::udp::endpoint(
          asio::ip::make_address("127.0.0.1"), fcu_remote(f)));
      s.connect(
        asio::ip::udp::endpoint(
          asio::ip::make_address("127.0.0.1"), fcu_bind(f)));
      fcu_socks_.push_back(std::move(s));
    }

    mavlink::minimal::msg::HEARTBEAT hb{};
    hb.type = 2;
    hb.autopilot = 3;
    hb.base_mode = 80;
    hb.system_status = 2;
    hb_len_ = build_frame(hb, 1, 1, hb_wire_);

    // warm up so endpoints learn remotes
    for (int f = 0; f < fcu_count_; f++) {
      fcu_socks_[f].send(asio::buffer(hb_wire_, hb_len_));
    }
    std::this_thread::sleep_for(500ms);
    for (auto & c : uas_recv_) {c->store(0);}
  }

  void TearDown(const benchmark::State &) override
  {
    exit_promise_->set_value();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    router_->endpoints.clear();
    router_->remote_index.clear();
    // Destroy the ROS entities (DDS participants) before shutdown so the
    // middleware does not outlive the global context.
    subs_.clear();
    pubs_.clear();
    fcu_socks_.clear();
    router_.reset();
    rclcpp::shutdown();
  }

  size_t delivered()
  {
    size_t total = 0;
    for (auto & c : uas_recv_) {total += c->load();}
    return total;
  }

protected:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread spin_thread_;
  std::shared_ptr<std::promise<void>> exit_promise_;
  std::shared_future<void> exit_future_;
  asio::io_context io_;
  Router::SharedPtr router_;
  std::vector<rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr> subs_;
  std::vector<rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr> pubs_;
  std::vector<std::shared_ptr<std::atomic<size_t>>> uas_recv_;
  std::vector<asio::ip::udp::socket> fcu_socks_;
  uint8_t hb_wire_[512];
  size_t hb_len_{0};
  int fcu_count_{0};
  int uas_count_{0};
};

// Sustained broadcast throughput: each iteration sends a batch of heartbeats
// from every FCU and waits for delivery; reports messages delivered per
// second across all UAS endpoints.
BENCHMARK_DEFINE_F(BenchmarkRouter, sustained)(benchmark::State & state)
{
  const int batch = static_cast<int>(state.range(0));
  size_t total_delivered = 0;

  for (auto _ : state) {
    const size_t before = delivered();
    for (int f = 0; f < fcu_count_; f++) {
      for (int i = 0; i < batch; i++) {
        fcu_socks_[f].send(asio::buffer(hb_wire_, hb_len_));
      }
    }
    // wait for the router to deliver the batch to every UAS
    const size_t expected = static_cast<size_t>(batch) * fcu_count_ * uas_count_;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (delivered() < before + expected && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::yield();
    }
    total_delivered += delivered() - before;
  }

  state.counters["delivered"] = benchmark::Counter(
    static_cast<double>(total_delivered));
  state.counters["delivery_pct"] = benchmark::Counter(
    100.0 * static_cast<double>(total_delivered) /
    (static_cast<double>(state.iterations()) * batch * fcu_count_ * uas_count_));
  state.SetItemsProcessed(static_cast<int64_t>(total_delivered));
}

BENCHMARK_REGISTER_F(BenchmarkRouter, sustained)
->Arg(20)      // batch per FCU per iteration
->Unit(benchmark::kMillisecond)
->MinTime(5);  // ~5 s timed portion per run

}  // namespace router
}  // namespace mavros

BENCHMARK_MAIN();
