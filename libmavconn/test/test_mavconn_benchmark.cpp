//
// libmavconn
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * @brief End-to-end mavconn transport pair benchmark.
 *
 * A sender MAVConn and a receiver MAVConn form a pair over a real transport
 * (udp:// loopback); the benchmark measures sustained message throughput
 * through the full send path (enqueue + async send) and receive path
 * (async recv + parse_buffer + callback). Use this to detect regressions in
 * the mavconn hot path.
 */

#include <benchmark/benchmark.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

#include "mavconn/interface.hpp"

using mavconn::Framing;
using mavconn::MAVConnInterface;
using mavlink_message_t = ::mavlink::mavlink_message_t;

namespace mavconn
{

class MavconnPairBenchmark : public benchmark::Fixture
{
public:
  void SetUp(const benchmark::State &) override
  {
    received_ = 0;
    rx_ = MAVConnInterface::open_url(
      "udp://127.0.0.1:14566@127.0.0.1:14555",
      1, 1,
      [this](const mavlink_message_t *, Framing) {
        received_.fetch_add(1, std::memory_order_relaxed);
      });
    tx_ = MAVConnInterface::open_url("udp://127.0.0.1:14555@127.0.0.1:14566");

    std::memset(&hb_, 0, sizeof(hb_));
    hb_.magic = MAVLINK_STX;
    hb_.msgid = 0;   // HEARTBEAT
    hb_.sysid = 1;
    hb_.compid = 1;
    hb_.len = 9;
  }

  void TearDown(const benchmark::State &) override
  {
    if (tx_) {tx_->close(); tx_.reset();}
    if (rx_) {rx_->close(); rx_.reset();}
  }

  MAVConnInterface::Ptr tx_;
  MAVConnInterface::Ptr rx_;
  std::atomic<size_t> received_{0};
  mavlink_message_t hb_;
};

BENCHMARK_DEFINE_F(MavconnPairBenchmark, udp_throughput)(benchmark::State & state)
{
  const size_t batch = static_cast<size_t>(state.range(0));
  size_t total = 0;
  for (auto _ : state) {
    received_ = 0;
    for (size_t i = 0; i < batch; i++) {
      tx_->send_message_ignore_drop(&hb_);
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received_.load(std::memory_order_relaxed) < batch &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::yield();
    }
    total += received_.load(std::memory_order_relaxed);
  }
  state.SetItemsProcessed(static_cast<int64_t>(total));
}

BENCHMARK_REGISTER_F(MavconnPairBenchmark, udp_throughput)
->Arg(100)
->Unit(benchmark::kMillisecond)
->MinTime(5);

}  // namespace mavconn

BENCHMARK_MAIN();
