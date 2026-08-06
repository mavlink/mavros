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

#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "mavconn/interface.hpp"

using mavconn::Framing;
using mavconn::MAVConnInterface;
using mavlink_message_t = ::mavlink::mavlink_message_t;

namespace mavconn
{

class MavconnUdpBenchmark : public benchmark::Fixture
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

BENCHMARK_DEFINE_F(MavconnUdpBenchmark, udp_throughput)(benchmark::State & state)
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

BENCHMARK_REGISTER_F(MavconnUdpBenchmark, udp_throughput)
->Arg(100)
->Unit(benchmark::kMillisecond)
->MinTime(5);

class MavconnTcpBenchmark : public benchmark::Fixture
{
public:
  void SetUp(const benchmark::State &) override
  {
    received_ = 0;
    rx_ = MAVConnInterface::open_url(
      "tcp-l://127.0.0.1:14570", 1, 1,
      [this](const mavlink_message_t *, Framing) {
        received_.fetch_add(1, std::memory_order_relaxed);
      });
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    tx_ = MAVConnInterface::open_url("tcp://127.0.0.1:14570", 2, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    init_hb();
  }

  void TearDown(const benchmark::State &) override
  {
    if (tx_) {tx_->close(); tx_.reset();}
    if (rx_) {rx_->close(); rx_.reset();}
  }

  void init_hb()
  {
    std::memset(&hb_, 0, sizeof(hb_));
    hb_.magic = MAVLINK_STX;
    hb_.msgid = 0;
    hb_.sysid = 1;
    hb_.compid = 1;
    hb_.len = 9;
  }

  MAVConnInterface::Ptr tx_;
  MAVConnInterface::Ptr rx_;
  std::atomic<size_t> received_{0};
  mavlink_message_t hb_;
};

BENCHMARK_DEFINE_F(MavconnTcpBenchmark, tcp_throughput)(benchmark::State & state)
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

BENCHMARK_REGISTER_F(MavconnTcpBenchmark, tcp_throughput)
->Arg(100)
->Unit(benchmark::kMillisecond)
->MinTime(5);

namespace
{

// Spawn `socat PTY,link=P1 PTY,link=P2`, which creates two linked ptys with
// stable symlink paths P1/P2 usable as serial device paths. Returns the pid
// (or -1 on failure). The child dies with the parent (PDEATHSIG).
pid_t spawn_socat_pair(std::string & pty1, std::string & pty2)
{
  pty1 = "/tmp/mavconn_rx_" + std::to_string(::getpid());
  pty2 = "/tmp/mavconn_tx_" + std::to_string(::getpid());
  ::unlink(pty1.c_str());
  ::unlink(pty2.c_str());

  const pid_t pid = ::fork();
  if (pid == 0) {
    ::prctl(PR_SET_PDEATHSIG, SIGTERM);
    const std::string a = "PTY,raw,echo=0,link=" + pty1;
    const std::string b = "PTY,raw,echo=0,link=" + pty2;
    ::execlp("socat", "socat", a.c_str(), b.c_str(), nullptr);
    ::_exit(127);
  }
  if (pid < 0) {
    return -1;
  }

  // wait for both symlinks to appear
  for (int i = 0; i < 50; i++) {
    if (::access(pty1.c_str(), F_OK) == 0 && ::access(pty2.c_str(), F_OK) == 0) {
      return pid;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return -1;
}

}  // namespace

class MavconnSerialBenchmark : public benchmark::Fixture
{
public:
  void SetUp(const benchmark::State &) override
  {
    received_ = 0;
    std::string p1, p2;
    socat_pid_ = spawn_socat_pair(p1, p2);
    if (socat_pid_ < 0) {
      throw std::runtime_error("socat pty pair not available");
    }

    rx_ = MAVConnInterface::open_url(
      std::string("serial://") + p1 + ":57600", 1, 1,
      [this](const mavlink_message_t *, Framing) {
        received_.fetch_add(1, std::memory_order_relaxed);
      });
    tx_ = MAVConnInterface::open_url(std::string("serial://") + p2 + ":57600", 2, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::memset(&hb_, 0, sizeof(hb_));
    hb_.magic = MAVLINK_STX;
    hb_.msgid = 0;
    hb_.sysid = 1;
    hb_.compid = 1;
    hb_.len = 9;
  }

  void TearDown(const benchmark::State &) override
  {
    if (tx_) {tx_->close(); tx_.reset();}
    if (rx_) {rx_->close(); rx_.reset();}
    if (socat_pid_ > 0) {
      ::kill(socat_pid_, SIGTERM);
      ::waitpid(socat_pid_, nullptr, 0);
    }
    ::unlink(("/tmp/mavconn_rx_" + std::to_string(::getpid())).c_str());
    ::unlink(("/tmp/mavconn_tx_" + std::to_string(::getpid())).c_str());
  }

  MAVConnInterface::Ptr tx_;
  MAVConnInterface::Ptr rx_;
  std::atomic<size_t> received_{0};
  mavlink_message_t hb_;
  pid_t socat_pid_{-1};
};

BENCHMARK_DEFINE_F(MavconnSerialBenchmark, serial_throughput)(benchmark::State & state)
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

BENCHMARK_REGISTER_F(MavconnSerialBenchmark, serial_throughput)
->Arg(100)
->Unit(benchmark::kMillisecond)
->MinTime(5);

}  // namespace mavconn

BENCHMARK_MAIN();
