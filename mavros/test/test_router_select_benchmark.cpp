// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Micro-benchmark of the router's route_message() selection logic in
 * isolation: mock endpoints (no DDS publish, no real I/O) so the measured cost
 * is only the routing decision. Used to isolate router work from the
 * libmavconn send path and the ROS/dds publish path.
 */

#include <benchmark/benchmark.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "mavros/mavros_router.hpp"
#include "mavros_msgs/msg/mavlink.hpp"

using mavros::router::Endpoint;
using mavros::router::Framing;
using mavros::router::Router;
using mavros::router::addr_t;
using mavros::router::id_t;
using mavlink_message_t = mavlink::mavlink_message_t;

namespace mavros
{
namespace router
{

namespace
{

class SelectEndpoint : public Endpoint
{
public:
  using SharedPtr = std::shared_ptr<SelectEndpoint>;

  std::atomic<size_t> sends{0};

  bool is_open() override
  {
    return true;
  }

  std::pair<bool, std::string> open() override
  {
    return {true, ""};
  }

  void close() override {}

  void send_message(
    const mavlink_message_t *,
    const Framing,
    const std::string &) override
  {
    sends.fetch_add(1, std::memory_order_relaxed);
  }

  void diag_run(diagnostic_updater::DiagnosticStatusWrapper &) override {}
};

}  // namespace

class RouterSelectBenchmark : public benchmark::Fixture
{
public:
  void SetUp(const benchmark::State &) override
  {
    rclcpp::init(0, nullptr);
    router_ = std::make_shared<Router>("bench_select");

    auto make_ep = [this](id_t id, Endpoint::Type type, addr_t addr) {
        auto ep = std::make_shared<SelectEndpoint>();
        ep->parent = router_;
        ep->id = id;
        ep->link_type = type;
        ep->remote_addrs = {0x0000, addr};
        router_->endpoints[id] = ep;
        return ep;
      };

    fcu_ = make_ep(1000, Endpoint::Type::fcu, 0x0100);
    for (int i = 0; i < 3; i++) {
      gcs_.push_back(make_ep(1001 + i, Endpoint::Type::gcs, 0x0200));
    }

    std::memset(&hb_, 0, sizeof(hb_));
    hb_.magic = MAVLINK_STX;
    hb_.msgid = 0;   // HEARTBEAT, no target -> broadcast
    hb_.sysid = 1;
    hb_.compid = 1;
    hb_.len = 9;

    // warm up: trigger the lazy index rebuild (any branch) before timing
    router_->route_message(fcu_, &hb_, Framing::ok);
  }

  void TearDown(const benchmark::State &) override
  {
    router_->endpoints.clear();
    router_->remote_index.clear();
    fcu_.reset();
    gcs_.clear();
    router_.reset();
    rclcpp::shutdown();
  }

  Router::SharedPtr router_;
  SelectEndpoint::SharedPtr fcu_;
  std::vector<SelectEndpoint::SharedPtr> gcs_;
  mavlink_message_t hb_;
};

BENCHMARK_DEFINE_F(RouterSelectBenchmark, route_broadcast)(benchmark::State & state)
{
  for (auto _ : state) {
    router_->route_message(fcu_, &hb_, Framing::ok);
  }
  state.SetItemsProcessed(state.iterations());
}

BENCHMARK_REGISTER_F(RouterSelectBenchmark, route_broadcast)
->Unit(benchmark::kMillisecond)
->MinTime(5);

BENCHMARK_MAIN();

}   // namespace router
}   // namespace mavros
