// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * End-to-end router test with real UDP endpoints.
 *
 * The routing rules are unit-tested with mocks in test_router.cpp; this test
 * exercises them over the real MAVConn UDP send/receive path, conversion to
 * mavros_msgs::msg::Mavlink and intra-process delivery to ROS (UAS) endpoints:
 *   - gcs_broadcast_reaches_all_fcus: one GCS talks to all FCUs;
 *   - fcu_broadcast_reaches_gcs: every FCU talks to the GCS;
 *   - uas_targeted_routing: a PING targeted at a UAS is delivered only to
 *     that UAS, plus the broadcast, like a real UAS expects;
 *   - uas_broadcast_reaches_gcs_and_fcus: UAS traffic is routed back to the
 *     GCS and every FCU.
 *
 * Sustained throughput is measured separately by the Google Benchmark
 * mavros-router-benchmark (see test_router_benchmark.cpp).
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <future>
#include <memory>
#include <set>
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

// Base UDP port; override with MAVROS_TEST_UDP_BASE if it collides.
static int udp_base()
{
  if (const char * env = std::getenv("MAVROS_TEST_UDP_BASE")) {
    return std::atoi(env);
  }
  return 15500;
}

// Port layout: FCU bind/remote, GCS bind/remote
static int fcu_bind(int f) {return udp_base() + f;}
static int fcu_remote(int f) {return udp_base() + 100 + f;}
static int gcs_bind() {return udp_base() + 200;}
static int gcs_remote() {return udp_base() + 300;}

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

// Extract the source system id from a MAVLink frame (v1 or v2).
static uint8_t frame_sysid(const uint8_t * wire)
{
  if (wire[0] == 0xFD) {  // MAVLink v2
    return wire[5];
  }
  if (wire[0] == 0xFE) {  // MAVLink v1
    return wire[3];
  }
  return 0;
}

class E2ERouter : public ::testing::Test
{
public:
  E2ERouter()
  {
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  }

  ~E2ERouter()
  {
    // Stop the executor first so nothing can run during teardown.
    stop_spinner();
    // Break the Endpoint <-> Router reference cycle (router->endpoints and
    // router->remote_index hold endpoints, and each endpoint->parent holds the
    // router), otherwise the Router node (a DDS participant) is leaked.
    for (auto & router : routers_) {
      router->endpoints.clear();
      router->remote_index.clear();
    }
    routers_.clear();
  }

  // Creates a Router with fcu_count UDP FCU endpoints, gcs_count GCS endpoints
  // and uas_count UAS (ROSEndpoint) endpoints, configured via parameters.
  Router::SharedPtr make_router(int fcu_count, int gcs_count, int uas_count)
  {
    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);
    auto router = std::make_shared<Router>(opts, "e2e_router");

    std::vector<std::string> fcu_urls, gcs_urls, uas_urls;
    for (int f = 0; f < fcu_count; f++) {
      fcu_urls.push_back(
        format("udp://127.0.0.1:%d@127.0.0.1:%d", fcu_bind(f), fcu_remote(f)));
    }
    for (int g = 0; g < gcs_count; g++) {
      (void)g;  // single shared GCS port
      gcs_urls.push_back(format("udp://127.0.0.1:%d@127.0.0.1:%d", gcs_bind(), gcs_remote()));
    }
    for (int u = 0; u < uas_count; u++) {
      uas_urls.push_back(format("/bench_uas%d", u + 1));
    }

    router->set_parameters({
          rclcpp::Parameter("fcu_urls", fcu_urls),
          rclcpp::Parameter("gcs_urls", gcs_urls),
          rclcpp::Parameter("uas_urls", uas_urls),
    });

    exec_->add_node(router);
    routers_.push_back(router);
    return router;
  }

  void start_spinner(const Router::SharedPtr & router, size_t expected_endpoints)
  {
    if (spinning_) {
      return;
    }
    spinning_ = true;
    exit_promise_ = std::make_shared<std::promise<void>>();
    exit_future_ = exit_promise_->get_future().share();
    spin_thread_ = std::thread([this]() {
          exec_->spin_until_future_complete(exit_future_, 100ms);
    });
    // wait for the startup_delay_timer to create all endpoints
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (get_endpoints(router).size() < expected_endpoints &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(10ms);
    }
  }

  void stop_spinner()
  {
    if (!spinning_) {
      return;
    }
    exit_promise_->set_value();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    spinning_ = false;
  }

  // UDP socket bound to the router's remote port, sending to the router's bind port.
  asio::ip::udp::socket make_sender(int local_port, int remote_port)
  {
    asio::ip::udp::socket s(io_);
    s.open(asio::ip::udp::v4());
    s.set_option(asio::ip::udp::socket::reuse_address(true));
    s.bind(
      asio::ip::udp::endpoint(
        asio::ip::make_address("127.0.0.1"), local_port));
    s.connect(
      asio::ip::udp::endpoint(
        asio::ip::make_address("127.0.0.1"), remote_port));
    return s;
  }

  // Drains datagrams queued on a socket, incrementing count per message and
  // optionally collecting the distinct source system ids seen.
  void drain(
    asio::ip::udp::socket & s, size_t & count,
    std::set<uint8_t> * sysids = nullptr)
  {
    while (s.available() > 0) {
      uint8_t buf[512];
      s.receive(asio::buffer(buf));
      count++;
      if (sysids) {
        sysids->insert(frame_sysid(buf));
      }
    }
  }

  std::shared_ptr<std::atomic<size_t>> subscribe_uas(
    const Router::SharedPtr & router, int u,
    std::vector<rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr> & subs)
  {
    auto counter = std::make_shared<std::atomic<size_t>>(0);
    subs.push_back(
      router->create_subscription<mavros_msgs::msg::Mavlink>(
        format("/bench_uas%d/mavlink_source", u + 1), rclcpp::QoS(1000).best_effort(),
        [counter](const mavros_msgs::msg::Mavlink::SharedPtr) {(*counter)++;}));
    return counter;
  }

  // Publishes a message "from" a UAS so the router learns the UAS's address.
  // The publisher is kept in pubs so the test can send more messages later.
  void teach_uas_address(
    const Router::SharedPtr & router, int u, uint8_t sysid,
    std::vector<rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr> & pubs)
  {
    pubs.push_back(
      router->create_publisher<mavros_msgs::msg::Mavlink>(
        format("/bench_uas%d/mavlink_sink", u + 1), rclcpp::QoS(1000).best_effort()));

    mavros_msgs::msg::Mavlink rmsg{};
    rmsg.header.stamp.sec = 1;
    rmsg.sysid = sysid;
    rmsg.compid = 1;
    rmsg.msgid = 0;  // HEARTBEAT
    pubs.back()->publish(rmsg);
  }

  // Waits until every UAS endpoint has learned the taught address
  // (sysid << 8 | compid, stored in remote_addrs on a mavlink_sink message).
  void wait_learned(const Router::SharedPtr & router, const std::set<addr_t> & taught)
  {
    auto learned_state = [&]() {
        size_t n = 0;
        std::set<addr_t> addrs;
        for (const auto & [id, ep] : get_endpoints(router)) {
          if (ep->link_type != Endpoint::Type::uas) {
            continue;
          }
          for (addr_t a : taught) {
            if (ep->remote_addrs.find(a) != ep->remote_addrs.end()) {
              addrs.insert(a);
              n++;
              break;
            }
          }
        }
        return std::pair<size_t, size_t>{n, addrs.size()};
      };
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (learned_state().first < taught.size() &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(10ms);
    }
  }

protected:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread spin_thread_;
  std::shared_ptr<std::promise<void>> exit_promise_;
  std::shared_future<void> exit_future_;
  bool spinning_{false};
  asio::io_context io_;
  std::vector<Router::SharedPtr> routers_;

  // Helper so the gtest-derived classes can access the Router's endpoints.
  std::unordered_map<id_t, Endpoint::SharedPtr> & get_endpoints(
    const Router::SharedPtr & router) const
  {
    return router->endpoints;
  }
};

// One GCS talks to all FCUs: a GCS broadcast must be delivered to every FCU
// endpoint, so a single GCS can command all autopilots.
TEST_F(E2ERouter, gcs_broadcast_reaches_all_fcus)
{
  constexpr int FCU_COUNT = 3;
  constexpr int UAS_COUNT = 4;
  constexpr int BROADCASTS = 10;

  auto router = make_router(FCU_COUNT, 1, UAS_COUNT);
  start_spinner(router, FCU_COUNT + 1 + UAS_COUNT);

  std::vector<asio::ip::udp::socket> fcus;
  for (int f = 0; f < FCU_COUNT; f++) {
    fcus.push_back(make_sender(fcu_remote(f), fcu_bind(f)));
  }
  auto gcs = make_sender(gcs_remote(), gcs_bind());

  mavlink::minimal::msg::HEARTBEAT hb{};
  hb.type = 2;
  hb.autopilot = 3;
  hb.base_mode = 80;
  hb.system_status = 2;
  uint8_t wire[512];
  auto len = build_frame(hb, 255, 190, wire);
  for (int i = 0; i < BROADCASTS; i++) {
    gcs.send(asio::buffer(wire, len));
  }

  std::vector<size_t> fcu_rx(FCU_COUNT, 0);
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  size_t total = 0;
  while (total < FCU_COUNT * BROADCASTS && std::chrono::steady_clock::now() < deadline) {
    total = 0;
    for (int f = 0; f < FCU_COUNT; f++) {
      drain(fcus[f], fcu_rx[f]);
      total += fcu_rx[f];
    }
    if (total < FCU_COUNT * BROADCASTS) {
      std::this_thread::sleep_for(10ms);
    }
  }

  for (int f = 0; f < FCU_COUNT; f++) {
    EXPECT_EQ(fcu_rx[f], BROADCASTS) << "FCU" << f + 1;
  }
  stop_spinner();
}

// Every FCU talks to the GCS: broadcasts from each FCU must all be delivered
// to the single GCS, so a ground station sees all autopilots.
TEST_F(E2ERouter, fcu_broadcast_reaches_gcs)
{
  constexpr int FCU_COUNT = 3;
  constexpr int UAS_COUNT = 4;
  constexpr int PER_FCU = 10;

  auto router = make_router(FCU_COUNT, 1, UAS_COUNT);
  start_spinner(router, FCU_COUNT + 1 + UAS_COUNT);

  std::vector<asio::ip::udp::socket> fcus;
  for (int f = 0; f < FCU_COUNT; f++) {
    fcus.push_back(make_sender(fcu_remote(f), fcu_bind(f)));
  }
  auto gcs = make_sender(gcs_remote(), gcs_bind());

  mavlink::minimal::msg::HEARTBEAT hb{};
  hb.type = 2;
  hb.autopilot = 3;
  hb.base_mode = 80;
  hb.system_status = 2;
  for (int f = 0; f < FCU_COUNT; f++) {
    uint8_t wire[512];
    auto len = build_frame(hb, f + 1, 1, wire);  // distinct sysid per FCU
    for (int i = 0; i < PER_FCU; i++) {
      fcus[f].send(asio::buffer(wire, len));
    }
  }

  size_t gcs_rx = 0;
  std::set<uint8_t> sysids;
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (gcs_rx < FCU_COUNT * PER_FCU && std::chrono::steady_clock::now() < deadline) {
    drain(gcs, gcs_rx, &sysids);
    if (gcs_rx < FCU_COUNT * PER_FCU) {
      std::this_thread::sleep_for(10ms);
    }
  }

  EXPECT_EQ(gcs_rx, FCU_COUNT * PER_FCU);
  EXPECT_EQ(sysids, (std::set<uint8_t>{1, 2, 3})) << "GCS should hear every FCU";
  stop_spinner();
}

// Targeted routing: each UAS learns its address, then an FCU sends a targeted
// PING to each UAS plus one broadcast. Each UAS must receive exactly its own
// PING and the broadcast, and none of the others'.
TEST_F(E2ERouter, uas_targeted_routing)
{
  constexpr int UAS_COUNT = 4;
  auto router = make_router(1, 0, UAS_COUNT);
  start_spinner(router, 1 + UAS_COUNT);

  std::vector<rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr> subs;
  std::vector<std::shared_ptr<std::atomic<size_t>>> counters;
  for (int u = 0; u < UAS_COUNT; u++) {
    counters.push_back(subscribe_uas(router, u, subs));
  }

  std::vector<rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr> pubs;
  std::set<addr_t> taught;
  for (int u = 0; u < UAS_COUNT; u++) {
    teach_uas_address(router, u, (u + 1) * 10, pubs);
    taught.insert((static_cast<addr_t>((u + 1) * 10) << 8) | 1);
  }

  wait_learned(router, taught);

  for (int u = 0; u < UAS_COUNT; u++) {
    counters[u]->store(0);
  }

  auto fcu = make_sender(fcu_remote(0), fcu_bind(0));
  for (int u = 0; u < UAS_COUNT; u++) {
    mavlink::common::msg::PING ping{};
    ping.time_usec = 1000 + u;
    ping.seq = static_cast<uint32_t>(u);
    ping.target_system = static_cast<uint8_t>((u + 1) * 10);
    ping.target_component = 1;
    uint8_t wire[512];
    auto len = build_frame(ping, 1, 1, wire);
    fcu.send(asio::buffer(wire, len));
  }
  mavlink::common::msg::PING bping{};
  bping.time_usec = 9999;
  bping.seq = 99;
  bping.target_system = 0;
  bping.target_component = 0;
  uint8_t bwire[512];
  auto blen = build_frame(bping, 1, 1, bwire);
  fcu.send(asio::buffer(bwire, blen));

  // Wait until each UAS received its targeted PING + the broadcast.
  const auto rcvd_deadline = std::chrono::steady_clock::now() + 10s;
  bool all_two = false;
  while (!all_two && std::chrono::steady_clock::now() < rcvd_deadline) {
    all_two = true;
    for (int u = 0; u < UAS_COUNT; u++) {
      if (counters[u]->load() < 2U) {
        all_two = false;
        break;
      }
    }
    if (!all_two) {
      std::this_thread::sleep_for(10ms);
    }
  }
  for (int u = 0; u < UAS_COUNT; u++) {
    EXPECT_EQ(counters[u]->load(), 2U) << "UAS" << u + 1;
  }

  stop_spinner();
}

// UAS traffic is routed back to the wire: a broadcast from each UAS must reach
// the GCS and every FCU endpoint.
TEST_F(E2ERouter, uas_broadcast_reaches_gcs_and_fcus)
{
  constexpr int FCU_COUNT = 3;
  constexpr int UAS_COUNT = 4;

  auto router = make_router(FCU_COUNT, 1, UAS_COUNT);
  start_spinner(router, FCU_COUNT + 1 + UAS_COUNT);

  std::vector<asio::ip::udp::socket> fcus;
  for (int f = 0; f < FCU_COUNT; f++) {
    fcus.push_back(make_sender(fcu_remote(f), fcu_bind(f)));
  }
  auto gcs = make_sender(gcs_remote(), gcs_bind());

  std::vector<rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr> subs;
  std::vector<std::shared_ptr<std::atomic<size_t>>> counters;
  std::vector<rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr> pubs;
  for (int u = 0; u < UAS_COUNT; u++) {
    counters.push_back(subscribe_uas(router, u, subs));
    pubs.push_back(
      router->create_publisher<mavros_msgs::msg::Mavlink>(
        format("/bench_uas%d/mavlink_sink", u + 1), rclcpp::QoS(1000).best_effort()));
    mavros_msgs::msg::Mavlink rmsg{};
    rmsg.header.stamp.sec = 2;
    rmsg.sysid = static_cast<uint8_t>((u + 1) * 10);
    rmsg.compid = 1;
    rmsg.msgid = 0;  // HEARTBEAT
    pubs.back()->publish(rmsg);
  }

  size_t gcs_rx = 0;
  std::vector<size_t> fcu_rx(FCU_COUNT, 0);
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  size_t total = 0;
  while (total < UAS_COUNT * (FCU_COUNT + 1) && std::chrono::steady_clock::now() < deadline) {
    drain(gcs, gcs_rx);
    total = gcs_rx;
    for (int f = 0; f < FCU_COUNT; f++) {
      drain(fcus[f], fcu_rx[f]);
      total += fcu_rx[f];
    }
    if (total < UAS_COUNT * (FCU_COUNT + 1)) {
      std::this_thread::sleep_for(10ms);
    }
  }

  EXPECT_EQ(gcs_rx, UAS_COUNT) << "GCS should hear every UAS";
  for (int f = 0; f < FCU_COUNT; f++) {
    EXPECT_EQ(fcu_rx[f], UAS_COUNT) << "FCU" << f + 1;
  }
  stop_spinner();
}

}  // namespace router
}  // namespace mavros

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
