//
// libmavconn
// Copyright 2026, mavros contributors. Licensed as the rest of the package.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * @brief Minimal C-ABI shim over MAVConnUDP for ctypes-based e2e tests.
 *
 * Test-only glue (built under BUILD_TESTING). Not installed, not a library.
 * It exposes just enough to drive a UDP link and capture decoded messages so
 * a Python (pytest + pymavlink) test can compare byte streams against the
 * reference implementation.
 */

#include <array>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "mavconn/udp.hpp"

namespace mavlink
{
namespace common
{
using namespace mavlink::minimal;  // NOLINT
namespace msg
{
using namespace mavlink::minimal::msg;  // NOLINT
}
}  // namespace common
}  // namespace mavlink

namespace
{

using mavlink_message_t = mavlink::mavlink_message_t;

// Mirror of the C struct handed to Python. Fixed-size, POD, ctypes-friendly.
struct RxMsg
{
  uint32_t msgid;
  uint8_t sysid;
  uint8_t compid;
  uint8_t seq;
  uint8_t len;
  uint8_t framing;      // mavconn::Framing value
  uint8_t payload[255];
};

struct Shim
{
  explicit Shim(std::shared_ptr<mavconn::MAVConnUDP> c)
  : conn(std::move(c))
  {}

  std::shared_ptr<mavconn::MAVConnUDP> conn;
  std::mutex mutex;
  std::condition_variable cond;
  std::deque<RxMsg> rx_queue;
};

inline Shim * as_shim(void * handle)
{
  return static_cast<Shim *>(handle);
}

}  // namespace

extern "C"
{
void * mavconn_udp_create(
  uint8_t sysid, uint8_t compid,
  uint16_t bind_port, const char * remote_host, uint16_t remote_port)
{
  auto conn = std::make_shared<mavconn::MAVConnUDP>(
    sysid, compid,
    "0.0.0.0", bind_port,
    std::string(remote_host ? remote_host : ""), remote_port);

  auto shim = std::make_unique<Shim>(conn);
  Shim * shim_ptr = shim.get();

  conn->connect(
    [shim_ptr](const mavlink_message_t * msg, const mavconn::Framing framing) {
      RxMsg out = {};
      out.msgid = msg->msgid;
      out.sysid = msg->sysid;
      out.compid = msg->compid;
      out.seq = msg->seq;
      out.len = msg->len;
      out.framing = static_cast<uint8_t>(framing);
      const auto copy_len = std::min<size_t>(msg->len, sizeof(out.payload));
      std::memcpy(out.payload, _MAV_PAYLOAD(msg), copy_len);

      std::lock_guard<std::mutex> lock(shim_ptr->mutex);
      shim_ptr->rx_queue.push_back(out);
      shim_ptr->cond.notify_all();
    });

  return shim.release();
}

int mavconn_poll_rx(void * handle, RxMsg * out, int timeout_ms)
{
  auto * shim = as_shim(handle);
  if (!shim || !out) {
    return -1;
  }

  std::unique_lock<std::mutex> lock(shim->mutex);
  const bool got = shim->cond.wait_for(
    lock, std::chrono::milliseconds(timeout_ms),
    [&]() {return !shim->rx_queue.empty();});
  if (!got) {
    return 0;      // timeout
  }

  *out = shim->rx_queue.front();
  shim->rx_queue.pop_front();
  return 1;
}

int mavconn_send_heartbeat(
  void * handle,
  uint8_t type, uint8_t autopilot, uint8_t base_mode,
  uint32_t custom_mode, uint8_t system_status)
{
  auto * shim = as_shim(handle);
  if (!shim) {
    return -1;
  }

  mavlink::common::msg::HEARTBEAT hb = {};
  hb.type = type;
  hb.autopilot = autopilot;
  hb.base_mode = base_mode;
  hb.custom_mode = custom_mode;
  hb.system_status = system_status;
  hb.mavlink_version = 3;  // matches pymavlink default

  try {
    // base-class convenience overload uses the object's own compid
    static_cast<mavconn::MAVConnInterface &>(*shim->conn).send_message(hb);
  } catch (const std::length_error &) {
    return -2;
  }
  return 0;
}

int mavconn_send_raw_bytes(void * handle, const uint8_t * bytes, uint32_t length)
{
  auto * shim = as_shim(handle);
  if (!shim || !bytes) {
    return -1;
  }

  try {
    shim->conn->send_bytes(bytes, length);
  } catch (const std::length_error &) {
    return -2;
  }
  return 0;
}

int mavconn_setup_signing(
  void * handle,
  const uint8_t * secret_key,   // 32 bytes
  uint8_t sign_outgoing, uint8_t link_id, uint64_t initial_timestamp)
{
  auto * shim = as_shim(handle);
  if (!shim || !secret_key) {
    return -1;
  }

  std::array<uint8_t, 32> key = {};
  std::memcpy(key.data(), secret_key, key.size());
  shim->conn->setup_signing(key, sign_outgoing != 0, link_id, initial_timestamp);
  return 0;
}

void mavconn_destroy(void * handle)
{
  auto * shim = as_shim(handle);
  if (!shim) {
    return;
  }
  shim->conn->close();
  delete shim;
}
}  // extern "C"
