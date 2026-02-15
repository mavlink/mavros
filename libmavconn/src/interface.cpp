//
// libmavconn
// Copyright 2013,2014,2015,2016,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief MAVConn class interface
 * @file interface.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */

#include <cassert>
#include <cstring>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "mavconn/console_bridge_compat.hpp"
#include "mavconn/interface.hpp"
#include "mavconn/msgbuffer.hpp"
#include "mavconn/serial.hpp"
#include "mavconn/tcp.hpp"
#include "mavconn/udp.hpp"

namespace mavconn
{
#define PFX "mavconn: "

using mavlink::mavlink_message_t;
using mavlink::mavlink_status_t;

// static members
std::once_flag MAVConnInterface::init_flag;
std::unordered_map<mavlink::msgid_t,
  const mavlink::mavlink_msg_entry_t *>
MAVConnInterface::message_entries {};
std::atomic<size_t> MAVConnInterface::conn_id_counter {0};

MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id)
: sys_id(system_id),
  comp_id(component_id),
  m_rx_parse_status{},
  m_buffer{},
  m_rx_status{},
  m_tx_status{},
  m_rx_signing{},
  m_tx_signing{},
  m_rx_signing_streams{},
  signing_enabled(false),
  tx_total_bytes(0),
  rx_total_bytes(0),
  last_tx_total_bytes(0),
  last_rx_total_bytes(0),
  last_iostat(steady_clock::now())
{
  conn_id = conn_id_counter.fetch_add(1);
  std::call_once(init_flag, init_msg_entry);
}

mavlink_status_t MAVConnInterface::get_status()
{
  return m_rx_status;
}

MAVConnInterface::IOStat MAVConnInterface::get_iostat()
{
  std::lock_guard<std::mutex> lock(iostat_mutex);
  IOStat stat;

  stat.tx_total_bytes = tx_total_bytes;
  stat.rx_total_bytes = rx_total_bytes;

  auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
  auto d_rx = stat.rx_total_bytes - last_rx_total_bytes;
  last_tx_total_bytes = stat.tx_total_bytes;
  last_rx_total_bytes = stat.rx_total_bytes;

  auto now = steady_clock::now();
  auto dt = now - last_iostat;
  last_iostat = now;

  const float dt_s = std::chrono::duration<float>(dt).count();
  if (dt_s > 0.0f) [[likely]] {  // NOLINT(readability/braces)
    stat.tx_speed = d_tx / dt_s;
    stat.rx_speed = d_rx / dt_s;
  } else {
    stat.tx_speed = 0.0f;
    stat.rx_speed = 0.0f;
  }

  return stat;
}

void MAVConnInterface::iostat_tx_add(size_t bytes)
{
  tx_total_bytes += bytes;
}

void MAVConnInterface::iostat_rx_add(size_t bytes)
{
  rx_total_bytes += bytes;
}

void MAVConnInterface::parse_buffer(
  const char * pfx, uint8_t * buf, const size_t bufsize,
  size_t bytes_received)
{
  mavlink::mavlink_message_t message;

  assert(bufsize >= bytes_received);

  iostat_rx_add(bytes_received);
  for (; bytes_received > 0; bytes_received--) {
    auto c = *buf++;

    auto msg_received = static_cast<Framing>(mavlink::mavlink_frame_char_buffer(
        &m_buffer, &m_rx_parse_status, c,
        &message, &m_rx_status));

    if (msg_received != Framing::incomplete) {
      log_recv(pfx, message, msg_received);

      if (message_received_cb) {
        message_received_cb(&message, msg_received);
      }
    }
  }
}

void MAVConnInterface::log_recv(const char * pfx, mavlink_message_t & msg, Framing framing)
{
  const char * framing_str =
    (framing == Framing::ok) ? "OK" : (framing == Framing::bad_crc) ? "!CRC" :
    (framing == Framing::bad_signature) ? "!SIG" :
    "ERR";

  const char * proto_version_str = (msg.magic == MAVLINK_STX) ? "v2.0" : "v1.0";

  CONSOLE_BRIDGE_logDebug(
    "%s%zu: recv: %s %4s Message-Id: %u [%u bytes] IDs: %u.%u Seq: %u",
    pfx, conn_id,
    proto_version_str,
    framing_str,
    msg.msgid, msg.len, msg.sysid, msg.compid, msg.seq);
}

void MAVConnInterface::log_send(const char * pfx, const mavlink_message_t * msg)
{
  const char * proto_version_str = (msg->magic == MAVLINK_STX) ? "v2.0" : "v1.0";

  CONSOLE_BRIDGE_logDebug(
    "%s%zu: send: %s Message-Id: %u [%u bytes] IDs: %u.%u Seq: %u",
    pfx, conn_id,
    proto_version_str,
    msg->msgid, msg->len, msg->sysid, msg->compid, msg->seq);
}

void MAVConnInterface::log_send_obj(const char * pfx, const mavlink::Message & msg)
{
  CONSOLE_BRIDGE_logDebug("%s%zu: send: %s", pfx, conn_id, msg.to_yaml().c_str());
}

void MAVConnInterface::send_message_ignore_drop(const mavlink::mavlink_message_t * msg)
{
  try {
    send_message(msg);
  } catch (std::length_error & e) {
    CONSOLE_BRIDGE_logError(
      PFX "%zu: DROPPED Message-Id %u [%u bytes] IDs: %u.%u Seq: %u: %s",
      conn_id,
      msg->msgid, msg->len, msg->sysid, msg->compid, msg->seq,
      e.what());
  }
}

void MAVConnInterface::send_message_ignore_drop(const mavlink::Message & msg, uint8_t source_compid)
{
  try {
    send_message(msg, source_compid);
  } catch (std::length_error & e) {
    CONSOLE_BRIDGE_logError(
      PFX "%zu: DROPPED Message %s: %s",
      conn_id,
      msg.get_name().c_str(),
      e.what());
  }
}

void MAVConnInterface::set_protocol_version(Protocol pver)
{
  if (pver == Protocol::V10) {
    m_tx_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    m_rx_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    m_rx_parse_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  } else {
    m_tx_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    m_rx_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    m_rx_parse_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
  }
}

Protocol MAVConnInterface::get_protocol_version()
{
  if (m_tx_status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
    return Protocol::V10;
  } else {
    return Protocol::V20;
  }
}

uint64_t MAVConnInterface::default_signing_timestamp()
{
  constexpr auto mavlink_signing_epoch = std::chrono::sys_days {std::chrono::year(2015) /
      std::chrono::January / 1};
  auto now = std::chrono::system_clock::now();
  if (now < mavlink_signing_epoch) {
    now = mavlink_signing_epoch;
  }

  const auto delta_us = std::chrono::duration_cast<std::chrono::microseconds>(
    now - mavlink_signing_epoch).count();
  return static_cast<uint64_t>(delta_us / 10);
}

void MAVConnInterface::apply_signing_config(bool sign_outgoing, uint8_t link_id, uint64_t timestamp)
{
  m_tx_signing.flags = sign_outgoing ? MAVLINK_SIGNING_FLAG_SIGN_OUTGOING : 0;
  m_tx_signing.link_id = link_id;
  m_tx_signing.timestamp = timestamp;
  m_tx_signing.last_status = mavlink::MAVLINK_SIGNING_STATUS_NONE;

  m_rx_signing.flags = sign_outgoing ? MAVLINK_SIGNING_FLAG_SIGN_OUTGOING : 0;
  m_rx_signing.link_id = link_id;
  m_rx_signing.timestamp = timestamp;
  m_rx_signing.last_status = mavlink::MAVLINK_SIGNING_STATUS_NONE;

  m_rx_signing_streams = {};

  m_tx_status.signing = &m_tx_signing;
  m_rx_parse_status.signing = &m_rx_signing;
  m_rx_parse_status.signing_streams = &m_rx_signing_streams;
  m_rx_status.signing = &m_rx_signing;
  m_rx_status.signing_streams = &m_rx_signing_streams;

  signing_enabled = true;
}

void MAVConnInterface::setup_signing(
  const std::array<uint8_t, 32> & secret_key,
  bool sign_outgoing,
  uint8_t link_id,
  std::optional<uint64_t> initial_timestamp)
{
  std::memcpy(m_tx_signing.secret_key, secret_key.data(), secret_key.size());
  std::memcpy(m_rx_signing.secret_key, secret_key.data(), secret_key.size());

  const auto timestamp = initial_timestamp.value_or(default_signing_timestamp());
  apply_signing_config(sign_outgoing, link_id, timestamp);
}

void MAVConnInterface::disable_signing()
{
  m_tx_signing = {};
  m_rx_signing = {};
  m_rx_signing_streams = {};

  m_tx_status.signing = nullptr;
  m_tx_status.signing_streams = nullptr;
  m_rx_parse_status.signing = nullptr;
  m_rx_parse_status.signing_streams = nullptr;
  m_rx_status.signing = nullptr;
  m_rx_status.signing_streams = nullptr;

  signing_enabled = false;
}

void MAVConnInterface::set_accept_unsigned_callback(mavlink::mavlink_accept_unsigned_t cb)
{
  m_tx_signing.accept_unsigned_callback = cb;
  m_rx_signing.accept_unsigned_callback = cb;
}

/**
 * Parse host:port pairs
 */
static void url_parse_host(
  const std::string & host,
  std::string & host_out, int & port_out,
  const std::string & def_host, const int def_port)
{
  auto parse_int_in_range = [](const std::string & value, int min, int max, const char * field) {
      size_t pos = 0;
      int parsed = 0;
      const std::string err = std::string("invalid ") + field + " value: '" + value + "'";
      try {
        parsed = std::stoi(value, &pos);
      } catch (const std::exception &) {
        throw DeviceError("url", err.c_str());
      }

      if (pos != value.size() || parsed < min || parsed > max) {
        throw DeviceError("url", err.c_str());
      }

      return parsed;
    };

  std::string port;

  auto sep_it = std::find(host.begin(), host.end(), ':');
  if (sep_it == host.end()) {
    // host
    if (!host.empty()) {
      host_out = host;
      port_out = def_port;
    } else {
      host_out = def_host;
      port_out = def_port;
    }
    return;
  }

  if (sep_it == host.begin()) {
    // :port
    host_out = def_host;
  } else {
    // host:port
    host_out.assign(host.begin(), sep_it);
  }

  port.assign(sep_it + 1, host.end());
  port_out = parse_int_in_range(port, 1, std::numeric_limits<uint16_t>::max(), "port");
}

/**
 * Parse ?ids=sid,cid
 */
static void url_parse_query(const std::string & query, uint8_t & sysid, uint8_t & compid)
{
  auto parse_uint8_value = [](const std::string & value, const char * field) {
      size_t pos = 0;
      int parsed = 0;
      const std::string err = std::string("invalid ") + field + " value: '" + value + "'";
      try {
        parsed = std::stoi(value, &pos);
      } catch (const std::exception &) {
        throw DeviceError("url", err.c_str());
      }

      if (pos != value.size() || parsed < 0 || parsed > std::numeric_limits<uint8_t>::max()) {
        throw DeviceError("url", err.c_str());
      }

      return static_cast<uint8_t>(parsed);
    };

  const std::string ids_end("ids=");
  std::string sys, comp;

  if (query.empty()) {
    return;
  }

  auto ids_it = std::search(
    query.begin(), query.end(),
    ids_end.begin(), ids_end.end());
  if (ids_it == query.end()) {
    CONSOLE_BRIDGE_logWarn(PFX "URL: unknown query arguments");
    return;
  }

  std::advance(ids_it, ids_end.length());
  auto comma_it = std::find(ids_it, query.end(), ',');
  if (comma_it == query.end()) {
    CONSOLE_BRIDGE_logError(PFX "URL: no comma in ids= query");
    return;
  }

  sys.assign(ids_it, comma_it);
  comp.assign(comma_it + 1, query.end());

  sysid = parse_uint8_value(sys, "system id");
  compid = parse_uint8_value(comp, "component id");

  CONSOLE_BRIDGE_logDebug(PFX "URL: found system/component id = [%u, %u]", sysid, compid);
}

static MAVConnInterface::Ptr url_parse_serial(
  const std::string & path, const std::string & query,
  uint8_t system_id, uint8_t component_id, bool hwflow, asio::io_service * shared_io)
{
  std::string file_path;
  int baudrate;

  // /dev/ttyACM0:57600
  url_parse_host(
    path, file_path, baudrate, MAVConnSerial::DEFAULT_DEVICE,
    MAVConnSerial::DEFAULT_BAUDRATE);
  url_parse_query(query, system_id, component_id);

  return std::make_shared<MAVConnSerial>(
    system_id, component_id,
    file_path, baudrate, hwflow, shared_io);
}

static MAVConnInterface::Ptr url_parse_udp(
  const std::string & hosts, const std::string & query,
  uint8_t system_id, uint8_t component_id, bool is_udpb, bool permanent_broadcast,
  asio::io_service * shared_io)
{
  std::string bind_pair, remote_pair;
  std::string bind_host, remote_host;
  int bind_port, remote_port;

  auto sep_it = std::find(hosts.begin(), hosts.end(), '@');
  if (sep_it == hosts.end()) {
    CONSOLE_BRIDGE_logError(PFX "UDP URL should contain @!");
    throw DeviceError("url", "UDP separator not found");
  }

  bind_pair.assign(hosts.begin(), sep_it);
  remote_pair.assign(sep_it + 1, hosts.end());

  // udp://0.0.0.0:14555@:14550
  url_parse_host(bind_pair, bind_host, bind_port, "0.0.0.0", MAVConnUDP::DEFAULT_BIND_PORT);
  url_parse_host(
    remote_pair, remote_host, remote_port, MAVConnUDP::DEFAULT_REMOTE_HOST,
    MAVConnUDP::DEFAULT_REMOTE_PORT);
  url_parse_query(query, system_id, component_id);

  if (is_udpb) {
    remote_host =
      permanent_broadcast ? MAVConnUDP::PERMANENT_BROADCAST_REMOTE_HOST : MAVConnUDP::
      BROADCAST_REMOTE_HOST;
  }

  return std::make_shared<MAVConnUDP>(
    system_id, component_id,
    bind_host, bind_port,
    remote_host, remote_port, shared_io);
}

static MAVConnInterface::Ptr url_parse_tcp_client(
  const std::string & host, const std::string & query,
  uint8_t system_id, uint8_t component_id, asio::io_service * shared_io)
{
  std::string server_host;
  int server_port;

  // tcp://localhost:5760
  url_parse_host(host, server_host, server_port, "localhost", 5760);
  url_parse_query(query, system_id, component_id);

  return std::make_shared<MAVConnTCPClient>(
    system_id, component_id,
    server_host, server_port, shared_io);
}

static MAVConnInterface::Ptr url_parse_tcp_server(
  const std::string & host, const std::string & query,
  uint8_t system_id, uint8_t component_id, asio::io_service * shared_io)
{
  std::string bind_host;
  int bind_port;

  // tcp-l://0.0.0.0:5760
  url_parse_host(host, bind_host, bind_port, "0.0.0.0", 5760);
  url_parse_query(query, system_id, component_id);

  return std::make_shared<MAVConnTCPServer>(
    system_id, component_id,
    bind_host, bind_port, shared_io);
}

MAVConnInterface::Ptr MAVConnInterface::open_url_no_connect(
  std::string url,
  uint8_t system_id,
  uint8_t component_id,
  asio::io_service * shared_io)
{
  /* Based on code found here:
   * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
   */

  const std::string proto_end("://");
  std::string proto;
  std::string host;
  std::string path;
  std::string query;

  auto proto_it = std::search(
    url.begin(), url.end(),
    proto_end.begin(), proto_end.end());
  if (proto_it == url.end()) {
    // looks like file path
    CONSOLE_BRIDGE_logDebug(PFX "URL: %s: looks like file path", url.c_str());
    return url_parse_serial(url, "", system_id, component_id, false, shared_io);
  }

  // copy protocol
  proto.reserve(std::distance(url.begin(), proto_it));
  std::transform(
    url.begin(), proto_it,
    std::back_inserter(proto),
    std::ref(tolower));

  // copy host
  std::advance(proto_it, proto_end.length());
  auto path_it = std::find(proto_it, url.end(), '/');
  std::transform(
    proto_it, path_it,
    std::back_inserter(host),
    std::ref(tolower));

  // copy path, and query if exists
  auto query_it = std::find(path_it, url.end(), '?');
  path.assign(path_it, query_it);
  if (query_it != url.end()) {
    ++query_it;
  }
  query.assign(query_it, url.end());

  CONSOLE_BRIDGE_logDebug(
    PFX "URL: %s: proto: %s, host: %s, path: %s, query: %s",
    url.c_str(), proto.c_str(), host.c_str(),
    path.c_str(), query.c_str());

  MAVConnInterface::Ptr interface_ptr;

  if (proto == "udp") {
    interface_ptr = url_parse_udp(host, query, system_id, component_id, false, false, shared_io);
  } else if (proto == "udp-b") {
    interface_ptr = url_parse_udp(host, query, system_id, component_id, true, false, shared_io);
  } else if (proto == "udp-pb") {
    interface_ptr = url_parse_udp(host, query, system_id, component_id, true, true, shared_io);
  } else if (proto == "tcp") {
    interface_ptr = url_parse_tcp_client(host, query, system_id, component_id, shared_io);
  } else if (proto == "tcp-l") {
    interface_ptr = url_parse_tcp_server(host, query, system_id, component_id, shared_io);
  } else if (proto == "serial") {
    interface_ptr = url_parse_serial(path, query, system_id, component_id, false, shared_io);
  } else if (proto == "serial-hwfc") {
    interface_ptr = url_parse_serial(path, query, system_id, component_id, true, shared_io);
  } else {
    throw DeviceError("url", "Unknown URL type");
  }

  return interface_ptr;
}

MAVConnInterface::Ptr MAVConnInterface::open_url(
  std::string url,
  uint8_t system_id,
  uint8_t component_id,
  const ReceivedCb & cb_handle_message,
  const ClosedCb & cb_handle_closed_port,
  asio::io_service * shared_io)
{
  auto interface_ptr = open_url_no_connect(url, system_id, component_id, shared_io);
  if (interface_ptr) {
    if (!cb_handle_message) {
      CONSOLE_BRIDGE_logWarn(
        PFX "You did not provide message handling callback to open_url(), "
        "It is unsafe to set it later.");
    }
    interface_ptr->connect(cb_handle_message, cb_handle_closed_port);
  }

  return interface_ptr;
}

}       // namespace mavconn
