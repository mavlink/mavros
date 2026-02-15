//
// libmavconn
// Copyright 2014,2015,2016,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief MAVConn TCP link classes
 * @file tcp.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */

#include <cassert>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "mavconn/console_bridge_compat.hpp"
#include "mavconn/tcp.hpp"
#include "mavconn/thread_utils.hpp"

namespace mavconn
{

using asio::buffer;
using asio::error_code;
using asio::io_service;
using asio::ip::tcp;
using mavlink::mavlink_message_t;
using mavlink::mavlink_status_t;
using utils::to_string_ss;

#define PFX "mavconn: tcp"
#define PFXd PFX "%zu: "

static asio::io_service & get_socket_io_service(tcp::socket & sock)
{
#if ASIO_VERSION >= 101400
  return static_cast<asio::io_context &>(sock.get_executor().context());
#else
  return sock.get_io_service();
#endif
}

static bool resolve_address_tcp(
  io_service & io, size_t chan, std::string host, uint16_t port,
  tcp::endpoint & ep)
{
  bool result = false;
  tcp::resolver resolver(io);
  error_code ec;

  tcp::resolver::query query(host, "");

  auto fn = [&](const tcp::endpoint & q_ep) {
      ep = q_ep;
      ep.port(port);
      result = true;
      CONSOLE_BRIDGE_logDebug(
        PFXd "host %s resolved as %s", chan, host.c_str(), to_string_ss(ep).c_str());
    };

#if ASIO_VERSION >= 101200
  for (auto q_ep : resolver.resolve(query, ec)) {
    fn(q_ep);
  }
#else
  std::for_each(resolver.resolve(query, ec), tcp::resolver::iterator(), fn);
#endif

  if (ec) {
    CONSOLE_BRIDGE_logWarn(PFXd "resolve error: %s", chan, ec.message().c_str());
    result = false;
  }

  return result;
}

/* -*- TCP client variant -*- */

MAVConnTCPClient::MAVConnTCPClient(
  uint8_t system_id, uint8_t component_id,
  std::string server_host, uint16_t server_port, asio::io_service * shared_io)
: MAVConnInterface(system_id, component_id),
  io_runner(shared_io),
  io_service(io_runner.io()),
  socket(io_service),
  is_destroying(false),
  tx_in_progress(false),
  tx_q{},
  rx_buf{}
{
  if (!resolve_address_tcp(io_service, conn_id, server_host, server_port, server_ep)) {
    throw DeviceError("tcp: resolve", "Bind address resolve failed");
  }

  CONSOLE_BRIDGE_logInform(PFXd "Server address: %s", conn_id, to_string_ss(server_ep).c_str());

  try {
    socket.open(tcp::v4());
    socket.connect(server_ep);
  } catch (asio::system_error & err) {
    throw DeviceError("tcp", err);
  }
}

MAVConnTCPClient::MAVConnTCPClient(
  uint8_t system_id, uint8_t component_id,
  asio::io_service & server_io)
: MAVConnInterface(system_id, component_id),
  io_runner(&server_io),
  io_service(io_runner.io()),
  socket(io_service),
  is_destroying(false),
  tx_in_progress(false),
  tx_q{},
  rx_buf{}
{
  // waiting when server call client_connected()
}

void MAVConnTCPClient::client_connected(size_t server_channel)
{
  CONSOLE_BRIDGE_logInform(
    PFXd "Got client, id: %zu, address: %s",
    server_channel, conn_id, to_string_ss(server_ep).c_str());

  // start recv
  auto sthis = shared_from_this();
  get_socket_io_service(socket).post([sthis]() {sthis->do_recv();});
}

MAVConnTCPClient::~MAVConnTCPClient()
{
  is_destroying = true;
  close();

  // If the client is already disconnected on error (By the io_service thread)
  // and io_service running
  if (io_runner.owns_thread() && io_runner.is_running()) {
    stop();
  }
}

void MAVConnTCPClient::connect(
  const ReceivedCb & cb_handle_message,
  const ClosedCb & cb_handle_closed_port)
{
  message_received_cb = cb_handle_message;
  port_closed_cb = cb_handle_closed_port;

  // give some work to io_service before start
  io_service.post([this]() {this->do_recv();});

  if (io_runner.owns_thread()) {
    // run io_service for async io
    io_runner.start(
      [this]() {
        utils::set_this_thread_name("mtcp%zu", conn_id);
        try {
          io_service.run();
        } catch (std::exception & ex) {
          CONSOLE_BRIDGE_logError(PFXd "io_service exception: %s", conn_id, ex.what());
        }
      });
  }
}

void MAVConnTCPClient::stop()
{
  if (!io_runner.owns_thread()) {
    return;
  }

  io_runner.shutdown_owned();
}

void MAVConnTCPClient::close()
{
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!is_open()) {
      return;
    }

    std::error_code ec;
    socket.shutdown(asio::ip::tcp::socket::shutdown_send, ec);
    if (ec) {
      CONSOLE_BRIDGE_logError(PFXd "shutdown: %s", conn_id, ec.message().c_str());
    }
    socket.cancel();
    socket.close();
  }

  // For owned contexts this is safe from callbacks: shutdown avoids self-join.
  if (io_runner.owns_thread()) {
    stop();
  }

  if (port_closed_cb) {
    port_closed_cb();
  }
}

void MAVConnTCPClient::send_bytes(const uint8_t * bytes, size_t length)
{
  if (!is_open()) {
    CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex);

    if (tx_q.size() >= MAX_TXQ_SIZE) {
      throw std::length_error("MAVConnTCPClient::send_bytes: TX queue overflow");
    }

    tx_q.emplace_back(bytes, length);
  }
  auto sthis = shared_from_this();
  get_socket_io_service(socket).post([sthis]() {sthis->do_send(true);});
}

void MAVConnTCPClient::send_message(const mavlink_message_t * message)
{
  assert(message != nullptr);

  if (!is_open()) {
    CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
    return;
  }

  log_send(PFX, message);

  {
    std::lock_guard<std::mutex> lock(mutex);

    if (tx_q.size() >= MAX_TXQ_SIZE) {
      throw std::length_error("MAVConnTCPClient::send_message: TX queue overflow");
    }

    tx_q.emplace_back(message);
  }
  auto sthis = shared_from_this();
  get_socket_io_service(socket).post([sthis]() {sthis->do_send(true);});
}

void MAVConnTCPClient::send_message(const mavlink::Message & message, const uint8_t source_compid)
{
  if (!is_open()) {
    CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
    return;
  }

  log_send_obj(PFX, message);

  {
    std::lock_guard<std::mutex> lock(mutex);

    if (tx_q.size() >= MAX_TXQ_SIZE) {
      throw std::length_error("MAVConnTCPClient::send_message: TX queue overflow");
    }

    tx_q.emplace_back(message, get_status_p(), sys_id, source_compid);
  }
  auto sthis = shared_from_this();
  get_socket_io_service(socket).post([sthis]() {sthis->do_send(true);});
}

void MAVConnTCPClient::do_recv()
{
  if (is_destroying) {
    return;
  }
  auto sthis = shared_from_this();
  socket.async_receive(
    buffer(rx_buf),
    [sthis](error_code error, size_t bytes_transferred) {
      if (error) {
        CONSOLE_BRIDGE_logError(PFXd "receive: %s", sthis->conn_id, error.message().c_str());
        sthis->close();
        return;
      }

      sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
      sthis->do_recv();
    });
}

void MAVConnTCPClient::do_send(bool check_tx_state)
{
  if (check_tx_state && tx_in_progress) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex);
  if (tx_q.empty()) {
    return;
  }

  tx_in_progress = true;
  auto sthis = shared_from_this();
  auto & buf_ref = tx_q.front();
  socket.async_send(
    buffer(buf_ref.dpos(), buf_ref.nbytes()),
    [sthis, &buf_ref](error_code error, size_t bytes_transferred) {
      assert(ssize_t(bytes_transferred) <= buf_ref.len);

      if (error) {
        CONSOLE_BRIDGE_logError(PFXd "send: %s", sthis->conn_id, error.message().c_str());
        sthis->close();
        return;
      }

      sthis->iostat_tx_add(bytes_transferred);
      bool continue_send = false;
      {
        std::lock_guard<std::mutex> lock(sthis->mutex);

        if (sthis->tx_q.empty()) {
          sthis->tx_in_progress = false;
          return;
        }

        buf_ref.pos += bytes_transferred;
        if (buf_ref.nbytes() == 0) {
          sthis->tx_q.pop_front();
        }

        if (!sthis->tx_q.empty()) {
          continue_send = true;
        } else {
          sthis->tx_in_progress = false;
        }
      }

      if (continue_send) {
        get_socket_io_service(sthis->socket).post([sthis]() {sthis->do_send(false);});
      }
    });
}

/* -*- TCP server variant -*- */

MAVConnTCPServer::MAVConnTCPServer(
  uint8_t system_id, uint8_t component_id,
  std::string server_host, uint16_t server_port, asio::io_service * shared_io)
: MAVConnInterface(system_id, component_id),
  io_runner(shared_io),
  io_service(io_runner.io()),
  acceptor(io_service),
  is_destroying(false)
{
  if (!resolve_address_tcp(io_service, conn_id, server_host, server_port, bind_ep)) {
    throw DeviceError("tcp-l: resolve", "Bind address resolve failed");
  }

  CONSOLE_BRIDGE_logInform(PFXd "Bind address: %s", conn_id, to_string_ss(bind_ep).c_str());

  try {
    acceptor.open(tcp::v4());
    acceptor.set_option(tcp::acceptor::reuse_address(true));
    acceptor.bind(bind_ep);
    acceptor.listen();
  } catch (asio::system_error & err) {
    throw DeviceError("tcp-l", err);
  }
}

MAVConnTCPServer::~MAVConnTCPServer()
{
  is_destroying = true;
  close();
}

void MAVConnTCPServer::connect(
  const ReceivedCb & cb_handle_message,
  const ClosedCb & cb_handle_closed_port)
{
  message_received_cb = cb_handle_message;
  port_closed_cb = cb_handle_closed_port;

  // give some work to io_service before start
  io_service.post([this]() {this->do_accept();});

  if (io_runner.owns_thread()) {
    // run io_service for async io
    io_runner.start(
      [this]() {
        utils::set_this_thread_name("mtcps%zu", conn_id);
        io_service.run();
      });
  }
}

void MAVConnTCPServer::close()
{
  std::vector<std::shared_ptr<MAVConnTCPClient>> clients;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!is_open()) {
      return;
    }
    is_destroying = true;
    clients.assign(client_list.begin(), client_list.end());
    client_list.clear();
  }

  CONSOLE_BRIDGE_logInform(
    PFXd "Terminating server. "
    "All connections will be closed.",
    conn_id);

  acceptor.close();
  for (auto & instp : clients) {
    instp->close();
  }

  if (io_runner.owns_thread()) {
    io_runner.shutdown_owned();
  }

  if (port_closed_cb) {
    port_closed_cb();
  }
}

mavlink_status_t MAVConnTCPServer::get_status()
{
  mavlink_status_t status {};
  std::vector<std::shared_ptr<MAVConnTCPClient>> clients;

  {
    std::lock_guard<std::mutex> lock(mutex);
    clients.assign(client_list.begin(), client_list.end());
  }
  for (auto & instp : clients) {
    auto inst_status = instp->get_status();

    // [[[cog:
    // for f in ('packet_rx_success_count', 'packet_rx_drop_count',
    //     'buffer_overrun', 'parse_error'):
    //     cog.outl("status.{f} += inst_status.{f};".format(**locals()))
    // ]]]
    status.packet_rx_success_count += inst_status.packet_rx_success_count;
    status.packet_rx_drop_count += inst_status.packet_rx_drop_count;
    status.buffer_overrun += inst_status.buffer_overrun;
    status.parse_error += inst_status.parse_error;
    // [[[end]]] (sum: zVguRtOlY8)

    /* seq counters always 0 for this connection type */
  }

  return status;
}

MAVConnInterface::IOStat MAVConnTCPServer::get_iostat()
{
  MAVConnInterface::IOStat iostat {};
  std::vector<std::shared_ptr<MAVConnTCPClient>> clients;

  {
    std::lock_guard<std::mutex> lock(mutex);
    clients.assign(client_list.begin(), client_list.end());
  }
  for (auto & instp : clients) {
    auto inst_iostat = instp->get_iostat();

    // [[[cog:
    // for p in ('tx', 'rx'):
    //     for f in ('total_bytes', 'speed'):
    //         cog.outl("iostat.{p}_{f} += inst_iostat.{p}_{f};".format(**locals()))
    // ]]]
    iostat.tx_total_bytes += inst_iostat.tx_total_bytes;
    iostat.tx_speed += inst_iostat.tx_speed;
    iostat.rx_total_bytes += inst_iostat.rx_total_bytes;
    iostat.rx_speed += inst_iostat.rx_speed;
    // [[[end]]] (sum: uGx8hu4rFe)
  }

  return iostat;
}

void MAVConnTCPServer::send_bytes(const uint8_t * bytes, size_t length)
{
  std::vector<std::shared_ptr<MAVConnTCPClient>> clients;
  {
    std::lock_guard<std::mutex> lock(mutex);
    clients.assign(client_list.begin(), client_list.end());
  }
  for (auto & instp : clients) {
    instp->send_bytes(bytes, length);
  }
}

void MAVConnTCPServer::send_message(const mavlink_message_t * message)
{
  std::vector<std::shared_ptr<MAVConnTCPClient>> clients;
  {
    std::lock_guard<std::mutex> lock(mutex);
    clients.assign(client_list.begin(), client_list.end());
  }
  for (auto & instp : clients) {
    instp->send_message(message);
  }
}

void MAVConnTCPServer::send_message(const mavlink::Message & message, const uint8_t source_compid)
{
  std::vector<std::shared_ptr<MAVConnTCPClient>> clients;
  {
    std::lock_guard<std::mutex> lock(mutex);
    clients.assign(client_list.begin(), client_list.end());
  }
  for (auto & instp : clients) {
    instp->send_message(message, source_compid);
  }
}

void MAVConnTCPServer::do_accept()
{
  if (is_destroying) {
    return;
  }
  auto sthis = shared_from_this();
  auto acceptor_client = std::make_shared<MAVConnTCPClient>(sys_id, comp_id, io_service);
  acceptor.async_accept(
    acceptor_client->socket,
    acceptor_client->server_ep,
    [sthis, acceptor_client](error_code error) {
      if (error) {
        CONSOLE_BRIDGE_logError(PFXd "accept: %s", sthis->conn_id, error.message().c_str());
        sthis->close();
        return;
      }

      std::weak_ptr<MAVConnTCPClient> weak_client{acceptor_client};
      {
        std::lock_guard<std::mutex> lock(sthis->mutex);

        acceptor_client->message_received_cb = sthis->message_received_cb;
        acceptor_client->port_closed_cb = [weak_client, sthis]() {
            sthis->client_closed(weak_client);
          };

        sthis->client_list.push_back(acceptor_client);
      }

      acceptor_client->client_connected(sthis->conn_id);
      sthis->do_accept();
    });
}

void MAVConnTCPServer::client_closed(std::weak_ptr<MAVConnTCPClient> weak_instp)
{
  if (auto instp = weak_instp.lock()) {
    CONSOLE_BRIDGE_logInform(
      PFXd "Client connection closed, id: %p, address: %s",
      conn_id, instp.get(), to_string_ss(instp->server_ep).c_str());

    {
      std::lock_guard<std::mutex> lock(mutex);
      client_list.remove(instp);
    }
  }
}
}       // namespace mavconn
