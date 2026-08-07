//
// libmavconn
// Copyright 2013,2014,2015,2016,2018,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief MAVConn Serial link class
 * @file serial.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include <cassert>
#include <string>

#include "mavconn/console_bridge_compat.hpp"
#include "mavconn/serial.hpp"
#include "mavconn/thread_utils.hpp"

namespace mavconn
{

using asio::buffer;
using asio::io_context;
using mavlink::mavlink_message_t;
using std::error_code;

#define PFX "mavconn: serial"
#define PFXd PFX "%zu: "

MAVConnSerial::MAVConnSerial(
  uint8_t system_id, uint8_t component_id,
  std::string device, unsigned baudrate, bool hwflow, asio::io_context * shared_io)
: MAVConnInterface(system_id, component_id),
  io_runner(shared_io),
  io_context(io_runner.io()),
  serial_dev(io_context),
  tx_in_progress(false),
  tx_q{},
  rx_buf{}
{
  using SPB = asio::serial_port_base;

  CONSOLE_BRIDGE_logInform(PFXd "device: %s @ %d bps", conn_id, device.c_str(), baudrate);

  try {
    serial_dev.open(device);

    // Set baudrate and 8N1 mode
    serial_dev.set_option(SPB::baud_rate(baudrate));
    serial_dev.set_option(SPB::character_size(8));
    serial_dev.set_option(SPB::parity(SPB::parity::none));
    serial_dev.set_option(SPB::stop_bits(SPB::stop_bits::one));

#if ASIO_VERSION >= 101200 || !defined(__linux__)
    // Flow control setting in older versions of ASIO is broken, use workaround (below) for now.
    serial_dev.set_option(
      SPB::flow_control(
        (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
#elif ASIO_VERSION < 101200 && defined(__linux__)
    // Workaround to set some options for the port manually. This is done in
    // ASIO, but until v1.12.0 there was a bug which doesn't enable relevant
    // code. Fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
    {
      int fd = serial_dev.native_handle();

      termios tio;
      tcgetattr(fd, &tio);

      // Set hardware flow control settings
      if (hwflow) {
        tio.c_iflag &= ~(IXOFF | IXON);
        tio.c_cflag |= CRTSCTS;
      } else {
        tio.c_iflag &= ~(IXOFF | IXON);
        tio.c_cflag &= ~CRTSCTS;
      }

      // Set serial port to "raw" mode to prevent EOF exit.
      cfmakeraw(&tio);

      // Commit settings
      tcsetattr(fd, TCSANOW, &tio);
    }
#endif

#if defined(__linux__)
    // Enable low latency mode on Linux
    {
      int fd = serial_dev.native_handle();

      struct serial_struct ser_info;
      ioctl(fd, TIOCGSERIAL, &ser_info);

      ser_info.flags |= ASYNC_LOW_LATENCY;

      ioctl(fd, TIOCSSERIAL, &ser_info);
    }
#endif
  } catch (asio::system_error & err) {
    throw DeviceError("serial", err);
  }
}

MAVConnSerial::~MAVConnSerial()
{
  close();
}

void MAVConnSerial::connect(
  const ReceivedCb & cb_handle_message,
  const ClosedCb & cb_handle_closed_port)
{
  message_received_cb = cb_handle_message;
  port_closed_cb = cb_handle_closed_port;

  // give some work to io_context before start
  asio::post(io_context, [this]() {this->do_read();});

  if (io_runner.owns_thread()) {
    // run io_context for async io
    io_runner.start(
      [this]() {
        utils::set_this_thread_name("mserial%zu", conn_id);
        io_context.run();
      });
  }
}


void MAVConnSerial::close()
{
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!is_open()) {
      return;
    }

    serial_dev.cancel();
    serial_dev.close();
  }

  // Join the io thread without holding mutex: closing the device makes do_read
  // complete with an error on the io thread, whose handler calls close() again
  // and would block on mutex held here while we join it.
  if (io_runner.owns_thread()) {
    io_runner.shutdown_owned();
  }

  if (port_closed_cb) {
    port_closed_cb();
  }
}

void MAVConnSerial::send_bytes(const uint8_t * bytes, size_t length)
{
  if (!is_open()) {
    CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
    return;
  }

  bool start_chain = false;
  {
    std::lock_guard<std::mutex> lock(mutex);

    if (tx_q.size() >= MAX_TXQ_SIZE) {
      throw std::length_error("MAVConnSerial::send_bytes: TX queue overflow");
    }

    tx_q.emplace_back(bytes, length);
    if (!tx_in_progress) {
      tx_in_progress = true;
      start_chain = true;
    }
  }
  if (start_chain) {
    auto sthis = shared_from_this();
    asio::post(io_context, [sthis]() {sthis->do_write(false);});
  }
}

void MAVConnSerial::send_message(const mavlink_message_t * message)
{
  assert(message != nullptr);

  if (!is_open()) {
    CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
    return;
  }

  log_send(PFX, message);

  bool start_chain = false;
  {
    std::lock_guard<std::mutex> lock(mutex);

    if (tx_q.size() >= MAX_TXQ_SIZE) {
      throw std::length_error("MAVConnSerial::send_message: TX queue overflow");
    }

    tx_q.emplace_back(message);
    if (!tx_in_progress) {
      tx_in_progress = true;
      start_chain = true;
    }
  }
  if (start_chain) {
    auto sthis = shared_from_this();
    asio::post(io_context, [sthis]() {sthis->do_write(false);});
  }
}

void MAVConnSerial::send_message(const mavlink::Message & message, const uint8_t source_compid)
{
  if (!is_open()) {
    CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
    return;
  }

  log_send_obj(PFX, message);

  bool start_chain = false;
  {
    std::lock_guard<std::mutex> lock(mutex);

    if (tx_q.size() >= MAX_TXQ_SIZE) {
      throw std::length_error("MAVConnSerial::send_message: TX queue overflow");
    }

    tx_q.emplace_back(message, get_status_p(), sys_id, source_compid);
    if (!tx_in_progress) {
      tx_in_progress = true;
      start_chain = true;
    }
  }
  if (start_chain) {
    auto sthis = shared_from_this();
    asio::post(io_context, [sthis]() {sthis->do_write(false);});
  }
}

void MAVConnSerial::do_read(void)
{
  auto sthis = shared_from_this();
  serial_dev.async_read_some(
    buffer(rx_buf),
    [sthis](error_code error, size_t bytes_transferred) {
      if (error) {
        CONSOLE_BRIDGE_logError(PFXd "receive: %s", sthis->conn_id, error.message().c_str());
        sthis->close();
        return;
      }

      sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
      sthis->do_read();
    });
}

void MAVConnSerial::do_write(bool check_tx_state)
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
  // asio::async_write loops over partial writes internally and calls the
  // handler once the whole buffer is sent, so no manual pos tracking/resend.
  asio::async_write(
    serial_dev, buffer(buf_ref.dpos(), buf_ref.nbytes()),
    [sthis](error_code error, size_t bytes_transferred) {
      if (error) {
        CONSOLE_BRIDGE_logError(PFXd "write: %s", sthis->conn_id, error.message().c_str());
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

        sthis->tx_q.pop_front();

        if (!sthis->tx_q.empty()) {
          continue_send = true;
        } else {
          sthis->tx_in_progress = false;
        }
      }

      if (continue_send) {
        asio::post(sthis->io_context, [sthis]() {sthis->do_write(false);});
      }
    });
}

}  // namespace mavconn
