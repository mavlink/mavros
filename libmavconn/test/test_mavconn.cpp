//
// libmavconn
// Copyright 2013-2016,2018,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Test mavconn library
 */

// Gmock broken on Ubuntu (thrusty),
//  its gmock 1.6 require gtest 1.7, while repository only provides 1.6
//  this error exist one year without any updates.
//  https://bugs.launchpad.net/ubuntu/+source/google-mock/+bug/1201279
//
//  I think on Debian that was fixed while ago. But i can't use it in ros buildfarm.
// #include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <thread>

#include "mavconn/interface.hpp"
#include "mavconn/serial.hpp"
#include "mavconn/tcp.hpp"
#include "mavconn/udp.hpp"

using namespace mavconn; // NOLINT
using mavlink_message_t = mavlink::mavlink_message_t;
using msgid_t = mavlink::msgid_t;

namespace mavlink
{
namespace common
{
using namespace mavlink::minimal;     // NOLINT
namespace msg
{
using namespace mavlink::minimal::msg;         // NOLINT
}
}
}  // namespace mavlink

static void send_heartbeat(MAVConnInterface * ip)
{
  using mavlink::common::MAV_AUTOPILOT;
  using mavlink::common::MAV_MODE;
  using mavlink::common::MAV_STATE;
  using mavlink::common::MAV_TYPE;

  mavlink::common::msg::HEARTBEAT hb = {};
  hb.type = static_cast<int>(MAV_TYPE::ONBOARD_CONTROLLER);
  hb.autopilot = static_cast<int>(MAV_AUTOPILOT::INVALID);
  hb.base_mode = static_cast<int>(MAV_MODE::MANUAL_ARMED);
  hb.custom_mode = 0;
  hb.system_status = static_cast<int>(MAV_STATE::ACTIVE);

  ip->send_message(hb);
}

class DummyConn : public MAVConnInterface
{
public:
  using MAVConnInterface::iostat_rx_add;
  using MAVConnInterface::iostat_tx_add;

  void connect(
    const ReceivedCb & cb_handle_message [[maybe_unused]],
    const ClosedCb & cb_handle_closed_port [[maybe_unused]] = ClosedCb()) override
  {}

  void close() override {}

  void send_message(const mavlink_message_t * message [[maybe_unused]]) override {}

  void send_message(
    const mavlink::Message & message [[maybe_unused]],
    const uint8_t source_compid [[maybe_unused]]) override
  {}

  void send_bytes(
    const uint8_t * bytes [[maybe_unused]],
    size_t length [[maybe_unused]]) override
  {}

  bool is_open() override
  {
    return true;
  }
};

TEST(IOSTAT, finite_speed_for_subsecond_polling)
{
  DummyConn conn;
  conn.iostat_tx_add(64);
  conn.iostat_rx_add(128);

  const auto stat = conn.get_iostat();
  EXPECT_EQ(stat.tx_total_bytes, 64U);
  EXPECT_EQ(stat.rx_total_bytes, 128U);
  EXPECT_TRUE(std::isfinite(stat.tx_speed));
  EXPECT_TRUE(std::isfinite(stat.rx_speed));
}

class UDP : public ::testing::Test
{
public:
  std::mutex mutex;
  std::condition_variable cond;

  msgid_t message_id;

  void recv_message(const mavlink_message_t * message, const Framing framing [[maybe_unused]])
  {
    // printf("Got message %u, len: %u, framing: %d\n", message->msgid, message->len, int(framing));
    message_id = message->msgid;
    cond.notify_one();
  }

  bool wait_one()
  {
    std::unique_lock<std::mutex> lock(mutex);
    return cond.wait_for(lock, std::chrono::seconds(2)) == std::cv_status::no_timeout;
  }
};

#if 0
// XXX(vooon): temporary disable that check - it don't work on Travis (with ICI)
TEST_F(UDP, bind_error)
{
  MAVConnInterface::Ptr conns[2];

  conns[0] = std::make_shared<MAVConnUDP>(42, 200, "localhost", 45000);
  ASSERT_THROW(conns[1] = std::make_shared<MAVConnUDP>(42, 200, "localhost", 45000), DeviceError);
}
#endif

TEST_F(UDP, send_message)
{
  MAVConnInterface::Ptr echo, client;

  message_id = std::numeric_limits<msgid_t>::max();
  auto msgid = mavlink::common::msg::HEARTBEAT::MSG_ID;

  // create echo server
  echo = std::make_shared<MAVConnUDP>(42, 200, "0.0.0.0", 45002);
  echo->connect(
    [&](const mavlink_message_t * msg, const Framing framing [[maybe_unused]]) {
      echo->send_message(msg);
    });

  // create client
  client = std::make_shared<MAVConnUDP>(44, 200, "0.0.0.0", 45003, "localhost", 45002);
  client->connect(
    [this](const mavlink_message_t * msg, const Framing framing) {
      this->recv_message(msg, framing);
    });

  // wait echo
  send_heartbeat(client.get());
  send_heartbeat(client.get());
  EXPECT_TRUE(wait_one());
  EXPECT_EQ(message_id, msgid);
}

TEST(IO_THREAD, udp_shared_io_service_stays_running_after_close)
{
  asio::io_service shared_io;
  auto io_work = std::make_unique<asio::io_service::work>(shared_io);
  std::jthread io_thread([&shared_io]() {shared_io.run();});

  std::mutex mutex;
  std::condition_variable cond;
  bool got_echo = false;
  bool posted_done = false;

  auto echo = std::make_shared<MAVConnUDP>(
    42, 200, "0.0.0.0", 45022, "", MAVConnUDP::DEFAULT_REMOTE_PORT, &shared_io);
  echo->connect(
    [&](const mavlink_message_t * msg, const Framing framing [[maybe_unused]]) {
      echo->send_message(msg);
    });

  auto client = std::make_shared<MAVConnUDP>(
    44, 200, "0.0.0.0", 45023, "localhost", 45022, &shared_io);
  client->connect(
    [&](const mavlink_message_t * message [[maybe_unused]],
      const Framing framing [[maybe_unused]])
    {
      std::lock_guard<std::mutex> lock(mutex);
      got_echo = true;
      cond.notify_all();
    });

  send_heartbeat(client.get());
  send_heartbeat(client.get());
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cond.wait_for(lock, std::chrono::seconds(2), [&]() {return got_echo;}));
  }

  echo->close();

  shared_io.post([&]() {
    std::lock_guard<std::mutex> lock(mutex);
    posted_done = true;
    cond.notify_all();
  });
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cond.wait_for(lock, std::chrono::seconds(2), [&]() {return posted_done;}));
  }

  client->close();
  io_work.reset();
  shared_io.stop();
}

TEST(IO_THREAD, udp_close_from_callback_does_not_deadlock)
{
  std::mutex mutex;
  std::condition_variable cond;
  bool callback_finished = false;
  bool closed_callback_called = false;

  auto echo = std::make_shared<MAVConnUDP>(42, 200, "0.0.0.0", 45032);
  echo->connect(
    [&](const mavlink_message_t * msg, const Framing framing [[maybe_unused]]) {
      echo->send_message(msg);
    });

  auto client = std::make_shared<MAVConnUDP>(
    44, 200, "0.0.0.0", 45033, "localhost", 45032);
  client->connect(
    [&](const mavlink_message_t * message [[maybe_unused]],
      const Framing framing [[maybe_unused]])
    {
      client->close();
      std::lock_guard<std::mutex> lock(mutex);
      callback_finished = true;
      cond.notify_all();
    },
    [&]() {
      std::lock_guard<std::mutex> lock(mutex);
      closed_callback_called = true;
      cond.notify_all();
    });

  send_heartbeat(client.get());
  send_heartbeat(client.get());

  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_EQ(
      cond.wait_for(lock, std::chrono::seconds(2), [&]() {return callback_finished;}), true);
    EXPECT_EQ(
      cond.wait_for(lock, std::chrono::seconds(2), [&]() {return closed_callback_called;}), true);
  }

  echo->close();
}

class TCP : public UDP
{
};

static bool accept_unsigned_for_test(
  const mavlink::mavlink_status_t * status [[maybe_unused]],
  uint32_t msgid [[maybe_unused]])
{
  return true;
}

TEST_F(TCP, bind_error)
{
  MAVConnInterface::Ptr conns[2];

  conns[0] = std::make_shared<MAVConnTCPServer>(42, 200, "localhost", 57600);
  conns[0]->connect(MAVConnInterface::ReceivedCb());
  ASSERT_THROW(
    conns[1] = std::make_shared<MAVConnTCPServer>(
      42, 200, "localhost",
      57600), DeviceError);
}

TEST_F(TCP, connect_error)
{
  MAVConnInterface::Ptr client;
  ASSERT_THROW(
    client = std::make_shared<MAVConnTCPClient>(42, 200, "localhost", 57666),
    DeviceError);
}

TEST_F(TCP, send_message)
{
  MAVConnInterface::Ptr echo_server, client;

  message_id = std::numeric_limits<msgid_t>::max();
  auto msgid = mavlink::common::msg::HEARTBEAT::MSG_ID;

  // create echo server
  echo_server = std::make_shared<MAVConnTCPServer>(42, 200, "0.0.0.0", 57602);
  echo_server->connect(
    [&](const mavlink_message_t * msg, const Framing framing [[maybe_unused]]) {
      echo_server->send_message(msg);
    });

  // create client
  client = std::make_shared<MAVConnTCPClient>(44, 200, "localhost", 57602);
  client->connect(
    [this](const mavlink_message_t * msg, const Framing framing) {
      this->recv_message(msg, framing);
    });

  // wait echo
  send_heartbeat(client.get());
  send_heartbeat(client.get());
  EXPECT_TRUE(wait_one());
  EXPECT_EQ(message_id, msgid);
}

TEST_F(TCP, client_reconnect)
{
  MAVConnInterface::Ptr echo_server;
  MAVConnInterface::Ptr client1, client2;

  // create echo server
  echo_server = std::make_shared<MAVConnTCPServer>(42, 200, "0.0.0.0", 57604);
  echo_server->connect(
    [&](const mavlink_message_t * msg, const Framing framing [[maybe_unused]]) {
      echo_server->send_message(msg);
    });

  EXPECT_NO_THROW(
  {
    client1 = std::make_shared<MAVConnTCPClient>(44, 200, "localhost", 57604);
  });

  EXPECT_NO_THROW(
  {
    client2 = std::make_shared<MAVConnTCPClient>(45, 200, "localhost", 57604);
  });

  EXPECT_NO_THROW(
  {
    client1 = std::make_shared<MAVConnTCPClient>(46, 200, "localhost", 57604);
  });
}

TEST(SERIAL, open_error)
{
  MAVConnInterface::Ptr serial;
  ASSERT_THROW(
    serial = std::make_shared<MAVConnSerial>(
      42, 200, "/some/magic/not/exist/path",
      57600),
    DeviceError);
}

TEST(SIGNING, udp_signed_packet_is_accepted)
{
  std::mutex mutex;
  std::condition_variable cond;
  auto got = false;
  auto framing = Framing::incomplete;
  auto message_id = std::numeric_limits<msgid_t>::max();

  const std::array<uint8_t, 32> key {0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42,
    0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42,
    0x42, 0x42, 0x42, 0x42, 0x42, 0x42};

  auto receiver = std::make_shared<MAVConnUDP>(42, 200, "0.0.0.0", 45042);
  receiver->setup_signing(key, true, 1, 1U);
  receiver->connect(
    [&](const mavlink_message_t * message, const Framing msg_framing) {
      std::lock_guard<std::mutex> lock(mutex);
      message_id = message->msgid;
      framing = msg_framing;
      got = true;
      cond.notify_all();
    });

  auto sender = std::make_shared<MAVConnUDP>(44, 200, "0.0.0.0", 45043, "localhost", 45042);
  sender->setup_signing(key, true, 2, 1U);
  sender->connect(MAVConnInterface::ReceivedCb());

  send_heartbeat(sender.get());
  send_heartbeat(sender.get());

  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cond.wait_for(lock, std::chrono::seconds(2), [&]() {return got;}));
  }
  EXPECT_EQ(framing, Framing::ok);
  EXPECT_EQ(message_id, mavlink::common::msg::HEARTBEAT::MSG_ID);

  sender->close();
  receiver->close();
}

TEST(SIGNING, udp_signature_mismatch_is_reported)
{
  std::mutex mutex;
  std::condition_variable cond;
  auto got = false;
  auto framing = Framing::incomplete;

  const std::array<uint8_t, 32> sender_key {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
  const std::array<uint8_t, 32> receiver_key {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};

  auto receiver = std::make_shared<MAVConnUDP>(42, 200, "0.0.0.0", 45044);
  receiver->setup_signing(receiver_key, true, 1, 1U);
  receiver->connect(
    [&](const mavlink_message_t * message [[maybe_unused]], const Framing msg_framing) {
      std::lock_guard<std::mutex> lock(mutex);
      framing = msg_framing;
      got = true;
      cond.notify_all();
    });

  auto sender = std::make_shared<MAVConnUDP>(44, 200, "0.0.0.0", 45045, "localhost", 45044);
  sender->setup_signing(sender_key, true, 2, 1U);
  sender->connect(MAVConnInterface::ReceivedCb());

  send_heartbeat(sender.get());
  send_heartbeat(sender.get());

  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cond.wait_for(lock, std::chrono::seconds(2), [&]() {return got;}));
  }
  EXPECT_EQ(framing, Framing::bad_signature);

  sender->close();
  receiver->close();
}

TEST(SIGNING, udp_accept_unsigned_callback_allows_unsigned_packets)
{
  std::mutex mutex;
  std::condition_variable cond;
  auto got = false;
  auto framing = Framing::incomplete;
  auto message_id = std::numeric_limits<msgid_t>::max();

  const std::array<uint8_t, 32> receiver_key {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
    0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
    0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33};

  auto receiver = std::make_shared<MAVConnUDP>(42, 200, "0.0.0.0", 45046);
  receiver->setup_signing(receiver_key, true, 1, 1U);
  receiver->set_accept_unsigned_callback(accept_unsigned_for_test);
  receiver->connect(
    [&](const mavlink_message_t * message, const Framing msg_framing) {
      std::lock_guard<std::mutex> lock(mutex);
      message_id = message->msgid;
      framing = msg_framing;
      got = true;
      cond.notify_all();
    });

  auto sender = std::make_shared<MAVConnUDP>(44, 200, "0.0.0.0", 45047, "localhost", 45046);
  sender->connect(MAVConnInterface::ReceivedCb());

  send_heartbeat(sender.get());
  send_heartbeat(sender.get());

  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cond.wait_for(lock, std::chrono::seconds(2), [&]() {return got;}));
  }
  EXPECT_EQ(framing, Framing::ok);
  EXPECT_EQ(message_id, mavlink::common::msg::HEARTBEAT::MSG_ID);

  sender->close();
  receiver->close();
}

#if 0
TEST(URL, open_url_serial)
{
  MAVConnInterface::Ptr serial;
  MAVConnSerial * serial_p;

  /* not best way to test tty access,
   * but it does not require any preparation
   * Disabled because it breaks terminal.
   */
  EXPECT_NO_THROW(
  {
    serial = MAVConnInterface::open_url("/dev/tty:115200");
    serial_p = dynamic_cast<MAVConnSerial *>(serial.get());
    EXPECT_NE(serial_p, nullptr);
  });

  EXPECT_NO_THROW(
  {
    serial = MAVConnInterface::open_url("serial:///dev/tty:115200?ids=2,240");
    serial_p = dynamic_cast<MAVConnSerial *>(serial.get());
    EXPECT_NE(serial_p, nullptr);
  });
}
#endif

TEST(URL, open_url_udp)
{
  MAVConnInterface::Ptr udp;
  MAVConnUDP * udp_p;

  EXPECT_NO_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:45004@localhost:45005/?ids=2,241");
    udp_p = dynamic_cast<MAVConnUDP *>(udp.get());
    EXPECT_NE(udp_p, nullptr);
  });

  EXPECT_NO_THROW(
  {
    udp = MAVConnInterface::open_url("udp://@localhost:45005");
    udp_p = dynamic_cast<MAVConnUDP *>(udp.get());
    EXPECT_NE(udp_p, nullptr);
  });

  EXPECT_NO_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:45006@");
    udp_p = dynamic_cast<MAVConnUDP *>(udp.get());
    EXPECT_NE(udp_p, nullptr);
  });

  EXPECT_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:45008");
  },
    DeviceError);

  EXPECT_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:0@localhost:14550");
  },
    DeviceError);

  EXPECT_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:65536@localhost:14550");
  },
    DeviceError);

  EXPECT_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:14550x@localhost:14550");
  },
    DeviceError);

  EXPECT_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:14550@localhost:14550/?ids=300,1");
  },
    DeviceError);

  EXPECT_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:14550@localhost:14550/?ids=-1,1");
  },
    DeviceError);

  EXPECT_THROW(
  {
    udp = MAVConnInterface::open_url("udp://localhost:14550@localhost:14550/?ids=a,1");
  },
    DeviceError);
}

TEST(URL, open_url_tcp)
{
  MAVConnInterface::Ptr tcp_server, tcp_client;

  MAVConnTCPServer * tcp_server_p;
  MAVConnTCPClient * tcp_client_p;

  EXPECT_NO_THROW(
  {
    tcp_server = MAVConnInterface::open_url("tcp-l://localhost:57606");
    tcp_server_p = dynamic_cast<MAVConnTCPServer *>(tcp_server.get());
    EXPECT_NE(tcp_server_p, nullptr);
  });

  EXPECT_NO_THROW(
  {
    tcp_client = MAVConnInterface::open_url("tcp://localhost:57606");
    tcp_client_p = dynamic_cast<MAVConnTCPClient *>(tcp_client.get());
    EXPECT_NE(tcp_client_p, nullptr);
  });

  EXPECT_THROW(
  {
    tcp_client = MAVConnInterface::open_url("tcp://localhost:0");
  },
    DeviceError);

  EXPECT_THROW(
  {
    tcp_client = MAVConnInterface::open_url("tcp://localhost:abc");
  },
    DeviceError);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
