//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Unit tests for FTPRequest payload header layout and accessors (#2323).
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "test_plugin_helpers.hpp"
#include "../src/plugins/ftp.cpp"  // NOLINT(build/include)

namespace mavros
{
namespace std_plugins
{

TEST(FTPRequestTest, payload_header_layout_and_size)
{
  EXPECT_EQ(sizeof(FTPRequest::PayloadHeader), 12U);
  EXPECT_EQ(offsetof(FTPRequest::PayloadHeader, seqNumber), 0U);
  EXPECT_EQ(offsetof(FTPRequest::PayloadHeader, session), 2U);
  EXPECT_EQ(offsetof(FTPRequest::PayloadHeader, opcode), 3U);
  EXPECT_EQ(offsetof(FTPRequest::PayloadHeader, size), 4U);
  EXPECT_EQ(offsetof(FTPRequest::PayloadHeader, req_opcode), 5U);
  EXPECT_EQ(offsetof(FTPRequest::PayloadHeader, padding), 6U);
  EXPECT_EQ(offsetof(FTPRequest::PayloadHeader, offset), 8U);
}

TEST(FTPRequestTest, header_get_set_roundtrip)
{
  FTPRequest req(FTPRequest::kCmdOpenFileRO, 42);

  auto hdr = req.header();
  EXPECT_EQ(hdr.opcode, FTPRequest::kCmdOpenFileRO);
  EXPECT_EQ(hdr.session, 42);
  EXPECT_EQ(hdr.seqNumber, 0);
  EXPECT_EQ(hdr.size, 0);
  EXPECT_EQ(hdr.offset, 0U);

  hdr.seqNumber = 12345;
  hdr.req_opcode = FTPRequest::kCmdNone;
  hdr.size = 64;
  hdr.offset = 0x12345678;
  req.set_header(hdr);

  const auto roundtrip = req.header();
  EXPECT_EQ(roundtrip.seqNumber, 12345);
  EXPECT_EQ(roundtrip.session, 42);
  EXPECT_EQ(roundtrip.opcode, FTPRequest::kCmdOpenFileRO);
  EXPECT_EQ(roundtrip.size, 64);
  EXPECT_EQ(roundtrip.req_opcode, FTPRequest::kCmdNone);
  EXPECT_EQ(roundtrip.offset, 0x12345678U);

  // Header bytes must land at the start of the payload, little-endian.
  const uint8_t expected[12] = {
    0x39, 0x30, 42, FTPRequest::kCmdOpenFileRO, 64, FTPRequest::kCmdNone, 0, 0,
    0x78, 0x56, 0x34, 0x12};
  EXPECT_EQ(std::memcmp(req.payload.data(), expected, sizeof(expected)), 0);
}

TEST(FTPRequestTest, data_u32_reads_after_header)
{
  FTPRequest req;
  const uint8_t wire[4] = {0xD4, 0xC3, 0xB2, 0xA1};
  std::memcpy(req.payload.data() + sizeof(FTPRequest::PayloadHeader), wire, sizeof(wire));

  EXPECT_EQ(req.data_u32(), 0xA1B2C3D4U);
}

TEST(FTPRequestTest, set_data_string)
{
  FTPRequest req;
  const std::string path = "/fs/microsd/test_file.txt";
  req.set_data_string(path);

  const auto hdr = req.header();
  EXPECT_EQ(hdr.size, path.size());
  EXPECT_STREQ(req.data_c(), path.c_str());
}

}  // namespace std_plugins

namespace uas
{

class FTPFlowTest : public TestUAS
{
public:
  UAS::SharedPtr uas;
  std::shared_ptr<mavros::std_plugins::FTPPlugin> plugin;
  std::shared_ptr<std::vector<mavros_msgs::msg::Mavlink>> out;
  rclcpp::Node::SharedPtr client;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(1, 1);

    plugin = std::make_shared<mavros::std_plugins::FTPPlugin>(uas.get());
    register_plugin(uas, plugin);

    add_node(uas);
    add_node(plugin->get_node());
    client = make_client_node("ftp_client");
    out = capture_sink(uas);
    start_spin();
  }

  void TearDown() override
  {
    stop_spin();
  }

  void inject_ftp(const mavros::std_plugins::FTPRequest & freq, uint8_t sysid = 1)
  {
    auto mmsg = convert_message(freq);
    mmsg.sysid = sysid;
    mmsg.compid = 1;
    mmsg.msgid = mavros::std_plugins::FTPRequest::MSG_ID;
    route(uas, &mmsg, mavconn::Framing::ok);
  }
};

TEST_F(FTPFlowTest, malformed_oversized_header_and_concurrent_services_do_not_crash)
{
  using mavros::std_plugins::FTPRequest;
  auto list_cli = client->create_client<mavros_msgs::srv::FileList>("/test_mavros_uas/ftp/list");
  auto checksum_cli = client->create_client<mavros_msgs::srv::FileChecksum>(
    "/test_mavros_uas/ftp/checksum");
  auto reset_cli = client->create_client<std_srvs::srv::Empty>("/test_mavros_uas/ftp/reset");
  ASSERT_TRUE(list_cli->wait_for_service(5s));
  ASSERT_TRUE(checksum_cli->wait_for_service(5s));
  ASSERT_TRUE(reset_cli->wait_for_service(5s));

  // 1. Start a list call and inject a corrupt packet with hdr.size = 250 (> DATA_MAXSZ = 239).
  //    Must be rejected with EBADMSG without out-of-bounds read or infinite loop.
  auto lreq = std::make_shared<mavros_msgs::srv::FileList::Request>();
  lreq->dir_path = "/";
  std::promise<mavros_msgs::srv::FileList::Response::SharedPtr> lprom;
  std::thread lthd([&]() {lprom.set_value(call_service(list_cli, lreq, 10s));});

  auto sent = wait_msgid(*out, FTPRequest::MSG_ID);
  ASSERT_EQ(sent.msgid, FTPRequest::MSG_ID);
  auto m_sent = TestUAS::to_mavlink(sent);
  FTPRequest decoded{};
  mavlink::MsgMap map(m_sent);
  decoded.deserialize(map);
  const uint16_t seq = decoded.header().seqNumber;

  FTPRequest bad_reply(FTPRequest::kRspAck, 0);
  auto bhdr = bad_reply.header();
  bhdr.seqNumber = seq + 1;
  bhdr.req_opcode = FTPRequest::kCmdListDirectory;
  bhdr.size = 250;  // exceeds DATA_MAXSZ (239)
  bad_reply.set_header(bhdr);
  inject_ftp(bad_reply);

  auto lres = lprom.get_future().get();
  lthd.join();
  ASSERT_NE(lres, nullptr);
  EXPECT_FALSE(lres->success);
  EXPECT_EQ(lres->r_errno, EBADMSG);

  // 2. Immediately run a valid checksum request and concurrent busy request to verify recovery.
  out->clear();
  auto creq = std::make_shared<mavros_msgs::srv::FileChecksum::Request>();
  creq->file_path = "/APM/scripts/test.lua";
  std::promise<mavros_msgs::srv::FileChecksum::Response::SharedPtr> cprom;
  std::thread cthd([&]() {cprom.set_value(call_service(checksum_cli, creq, 10s));});

  auto csent = wait_msgid(*out, FTPRequest::MSG_ID);
  ASSERT_EQ(csent.msgid, FTPRequest::MSG_ID);
  auto m_csent = TestUAS::to_mavlink(csent);
  FTPRequest cdecoded{};
  mavlink::MsgMap cmap(m_csent);
  cdecoded.deserialize(cmap);
  const uint16_t cseq = cdecoded.header().seqNumber;

  FTPRequest ok_reply(FTPRequest::kRspAck, 0);
  auto chdr = ok_reply.header();
  chdr.seqNumber = cseq + 1;
  chdr.req_opcode = FTPRequest::kCmdCalcFileCRC32;
  chdr.size = sizeof(uint32_t);
  ok_reply.set_header(chdr);
  const uint32_t expected_crc = 0xDEADBEEFU;
  std::memcpy(ok_reply.data(), &expected_crc, sizeof(expected_crc));
  inject_ftp(ok_reply);

  auto cres = cprom.get_future().get();
  cthd.join();
  ASSERT_NE(cres, nullptr);
  EXPECT_TRUE(cres->success);
  EXPECT_EQ(cres->crc32, expected_crc);
}

}  // namespace uas
}  // namespace mavros

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  mavros::uas::TestUAS::Init();
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
