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

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

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
}  // namespace mavros

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
