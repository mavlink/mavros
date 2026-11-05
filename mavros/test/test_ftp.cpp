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
#include <algorithm>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
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

using mavros::std_plugins::FTPRequest;

/**
 * Drives the FTP plugin services against a scripted FCU: every request the
 * plugin sends is passed to `fcu`, and the replies it returns are routed back.
 */
class FTPFlowTest : public TestUAS
{
public:
  using Replies = std::vector<FTPRequest>;
  using Responder = std::function<Replies(const FTPRequest &)>;

  UAS::SharedPtr uas;
  std::shared_ptr<mavros::std_plugins::FTPPlugin> plugin;
  rclcpp::Node::SharedPtr client;
  rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr fcu_sub;
  std::mutex fcu_mutex;
  Responder fcu;
  std::vector<FTPRequest> sent;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(1, 1);

    plugin = std::make_shared<mavros::std_plugins::FTPPlugin>(uas.get());
    register_plugin(uas, plugin);

    add_node(uas);
    add_node(plugin->get_node());
    client = make_client_node("ftp_client");
    fcu_sub = client->create_subscription<mavros_msgs::msg::Mavlink>(
      "/test_mavros_uas/mavlink_sink",
      rclcpp::QoS(1000).best_effort().durability_volatile(),
      [this](const mavros_msgs::msg::Mavlink::SharedPtr m) {on_sent(*m);});
    start_spin();
  }

  void TearDown() override
  {
    stop_spin();
  }

  void on_sent(const mavros_msgs::msg::Mavlink & rmsg)
  {
    if (rmsg.msgid != FTPRequest::MSG_ID) {
      return;
    }

    auto mmsg = to_mavlink(rmsg);
    FTPRequest req;
    mavlink::MsgMap map(mmsg);
    req.deserialize(map);

    Replies replies;
    {
      std::lock_guard<std::mutex> lk(fcu_mutex);
      sent.push_back(req);
      if (fcu) {
        replies = fcu(req);
      }
    }
    for (auto & r : replies) {
      inject(r);
    }
  }

  void inject(const FTPRequest & reply)
  {
    auto mmsg = convert_message(reply);
    mmsg.sysid = 1;
    mmsg.compid = 1;
    mmsg.msgid = FTPRequest::MSG_ID;
    route(uas, &mmsg, mavconn::Framing::ok);
  }

  void set_fcu(Responder r)
  {
    std::lock_guard<std::mutex> lk(fcu_mutex);
    fcu = std::move(r);
  }

  static FTPRequest ack(const FTPRequest & req, const std::vector<uint8_t> & data = {})
  {
    FTPRequest r(FTPRequest::kRspAck, req.header().session);
    auto hdr = r.header();
    hdr.seqNumber = req.header().seqNumber + 1;
    hdr.req_opcode = req.header().opcode;
    hdr.offset = req.header().offset;
    hdr.size = data.size();
    r.set_header(hdr);
    std::copy(data.begin(), data.end(), r.data());
    return r;
  }

  static FTPRequest nak(const FTPRequest & req, uint8_t code, int err = -1)
  {
    std::vector<uint8_t> data{code};
    if (err >= 0) {
      data.push_back(err);
    }
    auto r = ack(req, data);
    auto hdr = r.header();
    hdr.opcode = FTPRequest::kRspNak;
    r.set_header(hdr);
    return r;
  }

  static std::vector<uint8_t> bytes(const std::string & s)
  {
    return std::vector<uint8_t>(s.begin(), s.end());
  }

  static std::vector<uint8_t> u32(uint32_t v)
  {
    std::vector<uint8_t> d(4);
    std::memcpy(d.data(), &v, 4);
    return d;
  }

  template<class Srv>
  typename Srv::Response::SharedPtr call(
    const std::string & name,
    const typename Srv::Request::SharedPtr & req)
  {
    auto cli = client->create_client<Srv>("/test_mavros_uas/ftp/" + name);
    EXPECT_TRUE(cli->wait_for_service(5s));
    return call_service(cli, req, 15s);
  }

  mavros_msgs::srv::FileList::Response::SharedPtr list(const std::string & path)
  {
    auto req = std::make_shared<mavros_msgs::srv::FileList::Request>();
    req->dir_path = path;
    return call<mavros_msgs::srv::FileList>("list", req);
  }

  mavros_msgs::srv::FileChecksum::Response::SharedPtr checksum(const std::string & path)
  {
    auto req = std::make_shared<mavros_msgs::srv::FileChecksum::Request>();
    req->file_path = path;
    return call<mavros_msgs::srv::FileChecksum>("checksum", req);
  }

  mavros_msgs::srv::FileOpen::Response::SharedPtr open(const std::string & path, uint8_t mode)
  {
    auto req = std::make_shared<mavros_msgs::srv::FileOpen::Request>();
    req->file_path = path;
    req->mode = mode;
    return call<mavros_msgs::srv::FileOpen>("open", req);
  }
};

TEST_F(FTPFlowTest, list_not_found_at_offset_zero_is_error)
{
  set_fcu(
    [](const FTPRequest & req) {
      return Replies{nak(req, FTPRequest::kErrFileNotFound)};
    });
  auto res = list("/nonexistent");
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->r_errno, ENOENT);

  set_fcu(
    [](const FTPRequest & req) {
      return Replies{nak(req, FTPRequest::kErrFailErrno, ENOENT)};
    });
  res = list("/nonexistent");
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->r_errno, ENOENT);
}

TEST_F(FTPFlowTest, list_errno_after_entries_ends_listing)
{
  set_fcu(
    [](const FTPRequest & req) {
      if (req.header().offset == 0) {
        // ArduPilot layout: each entry followed by two NULs
        return Replies{ack(req, bytes(std::string("Fa.txt\t12\0\0Dsub\0\0", 17)))};
      }
      return Replies{nak(req, FTPRequest::kErrFailErrno, ENOENT)};
    });
  auto res = list("/");
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  ASSERT_EQ(res->list.size(), 2U);
  EXPECT_EQ(res->list[0].name, "a.txt");
  EXPECT_EQ(res->list[0].size, 12U);
  EXPECT_EQ(res->list[1].name, "sub");
}

TEST_F(FTPFlowTest, list_malformed_entry_fails)
{
  for (const auto & payload : {std::string("Fok\0X\0", 6), std::string("Fbad\tsize\0", 10),
      std::string("\0\0", 2)})
  {
    set_fcu(
      [payload](const FTPRequest & req) {
        if (req.header().offset == 0) {
          return Replies{ack(req, bytes(payload))};
        }
        return Replies{nak(req, FTPRequest::kErrEOF)};
      });
    auto res = list("/");
    ASSERT_NE(res, nullptr);
    EXPECT_FALSE(res->success);
    EXPECT_EQ(res->r_errno, ERANGE);
  }
}

TEST_F(FTPFlowTest, oversized_payload_is_rejected_and_recovers)
{
  set_fcu(
    [](const FTPRequest & req) {
      auto r = ack(req);
      auto hdr = r.header();
      hdr.size = 250;
      r.set_header(hdr);
      return Replies{r};
    });
  auto res = list("/");
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->r_errno, EBADMSG);

  set_fcu([](const FTPRequest & req) {return Replies{ack(req, u32(0xDEADBEEF))};});
  auto cres = checksum("/a.txt");
  ASSERT_NE(cres, nullptr);
  EXPECT_TRUE(cres->success);
  EXPECT_EQ(cres->crc32, 0xDEADBEEFU);
}

TEST_F(FTPFlowTest, stale_reply_is_ignored)
{
  set_fcu(
    [](const FTPRequest & req) {
      // late reply to the previous request, then the real one
      auto stale = ack(req, u32(0x11111111));
      auto hdr = stale.header();
      hdr.seqNumber = req.header().seqNumber;
      stale.set_header(hdr);
      return Replies{stale, ack(req, u32(0x22222222))};
    });
  auto res = checksum("/a.txt");
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->crc32, 0x22222222U);
}

TEST_F(FTPFlowTest, no_reply_times_out_and_recovers)
{
  set_fcu([](const FTPRequest &) {return Replies{};});
  auto rm = std::make_shared<mavros_msgs::srv::FileRemove::Request>();
  rm->file_path = "/a.txt";
  auto res = call<mavros_msgs::srv::FileRemove>("remove", rm);
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->r_errno, ETIMEDOUT);

  set_fcu([](const FTPRequest & req) {return Replies{ack(req, u32(0x12345678))};});
  auto cres = checksum("/a.txt");
  ASSERT_NE(cres, nullptr);
  EXPECT_TRUE(cres->success);
  EXPECT_EQ(cres->crc32, 0x12345678U);
}

TEST_F(FTPFlowTest, write_accepts_empty_acks)
{
  std::vector<uint8_t> written;
  set_fcu(
    [&written](const FTPRequest & req) {
      // ArduPilot ACKs CreateFile and WriteFile with an empty payload
      if (req.header().opcode == FTPRequest::kCmdWriteFile) {
        EXPECT_EQ(req.header().offset, written.size());
        written.insert(written.end(), req.data(), req.data() + req.header().size);
      }
      return Replies{ack(req)};
    });

  auto ores = open("/a.bin", mavros_msgs::srv::FileOpen::Request::MODE_CREATE);
  ASSERT_NE(ores, nullptr);
  ASSERT_TRUE(ores->success);

  auto wreq = std::make_shared<mavros_msgs::srv::FileWrite::Request>();
  wreq->file_path = "/a.bin";
  for (int i = 0; i < 600; i++) {
    wreq->data.push_back(i & 0xff);
  }
  const auto expected = wreq->data;
  auto wres = call<mavros_msgs::srv::FileWrite>("write", wreq);
  ASSERT_NE(wres, nullptr);
  EXPECT_TRUE(wres->success);
  EXPECT_EQ(written, expected);
}

TEST_F(FTPFlowTest, read_multi_chunk)
{
  std::vector<uint8_t> file(1000);
  for (size_t i = 0; i < file.size(); i++) {
    file[i] = i * 7;
  }
  set_fcu(
    [&file](const FTPRequest & req) {
      const auto hdr = req.header();
      if (hdr.opcode == FTPRequest::kCmdOpenFileRO) {
        return Replies{ack(req, u32(file.size()))};
      }
      if (hdr.opcode == FTPRequest::kCmdReadFile) {
        if (hdr.offset >= file.size()) {
          return Replies{nak(req, FTPRequest::kErrEOF)};
        }
        const size_t n = std::min<size_t>(
          {hdr.size, FTPRequest::DATA_MAXSZ, file.size() - hdr.offset});
        return Replies{ack(
            req, std::vector<uint8_t>(
              file.begin() + hdr.offset, file.begin() + hdr.offset + n))};
      }
      return Replies{ack(req)};
    });

  auto ores = open("/a.bin", mavros_msgs::srv::FileOpen::Request::MODE_READ);
  ASSERT_NE(ores, nullptr);
  ASSERT_TRUE(ores->success);
  EXPECT_EQ(ores->size, file.size());

  auto rreq = std::make_shared<mavros_msgs::srv::FileRead::Request>();
  rreq->file_path = "/a.bin";
  rreq->offset = 100;
  rreq->size = 600;
  auto rres = call<mavros_msgs::srv::FileRead>("read", rreq);
  ASSERT_NE(rres, nullptr);
  EXPECT_TRUE(rres->success);
  EXPECT_EQ(rres->data, std::vector<uint8_t>(file.begin() + 100, file.begin() + 700));
}

/**
 * File server modelled on ArduPilot: a burst streams up to burst_limit packets
 * (the last one flagged burst_complete) and ends with an EOF NAK at end of file.
 */
struct FakeFile
{
  std::vector<uint8_t> data;
  size_t burst_limit{500};
  bool support_burst{true};
  //! packet indexes (counted over all bursts) the "link" loses
  std::vector<size_t> drop;
  size_t packets{0};

  explicit FakeFile(size_t size)
  : data(size)
  {
    for (size_t i = 0; i < size; i++) {
      data[i] = i * 7 + (i >> 8);
    }
  }

  std::vector<uint8_t> slice(size_t off, size_t len) const
  {
    len = std::min(len, data.size() - std::min(off, data.size()));
    return std::vector<uint8_t>(data.begin() + off, data.begin() + off + len);
  }

  FTPFlowTest::Replies serve(const FTPRequest & req)
  {
    using Replies = FTPFlowTest::Replies;
    const auto hdr = req.header();
    if (hdr.opcode == FTPRequest::kCmdOpenFileRO) {
      return Replies{FTPFlowTest::ack(req, FTPFlowTest::u32(data.size()))};
    }
    if (hdr.opcode == FTPRequest::kCmdReadFile) {
      if (hdr.offset >= data.size()) {
        return Replies{FTPFlowTest::nak(req, FTPRequest::kErrEOF)};
      }
      return Replies{FTPFlowTest::ack(req, slice(hdr.offset, hdr.size))};
    }
    if (hdr.opcode != FTPRequest::kCmdBurstReadFile) {
      return Replies{FTPFlowTest::ack(req)};
    }
    if (!support_burst) {
      return Replies{FTPFlowTest::nak(req, FTPRequest::kErrUnknownCommand)};
    }

    Replies out;
    size_t off = hdr.offset;
    for (size_t i = 0; i <= burst_limit; i++) {
      FTPRequest r;
      if (off >= data.size()) {
        r = FTPFlowTest::nak(req, FTPRequest::kErrEOF);
      } else if (i == burst_limit) {
        break;
      } else {
        r = FTPFlowTest::ack(req, slice(off, hdr.size));
      }
      auto rh = r.header();
      rh.seqNumber = hdr.seqNumber + 1 + i;
      rh.offset = off;
      rh.padding[0] = (i + 1 == burst_limit);
      r.set_header(rh);
      if (std::find(drop.begin(), drop.end(), packets++) == drop.end()) {
        out.push_back(r);
      }
      if (off >= data.size()) {
        break;
      }
      off += rh.size;
    }
    return out;
  }
};

class FTPBurstTest : public FTPFlowTest
{
public:
  bool opened{false};

  std::vector<uint8_t> read(size_t size, size_t offset = 0, bool expect_ok = true)
  {
    if (!opened) {
      auto ores = open("/a.bin", mavros_msgs::srv::FileOpen::Request::MODE_READ);
      EXPECT_TRUE(ores && ores->success);
      opened = true;
    }

    auto rreq = std::make_shared<mavros_msgs::srv::FileRead::Request>();
    rreq->file_path = "/a.bin";
    rreq->offset = offset;
    rreq->size = size;
    auto rres = call<mavros_msgs::srv::FileRead>("read", rreq);
    EXPECT_NE(rres, nullptr);
    if (!rres) {
      return {};
    }
    EXPECT_EQ(rres->success, expect_ok);
    return rres->data;
  }

  size_t sent_count(uint8_t opcode)
  {
    std::lock_guard<std::mutex> lk(fcu_mutex);
    return std::count_if(
      sent.begin(), sent.end(),
      [opcode](const FTPRequest & r) {return r.header().opcode == opcode;});
  }
};

TEST_F(FTPBurstTest, read_to_eof_uses_burst)
{
  FakeFile file(2000);
  file.burst_limit = 3;
  set_fcu([&file](const FTPRequest & req) {return file.serve(req);});

  EXPECT_EQ(read(10000), file.data);
  // 2000 bytes = 9 packets, 3 per burst, then one more burst for the EOF NAK
  EXPECT_EQ(sent_count(FTPRequest::kCmdBurstReadFile), 4U);
  EXPECT_EQ(sent_count(FTPRequest::kCmdReadFile), 0U);
}

TEST_F(FTPBurstTest, read_before_eof_does_not_burst)
{
  FakeFile file(2000);
  set_fcu([&file](const FTPRequest & req) {return file.serve(req);});

  EXPECT_EQ(read(600, 100), file.slice(100, 600));
  EXPECT_EQ(sent_count(FTPRequest::kCmdBurstReadFile), 0U);
}

TEST_F(FTPBurstTest, lost_packet_restarts_burst_at_gap)
{
  FakeFile file(3000);
  // third packet of the first and of the second burst
  file.drop = {2, 16};
  set_fcu([&file](const FTPRequest & req) {return file.serve(req);});

  EXPECT_EQ(read(3000), file.data);
  EXPECT_EQ(sent_count(FTPRequest::kCmdBurstReadFile), 3U);
}

TEST_F(FTPBurstTest, stale_burst_packets_are_ignored)
{
  FakeFile file(2000);
  set_fcu(
    [&file](const FTPRequest & req) {
      auto out = file.serve(req);
      if (req.header().opcode == FTPRequest::kCmdBurstReadFile) {
        // tail of an earlier burst, at a higher offset than this one
        FakeFile old(5000);
        auto stale = ack(req, old.slice(4000, FTPRequest::DATA_MAXSZ));
        auto hdr = stale.header();
        hdr.seqNumber = req.header().seqNumber - 2000;
        hdr.offset = 4000;
        stale.set_header(hdr);
        out.insert(out.begin(), stale);
      }
      return out;
    });

  EXPECT_EQ(read(2000), file.data);
  EXPECT_EQ(sent_count(FTPRequest::kCmdBurstReadFile), 1U);
}

TEST_F(FTPBurstTest, falls_back_without_burst_support)
{
  FakeFile file(1000);
  file.support_burst = false;
  set_fcu([&file](const FTPRequest & req) {return file.serve(req);});

  EXPECT_EQ(read(1000), file.data);
  EXPECT_EQ(read(1000), file.data);
  EXPECT_EQ(sent_count(FTPRequest::kCmdBurstReadFile), 1U);
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
