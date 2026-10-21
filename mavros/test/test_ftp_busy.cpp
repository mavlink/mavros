//
// mavros
// Copyright 2026 Sean Hickey, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * @brief Test FTP busy responses preserve existing operations and sessions
 * @file test_ftp_busy.cpp
 * @author Sean Hickey
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <memory>
#include <vector>

#include "test_plugin_helpers.hpp"
#include "../src/plugins/ftp.cpp"  // NOLINT(build/include)

namespace mavros
{
namespace std_plugins
{

class FTPTestPeer
{
public:
  static void prepare(FTPPlugin & ftp, bool busy)
  {
    ftp.op_state = busy ? FTPPlugin::OP::READ : FTPPlugin::OP::IDLE;
    ftp.session_file_map["events.json.xz"] = 7;
    ftp.active_session = 7;
    ftp.read_buffer = {1, 2, 3};
    ftp.r_errno = EIO;
    ftp.last_send_seqnr = 41;
  }

  static void expect_unchanged(const FTPPlugin & ftp, bool busy)
  {
    EXPECT_EQ(ftp.op_state, busy ? FTPPlugin::OP::READ : FTPPlugin::OP::IDLE);
    EXPECT_EQ(ftp.session_file_map.size(), 1U);
    EXPECT_EQ(ftp.session_file_map.at("events.json.xz"), 7);
    EXPECT_EQ(ftp.active_session, 7);
    EXPECT_EQ(ftp.read_buffer, (std::vector<uint8_t>{1, 2, 3}));
    EXPECT_EQ(ftp.r_errno, EIO);
    EXPECT_EQ(ftp.last_send_seqnr, 41);  // Sending an FTP request advances this sequence.
  }

  static void read(
    FTPPlugin & ftp,
    const mavros_msgs::srv::FileRead::Response::SharedPtr & response)
  {
    auto request = std::make_shared<mavros_msgs::srv::FileRead::Request>();
    request->file_path = "events.json.xz";
    request->size = 1;
    ftp.read_cb(request, response);
  }

  static void open(
    FTPPlugin & ftp,
    const mavros_msgs::srv::FileOpen::Response::SharedPtr & response)
  {
    auto request = std::make_shared<mavros_msgs::srv::FileOpen::Request>();
    request->file_path = "events.json.xz";
    request->mode = mavros_msgs::srv::FileOpen::Request::MODE_READ;
    ftp.open_cb(request, response);
  }
};

class FTPBusyTest : public uas::TestUAS
{
protected:
  uas::UAS::SharedPtr vehicle;
  std::shared_ptr<FTPPlugin> ftp;

  void SetUp() override
  {
    vehicle = create_uas();
    ftp = std::make_shared<FTPPlugin>(vehicle.get());
  }
};

TEST_F(FTPBusyTest, busy_read_returns_ebusy_without_disturbing_active_operation)
{
  FTPTestPeer::prepare(*ftp, true);
  auto response = std::make_shared<mavros_msgs::srv::FileRead::Response>();
  response->success = true;
  EXPECT_NO_THROW(FTPTestPeer::read(*ftp, response));
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->r_errno, EBUSY);
  FTPTestPeer::expect_unchanged(*ftp, true);
}

TEST_F(FTPBusyTest, duplicate_open_returns_ebusy_without_replacing_session)
{
  FTPTestPeer::prepare(*ftp, false);
  auto response = std::make_shared<mavros_msgs::srv::FileOpen::Response>();
  response->success = true;
  EXPECT_NO_THROW(FTPTestPeer::open(*ftp, response));
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->r_errno, EBUSY);
  FTPTestPeer::expect_unchanged(*ftp, false);
}

}  // namespace std_plugins
}  // namespace mavros

int main(int argc, char ** argv)
{
  mavros::uas::TestUAS::Init();
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
