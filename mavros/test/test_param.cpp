//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Tests for the parameter plugin ROS API (set/get/pull) plus the pure
 * `Parameter` MAVLink <-> rclcpp conversion logic.
 *
 * The plugin is compiled into this TU by including its source, so both the
 * private `Parameter` class and the full service surface are available.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <thread>

#include "test_plugin_helpers.hpp"
#include "../src/plugins/param.cpp"  // NOLINT(build/include)

using namespace std::chrono_literals;  // NOLINT

using mavros::utils::enum_value;
using mavlink::common::MAV_PARAM_TYPE;
using mavlink::common::msg::PARAM_REQUEST_LIST;
using mavlink::common::msg::PARAM_REQUEST_READ;
using mavlink::common::msg::PARAM_SET;
using mavlink::common::msg::PARAM_VALUE;
using mavros_msgs::srv::ParamPull;
using mavros_msgs::srv::ParamSetV2;

namespace mavros
{
namespace uas
{

// Encode an integer into the MAVLink param union (which is carried in the
// float `param_value` field).
float union_float_from_int(int type, int64_t v)
{
  mavlink::mavlink_param_union_t uv{};
  switch (type) {
    case enum_value(MAV_PARAM_TYPE::INT8): uv.param_int8 = static_cast<int8_t>(v); break;
    case enum_value(MAV_PARAM_TYPE::UINT8): uv.param_uint8 = static_cast<uint8_t>(v); break;
    case enum_value(MAV_PARAM_TYPE::INT16): uv.param_int16 = static_cast<int16_t>(v); break;
    case enum_value(MAV_PARAM_TYPE::UINT16): uv.param_uint16 = static_cast<uint16_t>(v); break;
    case enum_value(MAV_PARAM_TYPE::INT32): uv.param_int32 = static_cast<int32_t>(v); break;
    case enum_value(MAV_PARAM_TYPE::UINT32): uv.param_uint32 = static_cast<uint32_t>(v); break;
    default: uv.param_float = static_cast<float>(v); break;
  }
  return uv.param_float;
}

// Pure conversion tests need no ROS stack.
class ParameterTest : public ::testing::Test
{
};

class ParamFlowTest : public TestUAS
{
public:
  UAS::SharedPtr uas;
  std::shared_ptr<mavros::std_plugins::ParamPlugin> plugin;
  std::shared_ptr<std::vector<mavros_msgs::msg::Mavlink>> out;
  rclcpp::Node::SharedPtr client;
  std::mutex requests_mutex;
  std::condition_variable requests_changed;

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(1, 1);

    plugin = std::make_shared<mavros::std_plugins::ParamPlugin>(uas.get());
    register_plugin(uas, plugin);

    // Deterministic fast timeouts so failure paths do not hang.
    auto pn = plugin->get_node();
    pn->set_parameter(rclcpp::Parameter("param_set_timeout", 0.1));
    pn->set_parameter(rclcpp::Parameter("param_list_timeout", 0.5));
    pn->set_parameter(rclcpp::Parameter("param_retries", 1));

    add_node(uas);
    add_node(plugin->get_node());
    client = make_client_node("param_client");
    out = std::make_shared<std::vector<mavros_msgs::msg::Mavlink>>();
    sink_subs_.push_back(
      uas->create_subscription<mavros_msgs::msg::Mavlink>(
        "/test_mavros_uas/mavlink_sink", rclcpp::QoS(1000).best_effort(),
        [this](const mavros_msgs::msg::Mavlink::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(requests_mutex);
          out->push_back(*msg);
          requests_changed.notify_all();
        }));
    start_spin();
  }

  void TearDown() override
  {
    stop_spin();
  }

  //! Wait for request messages and return a synchronized snapshot of that type.
  std::vector<mavros_msgs::msg::Mavlink> wait_requests(
    uint32_t msgid, size_t count, std::chrono::milliseconds timeout = 5s)
  {
    std::unique_lock<std::mutex> lock(requests_mutex);
    auto matches = [msgid](const auto & msg) {return msg.msgid == msgid;};
    requests_changed.wait_for(
      lock, timeout, [&]() {
        const auto received = std::count_if(out->begin(), out->end(), matches);
        return static_cast<size_t>(received) >= count;
      });
    std::vector<mavros_msgs::msg::Mavlink> result;
    std::copy_if(out->begin(), out->end(), std::back_inserter(result), matches);
    return result;
  }

  void inject(const mavlink::mavlink_message_t & mmsg)
  {
    route(uas, &mmsg, mavconn::Framing::ok);
    std::this_thread::sleep_for(10ms);
  }

  mavlink::mavlink_message_t fcu_param_value(
    const std::string & id, int type, float value, uint16_t index, uint16_t count,
    uint8_t sysid = 1)
  {
    PARAM_VALUE pv{};
    mavlink::set_string(pv.param_id, id.c_str());
    pv.param_type = type;
    pv.param_value = value;
    pv.param_index = index;
    pv.param_count = count;
    auto mmsg = convert_message(pv);
    mmsg.sysid = sysid;
    mmsg.compid = 1;
    mmsg.msgid = PARAM_VALUE::MSG_ID;
    return mmsg;
  }

  template<class M>
  M decode(const mavros_msgs::msg::Mavlink & rmsg)
  {
    auto m = TestUAS::to_mavlink(rmsg);
    M out{};
    mavlink::MsgMap map(m);
    out.deserialize(map);
    return out;
  }

  size_t count_msgid(uint32_t msgid) const
  {
    size_t n = 0;
    for (const auto & m : *out) {
      if (m.msgid == msgid) {
        n++;
      }
    }
    return n;
  }
};

// Pure conversion: MAVLink PARAM_VALUE -> rclcpp value for every type.
TEST_F(ParameterTest, parameter_set_value_types)
{
  mavros::std_plugins::Parameter p("TEST");

  auto check = [&](int mtype, int64_t raw, rclcpp::ParameterType rtype, int64_t expected) {
      PARAM_VALUE pv{};
      pv.param_type = mtype;
      pv.param_value = union_float_from_int(mtype, raw);
      p.set_value(pv);
      EXPECT_EQ(p.param_value.get_type(), rtype);
      EXPECT_EQ(p.param_value.get<int64_t>(), expected);
    };

  check(enum_value(MAV_PARAM_TYPE::INT8), -5, rclcpp::PARAMETER_INTEGER, -5);
  check(enum_value(MAV_PARAM_TYPE::UINT8), 200, rclcpp::PARAMETER_INTEGER, 200);
  check(enum_value(MAV_PARAM_TYPE::INT16), -1000, rclcpp::PARAMETER_INTEGER, -1000);
  check(enum_value(MAV_PARAM_TYPE::UINT16), 40000, rclcpp::PARAMETER_INTEGER, 40000);
  check(enum_value(MAV_PARAM_TYPE::INT32), 123456, rclcpp::PARAMETER_INTEGER, 123456);
  check(enum_value(MAV_PARAM_TYPE::UINT32), 2000000000, rclcpp::PARAMETER_INTEGER, 2000000000);

  // real32
  PARAM_VALUE pf{};
  pf.param_type = enum_value(MAV_PARAM_TYPE::REAL32);
  pf.param_value = 3.5f;
  p.set_value(pf);
  EXPECT_EQ(p.param_value.get_type(), rclcpp::PARAMETER_DOUBLE);
  EXPECT_DOUBLE_EQ(p.param_value.get<double>(), 3.5);

  // unsupported
  PARAM_VALUE pu{};
  pu.param_type = 99;
  p.set_value(pu);
  EXPECT_EQ(p.param_value.get_type(), rclcpp::PARAMETER_NOT_SET);
}

// Pure conversion: rclcpp value -> PARAM_SET.
TEST_F(ParameterTest, parameter_to_param_set)
{
  auto check = [](const rclcpp::ParameterValue & v, int expected_type, float expected_union) {
      mavros::std_plugins::Parameter p("TEST", 0, 0, v);
      auto ps = p.to_param_set();
      EXPECT_EQ(mavlink::to_string(ps.param_id), "TEST");
      EXPECT_EQ(ps.param_type, expected_type);
      EXPECT_EQ(ps.param_value, expected_union);
    };

  check(
    rclcpp::ParameterValue(true), enum_value(MAV_PARAM_TYPE::UINT8),
    union_float_from_int(enum_value(MAV_PARAM_TYPE::UINT8), 1));
  check(
    rclcpp::ParameterValue(42), enum_value(MAV_PARAM_TYPE::INT32),
    union_float_from_int(enum_value(MAV_PARAM_TYPE::INT32), 42));
  check(rclcpp::ParameterValue(3.5), enum_value(MAV_PARAM_TYPE::REAL32), 3.5f);
}

// Pure conversion: excluded parameter ids.
TEST_F(ParameterTest, parameter_exclude_ids)
{
  EXPECT_TRUE(mavros::std_plugins::Parameter::check_exclude_param_id("CMD_TOTAL"));
  EXPECT_TRUE(mavros::std_plugins::Parameter::check_exclude_param_id("_HASH_CHECK"));
  EXPECT_FALSE(mavros::std_plugins::Parameter::check_exclude_param_id("MY_PARAM"));
}

// Set flow: force-set an integer param, FCU echoes the value.
TEST_F(ParamFlowTest, set_success_int)
{
  auto set_client = client->create_client<ParamSetV2>("/test_mavros_uas/param/set");
  ASSERT_TRUE(set_client->wait_for_service(5s));

  auto req = std::make_shared<ParamSetV2::Request>();
  req->param_id = "TEST_I";
  req->force_set = true;
  req->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  req->value.integer_value = 42;

  std::promise<ParamSetV2::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(set_client, req, 10s));});

  auto ps = wait_msgid(*out, PARAM_SET::MSG_ID);
  ASSERT_EQ(ps.msgid, PARAM_SET::MSG_ID);
  auto pset = decode<PARAM_SET>(ps);
  EXPECT_EQ(mavlink::to_string(pset.param_id), "TEST_I");
  EXPECT_EQ(pset.param_type, enum_value(MAV_PARAM_TYPE::INT32));
  EXPECT_EQ(pset.param_value, union_float_from_int(enum_value(MAV_PARAM_TYPE::INT32), 42));
  EXPECT_EQ(pset.target_system, 1);

  inject(
    fcu_param_value(
      "TEST_I", enum_value(MAV_PARAM_TYPE::INT32),
      union_float_from_int(enum_value(MAV_PARAM_TYPE::INT32), 42), 0, 1));

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->value.integer_value, 42);
}

// Set flow: force-set a double param, FCU echoes the value.
TEST_F(ParamFlowTest, set_success_double)
{
  auto set_client = client->create_client<ParamSetV2>("/test_mavros_uas/param/set");
  ASSERT_TRUE(set_client->wait_for_service(5s));

  auto req = std::make_shared<ParamSetV2::Request>();
  req->param_id = "TEST_F";
  req->force_set = true;
  req->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  req->value.double_value = 3.5;

  std::promise<ParamSetV2::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(set_client, req, 10s));});

  EXPECT_EQ(wait_msgid(*out, PARAM_SET::MSG_ID).msgid, PARAM_SET::MSG_ID);

  inject(fcu_param_value("TEST_F", enum_value(MAV_PARAM_TYPE::REAL32), 3.5f, 0, 1));

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_DOUBLE_EQ(res->value.double_value, 3.5);
}

// Pull flow: FCU streams N params, pull reports them; get_parameters serves cache.
TEST_F(ParamFlowTest, pull_and_get)
{
  auto pull_client = client->create_client<ParamPull>("/test_mavros_uas/param/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));

  auto req = std::make_shared<ParamPull::Request>();
  std::promise<ParamPull::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});

  EXPECT_EQ(wait_msgid(*out, PARAM_REQUEST_LIST::MSG_ID).msgid, PARAM_REQUEST_LIST::MSG_ID);

  inject(fcu_param_value("A_0", enum_value(MAV_PARAM_TYPE::INT32),
        union_float_from_int(enum_value(MAV_PARAM_TYPE::INT32), 10), 0, 2));
  inject(fcu_param_value("A_1", enum_value(MAV_PARAM_TYPE::INT32),
        union_float_from_int(enum_value(MAV_PARAM_TYPE::INT32), 20), 1, 2));

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->param_received, 2u);

  // get_parameters serves the cached values
  auto get_parameters = client->create_client<rcl_interfaces::srv::GetParameters>(
    "/test_mavros_uas/param/get_parameters");
  ASSERT_TRUE(get_parameters->wait_for_service(5s));
  auto greq = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  greq->names = {"A_0", "A_1", "MISSING"};
  auto gres = call_service(get_parameters, greq, 5s);
  ASSERT_NE(gres, nullptr);
  ASSERT_EQ(gres->values.size(), 3u);
  EXPECT_EQ(gres->values[0].integer_value, 10);
  EXPECT_EQ(gres->values[1].integer_value, 20);
  EXPECT_EQ(gres->values[2].type, rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
}

// Unsolicited replies must not amplify reads or prevent completing a pull.
TEST_F(ParamFlowTest, pull_retry_duplicate_burst_and_out_of_order)
{
  plugin->get_node()->set_parameter(rclcpp::Parameter("param_set_timeout", 0.4));
  plugin->get_node()->set_parameter(rclcpp::Parameter("param_list_timeout", 3.0));
  auto pull_client = client->create_client<ParamPull>("/test_mavros_uas/param/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));
  auto future = pull_client->async_send_request(std::make_shared<ParamPull::Request>());
  ASSERT_EQ(wait_requests(PARAM_REQUEST_LIST::MSG_ID, 1).size(), 1u);

  inject(fcu_param_value("A_0", enum_value(MAV_PARAM_TYPE::REAL32), 10.0f, 0, 4));
  auto reads = wait_requests(PARAM_REQUEST_READ::MSG_ID, 1);
  ASSERT_EQ(reads.size(), 1u);
  EXPECT_EQ(decode<PARAM_REQUEST_READ>(reads.front()).param_index, 1);

  for (int i = 0; i < 8; i++) {
    inject(fcu_param_value("A_0", enum_value(MAV_PARAM_TYPE::REAL32), 11.0f, 0, 4));
  }
  inject(fcu_param_value("A_3", enum_value(MAV_PARAM_TYPE::REAL32), 40.0f, 3, 4));
  EXPECT_EQ(wait_requests(PARAM_REQUEST_READ::MSG_ID, 0).size(), 1u);

  inject(fcu_param_value("A_1", enum_value(MAV_PARAM_TYPE::REAL32), 20.0f, 1, 4));
  reads = wait_requests(PARAM_REQUEST_READ::MSG_ID, 2);
  ASSERT_EQ(reads.size(), 2u);
  EXPECT_EQ(decode<PARAM_REQUEST_READ>(reads.back()).param_index, 2);
  inject(fcu_param_value("A_2", enum_value(MAV_PARAM_TYPE::REAL32), 30.0f, 2, 4));

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  auto res = future.get();
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->param_received, 4u);
}

// Retry exhaustion must advance to the next missing index during a duplicate stream.
TEST_F(ParamFlowTest, pull_retry_duplicates_do_not_postpone_exhaustion)
{
  plugin->get_node()->set_parameter(rclcpp::Parameter("param_list_timeout", 3.0));
  auto pull_client = client->create_client<ParamPull>("/test_mavros_uas/param/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));
  auto future = pull_client->async_send_request(std::make_shared<ParamPull::Request>());
  ASSERT_EQ(wait_requests(PARAM_REQUEST_LIST::MSG_ID, 1).size(), 1u);

  inject(fcu_param_value("A_0", enum_value(MAV_PARAM_TYPE::REAL32), 10.0f, 0, 3));
  auto reads = wait_requests(PARAM_REQUEST_READ::MSG_ID, 1);
  ASSERT_EQ(reads.size(), 1u);
  EXPECT_EQ(decode<PARAM_REQUEST_READ>(reads.front()).param_index, 1);

  // Keep duplicates closer together than the 100 ms retry timeout.
  bool requested_next = false;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!requested_next && std::chrono::steady_clock::now() < deadline) {
    inject(fcu_param_value("A_0", enum_value(MAV_PARAM_TYPE::REAL32), 10.0f, 0, 3));
    reads = wait_requests(PARAM_REQUEST_READ::MSG_ID, 0);
    requested_next = std::any_of(
      reads.begin(), reads.end(), [this](const auto & msg) {
        return decode<PARAM_REQUEST_READ>(msg).param_index == 2;
      });
  }
  EXPECT_TRUE(requested_next);
  EXPECT_EQ(reads.size(), 2u);

  // A late reply for the exhausted index still belongs in the cache.
  inject(fcu_param_value("A_1", enum_value(MAV_PARAM_TYPE::REAL32), 20.0f, 1, 3));
  inject(fcu_param_value("A_2", enum_value(MAV_PARAM_TYPE::REAL32), 30.0f, 2, 3));
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);
}

// Set timeout: no FCU echo, retries exhausted, report failure.
TEST_F(ParamFlowTest, set_timeout)
{
  auto set_client = client->create_client<ParamSetV2>("/test_mavros_uas/param/set");
  ASSERT_TRUE(set_client->wait_for_service(5s));

  auto req = std::make_shared<ParamSetV2::Request>();
  req->param_id = "TEST_I";
  req->force_set = true;
  req->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  req->value.integer_value = 7;

  std::promise<ParamSetV2::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(set_client, req, 10s));});

  EXPECT_EQ(wait_msgid(*out, PARAM_SET::MSG_ID).msgid, PARAM_SET::MSG_ID);

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
  // initial send + param_retries(1) resend
  EXPECT_GE(count_msgid(PARAM_SET::MSG_ID), 2u);
}

// Pull timeout: FCU never responds.
TEST_F(ParamFlowTest, pull_timeout)
{
  auto pull_client = client->create_client<ParamPull>("/test_mavros_uas/param/pull");
  ASSERT_TRUE(pull_client->wait_for_service(5s));

  auto req = std::make_shared<ParamPull::Request>();
  std::promise<ParamPull::Response::SharedPtr> prom;
  std::thread svc([&]() {prom.set_value(call_service(pull_client, req, 10s));});

  EXPECT_EQ(wait_msgid(*out, PARAM_REQUEST_LIST::MSG_ID).msgid, PARAM_REQUEST_LIST::MSG_ID);

  auto res = prom.get_future().get();
  svc.join();

  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
}

// Excluded parameter rejected without force_set.
TEST_F(ParamFlowTest, set_excluded_param)
{
  auto set_client = client->create_client<ParamSetV2>("/test_mavros_uas/param/set");
  ASSERT_TRUE(set_client->wait_for_service(5s));

  auto req = std::make_shared<ParamSetV2::Request>();
  req->param_id = "CMD_TOTAL";
  req->force_set = false;
  req->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  req->value.integer_value = 1;

  auto res = call_service(set_client, req, 5s);
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
}

// Unknown parameter rejected without force_set.
TEST_F(ParamFlowTest, set_unknown_param)
{
  auto set_client = client->create_client<ParamSetV2>("/test_mavros_uas/param/set");
  ASSERT_TRUE(set_client->wait_for_service(5s));

  auto req = std::make_shared<ParamSetV2::Request>();
  req->param_id = "NOT_KNOWN";
  req->force_set = false;
  req->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  req->value.integer_value = 1;

  auto res = call_service(set_client, req, 5s);
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->success);
}

}   // namespace uas
}   // namespace mavros

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  mavros::uas::TestUAS::Init();
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
