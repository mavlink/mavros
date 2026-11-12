//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Tests for sys_time TIMESYNC: requests and replies are addressed, and only the
 * FCU's replies to our own requests feed the offset estimate; the "Time Sync"
 * diagnostic and the timesync status report whether the estimate has converged.
 */

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "test_plugin_helpers.hpp"
#include "../src/plugins/sys_time.cpp"  // NOLINT(build/include)

using mavros::uas::TestUAS;
using mavros::uas::UAS;
using mavlink::common::msg::TIMESYNC;
using diagnostic_msgs::msg::DiagnosticStatus;

static std::string value_of(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return "<missing>";
}

class SysTimeTest : public TestUAS
{
public:
  static constexpr uint8_t FCU_SYSTEM = 1;
  static constexpr uint8_t FCU_COMPONENT = 1;
  // Another component of the FCU's system, e.g. a camera or companion app.
  static constexpr uint8_t OTHER_COMPONENT = 154;

  UAS::SharedPtr uas;
  std::shared_ptr<mavros::std_plugins::SystemTimePlugin> plugin;
  std::shared_ptr<CaptureBuffer> out;
  rclcpp::Node::SharedPtr client;
  rclcpp::Subscription<mavros_msgs::msg::TimesyncStatus>::SharedPtr status_sub;
  std::mutex status_mutex;
  std::vector<mavros_msgs::msg::TimesyncStatus> statuses;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub;
  std::mutex diag_mutex;
  std::optional<DiagnosticStatus> time_sync;  // the latest "Time Sync" status
  std::vector<size_t> time_sync_counts;  // "Time Sync" statuses in each diagnostics array

  void SetUp() override
  {
    uas = create_uas();
    uas->set_tgt(FCU_SYSTEM, FCU_COMPONENT);

    plugin = std::make_shared<mavros::std_plugins::SystemTimePlugin>(uas.get());
    register_plugin(uas, plugin);

    add_node(uas);
    add_node(plugin->get_node());
    client = make_client_node("sys_time_client");
    status_sub = client->create_subscription<mavros_msgs::msg::TimesyncStatus>(
      "/test_mavros_uas/timesync_status", rclcpp::SensorDataQoS(),
      [this](const mavros_msgs::msg::TimesyncStatus::SharedPtr m) {
        std::lock_guard<std::mutex> lock(status_mutex);
        statuses.push_back(*m);
      });
    diag_sub = client->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) {
        const std::string suffix = "Time Sync";
        std::lock_guard<std::mutex> lock(diag_mutex);
        size_t count = 0;
        for (const auto & st : m->status) {
          if (st.name.size() >= suffix.size() &&
          st.name.compare(st.name.size() - suffix.size(), suffix.size(), suffix) == 0)
          {
            time_sync = st;
            count++;
          }
        }
        if (count > 0) {
          time_sync_counts.push_back(count);
        }
      });
    out = capture_sink(uas);
    start_spin();
  }

  void TearDown() override
  {
    stop_spin();
  }

  void inject(const TIMESYNC & m, uint8_t sysid, uint8_t compid)
  {
    auto mmsg = convert_message(m);
    mmsg.sysid = sysid;
    mmsg.compid = compid;
    route(uas, &mmsg, mavconn::Framing::ok);
  }

  static TIMESYNC timesync(int64_t tc1, int64_t ts1, uint8_t tgt_sys = 0, uint8_t tgt_comp = 0)
  {
    TIMESYNC m{};
    m.tc1 = tc1;
    m.ts1 = ts1;
    m.target_system = tgt_sys;
    m.target_component = tgt_comp;
    return m;
  }

  static TIMESYNC decode(const mavros_msgs::msg::Mavlink & rmsg)
  {
    auto m = TestUAS::to_mavlink(rmsg);
    TIMESYNC out{};
    mavlink::MsgMap map(m);
    out.deserialize(map);
    return out;
  }

  std::vector<TIMESYNC> sent()
  {
    std::vector<TIMESYNC> ret;
    for (const auto & m : out->snapshot()) {
      if (m.msgid == TIMESYNC::MSG_ID) {
        ret.push_back(decode(m));
      }
    }
    return ret;
  }

  size_t status_count()
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    return statuses.size();
  }

  // The plugin handles an injected message before inject() returns, and its
  // replies and statuses arrive in that order. So the tests check that a message
  // was ignored by sending a valid one after it: once the valid one's result
  // arrives, a result from the ignored one would already be there before it.

  //! Wait for n TIMESYNC replies (tc1 > 0) sent by the plugin; return those sent.
  std::vector<TIMESYNC> wait_replies(size_t n)
  {
    std::vector<TIMESYNC> replies;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    do {
      replies.clear();
      for (const auto & m : sent()) {
        if (m.tc1 > 0) {
          replies.push_back(m);
        }
      }
      if (replies.size() >= n) {
        break;
      }
      std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return replies;
  }

  //! Wait for n timesync statuses; return the remote timestamp of each published.
  std::vector<uint64_t> wait_status_stamps(size_t n)
  {
    wait_status_count(n);
    std::lock_guard<std::mutex> lock(status_mutex);
    std::vector<uint64_t> stamps;
    for (const auto & st : statuses) {
      stamps.push_back(st.remote_timestamp_ns);
    }
    return stamps;
  }

  //! Wait for a request sent after the one with the given ts1.
  TIMESYNC wait_request_after(int64_t ts1)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      for (const auto & m : sent()) {
        if (m.tc1 == 0 && m.ts1 > ts1) {
          return m;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    ADD_FAILURE() << "no request after ts1 " << ts1;
    return TIMESYNC{};
  }

  //! Start requesting and return the first request sent.
  TIMESYNC first_request()
  {
    plugin->get_node()->set_parameter(rclcpp::Parameter("timesync_rate", 20.0));
    auto rmsg = wait_msgid(*out, TIMESYNC::MSG_ID);
    EXPECT_EQ(rmsg.msgid, TIMESYNC::MSG_ID);
    return decode(rmsg);
  }

  //! Publish sim time until the plugin's clock reads it.
  void set_sim_time(const rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr & pub, int sec)
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock.sec = sec;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (plugin->get_node()->now().seconds() != sec &&
      std::chrono::steady_clock::now() < deadline)
    {
      pub->publish(clock);
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_EQ(plugin->get_node()->now().seconds(), sec);
  }

  //! Wait for a request whose ts1 is below the given value.
  bool wait_request_below(int64_t ts1)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      for (const auto & m : sent()) {
        if (m.tc1 == 0 && m.ts1 < ts1) {
          return true;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  /**
   * Wait for a "Time Sync" status for which done() holds, calling meanwhile()
   * every 10 ms; return the latest status. The updater is forced every 100 ms
   * rather than waited for at 1 Hz, so the diagnostic's window is about 1 s.
   */
  DiagnosticStatus wait_time_sync(
    const std::function<bool(const DiagnosticStatus &)> & done,
    const std::function<void()> & meanwhile = {})
  {
    const auto deadline = std::chrono::steady_clock::now() + 6s;
    auto next_update = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline) {
      if (meanwhile) {
        meanwhile();
      }
      if (std::chrono::steady_clock::now() >= next_update) {
        uas->diagnostic_updater.force_update();
        next_update += 100ms;
      }
      {
        std::lock_guard<std::mutex> lock(diag_mutex);
        if (time_sync && done(*time_sync)) {
          return *time_sync;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    std::lock_guard<std::mutex> lock(diag_mutex);
    ADD_FAILURE() << "no matching Time Sync status; last: " <<
      (time_sync ? time_sync->message : "none");
    return time_sync.value_or(DiagnosticStatus{});
  }

  //! Converge (convergence_window 1, two replies) and return the last request answered.
  TIMESYNC converge()
  {
    auto node = plugin->get_node();
    node->set_parameter(rclcpp::Parameter("convergence_window", 1));
    node->set_parameter(rclcpp::Parameter("max_rtt_sample", 1000));
    auto request = first_request();
    inject(timesync(request.ts1 + 1'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
    auto later = wait_request_after(request.ts1);
    inject(timesync(later.ts1 + 1'000'000'000, later.ts1), FCU_SYSTEM, FCU_COMPONENT);
    wait_status_count(2);
    EXPECT_NE(uas->get_time_offset(), 0);
    return later;
  }

  //! Answer each of our requests not answered yet, as an FCU whose clock is offset_ns behind.
  void answer_requests(std::set<int64_t> & answered, int64_t offset_ns)
  {
    for (const auto & m : sent()) {
      if (m.tc1 == 0 && answered.insert(m.ts1).second) {
        inject(timesync(m.ts1 + offset_ns, m.ts1), FCU_SYSTEM, FCU_COMPONENT);
      }
    }
  }

  void wait_status_count(size_t n)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (status_count() < n && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }
};

TEST_F(SysTimeTest, requests_are_addressed_to_the_fcu)
{
  auto request = first_request();
  EXPECT_EQ(request.tc1, 0);
  EXPECT_GT(request.ts1, 0);
  EXPECT_EQ(request.target_system, FCU_SYSTEM);
  EXPECT_EQ(request.target_component, FCU_COMPONENT);
}

TEST_F(SysTimeTest, replies_are_addressed_to_the_requester)
{
  inject(timesync(0, 1234), FCU_SYSTEM, OTHER_COMPONENT);

  auto replies = wait_replies(1);
  ASSERT_EQ(replies.size(), 1u);
  EXPECT_GT(replies[0].tc1, 0);
  EXPECT_EQ(replies[0].ts1, 1234);
  EXPECT_EQ(replies[0].target_system, FCU_SYSTEM);
  EXPECT_EQ(replies[0].target_component, OTHER_COMPONENT);
}

TEST_F(SysTimeTest, requests_for_another_component_are_not_answered)
{
  inject(timesync(0, 1234, FCU_SYSTEM, OTHER_COMPONENT), FCU_SYSTEM, FCU_COMPONENT);
  // A broadcast request is answered; it is the only reply.
  inject(timesync(0, 5678), FCU_SYSTEM, FCU_COMPONENT);

  auto replies = wait_replies(1);
  ASSERT_EQ(replies.size(), 1u);
  EXPECT_EQ(replies[0].ts1, 5678);
}

TEST_F(SysTimeTest, only_the_fcus_reply_to_our_request_is_used)
{
  auto request = first_request();

  // A reply to someone else's request (another ts1), and our ts1 echoed by
  // another component: neither is ours.
  inject(timesync(1'000'000'000, request.ts1 + 7), FCU_SYSTEM, FCU_COMPONENT);
  inject(timesync(2'000'000'000, request.ts1), FCU_SYSTEM, OTHER_COMPONENT);
  // The FCU's reply, without target fields as ArduPilot sends it.
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);

  auto stamps = wait_status_stamps(1);
  ASSERT_EQ(stamps.size(), 1u);
  EXPECT_EQ(stamps[0], 5'000'000'000u);

  // The same reply again (e.g. a router duplicate) answers nothing pending; the
  // FCU's reply to a later request does.
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  auto later = wait_request_after(request.ts1);
  inject(timesync(7'000'000'000, later.ts1), FCU_SYSTEM, FCU_COMPONENT);

  stamps = wait_status_stamps(2);
  ASSERT_EQ(stamps.size(), 2u);
  EXPECT_EQ(stamps[1], 7'000'000'000u);
}

TEST_F(SysTimeTest, reply_target_fields_are_not_checked)
{
  // PX4 v1.14 - v1.17 leaves the target fields of its replies uninitialised.
  auto request = first_request();
  inject(timesync(5'000'000'000, request.ts1, 0x5a, 0xa5), FCU_SYSTEM, FCU_COMPONENT);

  auto stamps = wait_status_stamps(1);
  ASSERT_EQ(stamps.size(), 1u);
  EXPECT_EQ(stamps[0], 5'000'000'000u);
}

TEST_F(SysTimeTest, target_component_zero_accepts_any_component)
{
  uas->set_tgt(FCU_SYSTEM, 0);
  auto request = first_request();
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  EXPECT_EQ(wait_status_stamps(1), std::vector<uint64_t>{5'000'000'000u});
}

TEST_F(SysTimeTest, the_status_says_when_the_estimate_has_converged)
{
  // The UAS is given the estimate once convergence_window samples have been
  // used: here the second.
  plugin->get_node()->set_parameter(rclcpp::Parameter("convergence_window", 2));
  plugin->get_node()->set_parameter(rclcpp::Parameter("max_rtt_sample", 1000));
  auto request = first_request();
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  wait_status_count(1);
  EXPECT_EQ(uas->get_time_offset(), 0);
  auto later = wait_request_after(request.ts1);
  inject(timesync(5'000'000'000 + (later.ts1 - request.ts1), later.ts1), FCU_SYSTEM, FCU_COMPONENT);
  wait_status_count(2);
  EXPECT_NE(uas->get_time_offset(), 0);

  std::lock_guard<std::mutex> lock(status_mutex);
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_FALSE(statuses[0].converged);
  EXPECT_TRUE(statuses[1].converged);
}

TEST_F(SysTimeTest, before_the_first_sample_the_diagnostic_is_not_converged)
{
  first_request();
  auto status = wait_time_sync([](const DiagnosticStatus & st) {return !st.values.empty();});
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "No events recorded.");
  EXPECT_EQ(value_of(status, "Converged"), "False");
  EXPECT_EQ(value_of(status, "Samples (convergence window)"), "0 / 500");
}

TEST_F(SysTimeTest, the_diagnostic_is_disabled_with_mode_none)
{
  first_request();
  plugin->get_node()->set_parameter(rclcpp::Parameter("timesync_mode", "NONE"));
  auto status = wait_time_sync(
    [](const DiagnosticStatus & st) {return st.message == "Disabled (timesync_mode NONE)";});
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
}

TEST_F(SysTimeTest, a_reply_beyond_the_rtt_limit_is_counted_as_rejected)
{
  auto request = first_request();
  std::this_thread::sleep_for(30ms);  // max_rtt_sample is 10 ms
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  auto status = wait_time_sync(
    [](const DiagnosticStatus & st) {
      return value_of(st, "Rejected samples (high RTT)") == "1";
    });
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "No samples accepted.");
}

TEST_F(SysTimeTest, a_converged_estimate_at_the_request_rate_is_normal)
{
  // 20 Hz, above the rate the diagnostic used to call too high.
  auto node = plugin->get_node();
  node->set_parameter(rclcpp::Parameter("convergence_window", 1));
  node->set_parameter(rclcpp::Parameter("max_rtt_sample", 1000));
  first_request();
  std::set<int64_t> answered;
  auto status = wait_time_sync(
    [](const DiagnosticStatus & st) {return st.message == "Normal";},
    [&]() {answer_requests(answered, 1'000'000'000);});
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(value_of(status, "Converged"), "True");
}

TEST_F(SysTimeTest, a_time_jump_resets_the_filter_and_the_offset_in_use)
{
  auto node = plugin->get_node();
  node->set_parameter(rclcpp::Parameter("convergence_window", 1));
  node->set_parameter(rclcpp::Parameter("max_rtt_sample", 1000));
  node->set_parameter(rclcpp::Parameter("max_consecutive_high_deviation", 0));
  // Below the jump, but above how much RTTs can differ on a loaded machine.
  node->set_parameter(rclcpp::Parameter("max_deviation_sample", 500));
  auto request = first_request();
  const int64_t behind_ns = 1'000'000'000;
  inject(timesync(request.ts1 + behind_ns, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  auto later = wait_request_after(request.ts1);
  inject(timesync(later.ts1 + behind_ns, later.ts1), FCU_SYSTEM, FCU_COMPONENT);
  wait_status_count(2);
  ASSERT_NE(uas->get_time_offset(), 0);

  // The FCU's clock jumps a second: the filter resets, and FCU timestamps are
  // no longer converted with the old estimate.
  auto jump = wait_request_after(later.ts1);
  inject(timesync(jump.ts1 + 2 * behind_ns, jump.ts1), FCU_SYSTEM, FCU_COMPONENT);
  wait_status_count(3);
  EXPECT_EQ(uas->get_time_offset(), 0);
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    ASSERT_EQ(statuses.size(), 3u);
    EXPECT_FALSE(statuses[2].converged);
  }
  auto status = wait_time_sync(
    [](const DiagnosticStatus & st) {
      return value_of(st, "Filter resets (time jumps)") == "1";
    });
  EXPECT_EQ(value_of(status, "Time offset in use (s)"), "0.000000000");
}

TEST_F(SysTimeTest, a_rate_change_keeps_one_diagnostic)
{
  first_request();
  plugin->get_node()->set_parameter(rclcpp::Parameter("timesync_rate", 10.0));
  size_t seen;
  {
    std::lock_guard<std::mutex> lock(diag_mutex);
    seen = time_sync_counts.size();
  }
  // Two diagnostics arrays after the change.
  wait_time_sync(
    [&](const DiagnosticStatus &) {
      return time_sync_counts.size() >= seen + 2;
    });
  std::lock_guard<std::mutex> lock(diag_mutex);
  ASSERT_FALSE(time_sync_counts.empty());
  EXPECT_EQ(time_sync_counts.back(), 1u);
}

TEST_F(SysTimeTest, a_mode_change_drops_the_estimate)
{
  converge();
  // Setting the same mode again keeps it.
  plugin->get_node()->set_parameter(rclcpp::Parameter("timesync_mode", "MAVLINK"));
  EXPECT_NE(uas->get_time_offset(), 0);
  // PASSTHROUGH would add it to FCU timestamps.
  plugin->get_node()->set_parameter(rclcpp::Parameter("timesync_mode", "PASSTHROUGH"));
  EXPECT_EQ(uas->get_time_offset(), 0);
}

TEST_F(SysTimeTest, a_mode_change_clears_the_diagnostics_estimate)
{
  // Without replies after the change, only the reset can update the diagnostic.
  converge();
  plugin->get_node()->set_parameter(rclcpp::Parameter("timesync_mode", "NONE"));
  auto status = wait_time_sync(
    [](const DiagnosticStatus & st) {return st.message == "Disabled (timesync_mode NONE)";});
  EXPECT_EQ(value_of(status, "Converged"), "False");
  EXPECT_EQ(value_of(status, "Samples (convergence window)"), "0 / 1");
  EXPECT_EQ(value_of(status, "Time offset in use (s)"), "0.000000000");
  EXPECT_EQ(value_of(status, "Estimated time offset (s)"), "0.000000000");
}

TEST_F(SysTimeTest, out_of_range_parameters_are_rejected)
{
  // With convergence_window 0 the empty filter would count as converged.
  auto node = plugin->get_node();
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("convergence_window", 3)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("convergence_window", 0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("convergence_window", -5)).successful);
  EXPECT_EQ(node->get_parameter("convergence_window").as_int(), 3);
  first_request();
  auto status = wait_time_sync([](const DiagnosticStatus & st) {return !st.values.empty();});
  EXPECT_EQ(value_of(status, "Samples (convergence window)"), "0 / 3");

  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter("timesync_min_reply_ratio", 1.5)).successful);
  EXPECT_FALSE(
    node->set_parameter(
      rclcpp::Parameter("timesync_min_reply_ratio", std::nan(""))).successful);
  EXPECT_EQ(node->get_parameter("timesync_min_reply_ratio").as_double(), 0.0);
}

TEST_F(SysTimeTest, replies_to_requests_before_a_mode_change_are_ignored)
{
  auto request = first_request();
  auto node = plugin->get_node();
  node->set_parameter(rclcpp::Parameter("timesync_mode", "NONE"));
  node->set_parameter(rclcpp::Parameter("timesync_mode", "MAVLINK"));
  inject(timesync(1'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  // A request sent after the change is answered and used; it is the only one.
  auto later = wait_request_after(request.ts1);
  inject(timesync(5'000'000'000, later.ts1), FCU_SYSTEM, FCU_COMPONENT);
  EXPECT_EQ(wait_status_stamps(1), std::vector<uint64_t>{5'000'000'000u});
}

TEST_F(SysTimeTest, a_convergence_window_change_applies_at_once)
{
  converge();
  const auto offset = uas->get_time_offset();
  auto node = plugin->get_node();
  node->set_parameter(rclcpp::Parameter("convergence_window", 100));
  EXPECT_EQ(uas->get_time_offset(), 0);
  node->set_parameter(rclcpp::Parameter("convergence_window", 1));
  EXPECT_EQ(uas->get_time_offset(), offset);
}

TEST_F(SysTimeTest, too_few_replies_is_a_warning_when_enabled)
{
  plugin->get_node()->set_parameter(rclcpp::Parameter("timesync_min_reply_ratio", 0.5));
  auto request = first_request();
  // One reply; the plugin keeps requesting at 20 Hz.
  inject(timesync(5'000'000'000, request.ts1), FCU_SYSTEM, FCU_COMPONENT);
  auto status = wait_time_sync(
    [](const DiagnosticStatus & st) {return st.message.rfind("Too few replies (", 0) == 0;});
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_NE(value_of(status, "Replies per request"), "<missing>");
}

TEST_F(SysTimeTest, sync_resumes_after_the_clock_steps_back)
{
  // With sim time a SITL restart steps our clock back; an NTP step can too.
  auto clock_pub = client->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  plugin->get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));
  set_sim_time(clock_pub, 100);
  first_request();

  set_sim_time(clock_pub, 50);
  ASSERT_TRUE(wait_request_below(100'000'000'000));
  const int64_t ts1 = 50'000'000'000;
  inject(timesync(5'000'000'000, ts1), FCU_SYSTEM, FCU_COMPONENT);
  EXPECT_EQ(wait_status_stamps(1), std::vector<uint64_t>{5'000'000'000u});
}

using mavros::std_plugins::TimeSyncStatus;

//! The diagnostic's status, a moment after its window started so the rate is in range.
static diagnostic_msgs::msg::DiagnosticStatus run_diagnostic(TimeSyncStatus & diag)
{
  std::this_thread::sleep_for(200ms);
  diagnostic_updater::DiagnosticStatusWrapper stat;
  diag.run(stat);
  return stat;
}

//! The filter's progress: samples used, the convergence window, and its estimate, in use or not.
static TimeSyncStatus::Progress progress(
  uint32_t samples, uint32_t window, bool in_use,
  int64_t offset_ns)
{
  TimeSyncStatus::Progress p;
  p.samples = samples;
  p.convergence_window = window;
  p.in_use = in_use;
  p.offset_in_use_ns = in_use ? offset_ns : 0;
  p.estimated_offset_ns = offset_ns;
  return p;
}

TEST(TimeSyncStatusTest, a_converging_estimate_is_a_warning_until_converged)
{
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  diag.tick(1'000'000, 5'000'000'000, TimeSyncStatus::Sample::ACCEPTED,
    progress(1, 3, false, 0));
  auto status = run_diagnostic(diag);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "Not converged (1/3 samples).");
  EXPECT_EQ(value_of(status, "Converged"), "False");
  EXPECT_EQ(value_of(status, "Time offset in use (s)"), "0.000000000");

  diag.tick(1'000'000, 5'100'000'000, TimeSyncStatus::Sample::ACCEPTED,
    progress(3, 3, true, 42));
  status = run_diagnostic(diag);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Normal");
  EXPECT_EQ(value_of(status, "Converged"), "True");
  EXPECT_EQ(value_of(status, "Samples (convergence window)"), "3 / 3");
  EXPECT_EQ(value_of(status, "Time offset in use (s)"), "0.000000042");
}

TEST(TimeSyncStatusTest, a_window_of_rejected_samples_is_a_warning)
{
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  const auto converged = progress(5, 3, true, 42);  // converged earlier
  diag.filter_reset(4'900'000'000);
  diag.tick(50'000'000, 5'000'000'000, TimeSyncStatus::Sample::HIGH_RTT, converged);
  diag.tick(1'000'000, 5'100'000'000, TimeSyncStatus::Sample::HIGH_DEVIATION, converged);
  auto status = run_diagnostic(diag);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "No samples accepted.");
  EXPECT_EQ(value_of(status, "Rejected samples (high RTT)"), "1");
  EXPECT_EQ(value_of(status, "Rejected samples (high deviation)"), "1");
  EXPECT_EQ(value_of(status, "Filter resets (time jumps)"), "1");
}

TEST(TimeSyncStatusTest, no_samples_is_an_error_and_mode_none_is_disabled)
{
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  auto status = run_diagnostic(diag);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "No events recorded.");

  // With mode NONE nothing is requested: no samples is expected, not an error.
  diag.set_mode("NONE", false);
  status = run_diagnostic(diag);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Disabled (timesync_mode NONE)");
}

TEST(TimeSyncStatusTest, too_high_a_rate_is_above_the_request_rate)
{
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  const auto converged = progress(5, 3, true, 42);
  diag.set_rate(1.0);
  for (int i = 0; i < 3; i++) {  // 15 Hz over the 200 ms window
    diag.tick(1'000'000, 5'000'000'000, TimeSyncStatus::Sample::ACCEPTED, converged);
  }
  auto status = run_diagnostic(diag);
  EXPECT_EQ(status.message, "Frequency too high.");
}

TEST(TimeSyncStatusTest, one_reply_above_a_low_rate_is_not_too_high)
{
  // The window holds whole replies: at 0.5 Hz a 10 s window holds 5 or 6.
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  diag.set_rate(0.5);
  diag.tick(1'000'000, 5'000'000'000, TimeSyncStatus::Sample::ACCEPTED,
    progress(5, 3, true, 42));
  auto status = run_diagnostic(diag);  // 5 Hz over the 200 ms window
  EXPECT_EQ(status.message, "Normal");
}

TEST(TimeSyncStatusTest, a_rate_change_restarts_the_window)
{
  // Replies at the old, higher rate are not too many for the new one.
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  diag.set_rate(20.0);
  const auto converged = progress(5, 3, true, 42);
  for (int i = 0; i < 4; i++) {
    diag.tick(1'000'000, 5'000'000'000, TimeSyncStatus::Sample::ACCEPTED, converged);
  }
  diag.set_rate(1.0);
  diag.tick(1'000'000, 5'000'000'000, TimeSyncStatus::Sample::ACCEPTED, converged);
  auto status = run_diagnostic(diag);
  EXPECT_EQ(status.message, "Normal");
}

TEST(TimeSyncStatusTest, a_reset_clears_what_the_diagnostic_shows_in_use)
{
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  diag.tick(1'000'000, 5'000'000'000, TimeSyncStatus::Sample::ACCEPTED,
    progress(5, 3, true, 42));
  diag.set_progress(progress(0, 3, false, 0));
  auto status = run_diagnostic(diag);
  EXPECT_EQ(status.message, "Not converged (0/3 samples).");
  EXPECT_EQ(value_of(status, "Converged"), "False");
  EXPECT_EQ(value_of(status, "Time offset in use (s)"), "0.000000000");
}

TEST(TimeSyncStatusTest, a_reply_in_flight_is_not_too_few)
{
  // With a ratio of 1 the reply to the window's last request arrives in the next.
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  diag.set_min_reply_ratio(1.0);
  const auto converged = progress(5, 3, true, 42);
  for (int i = 0; i < 4; i++) {
    diag.request_sent();
  }
  for (int i = 0; i < 3; i++) {
    diag.tick(1'000'000, 5'000'000'000, TimeSyncStatus::Sample::ACCEPTED, converged);
  }
  auto status = run_diagnostic(diag);
  EXPECT_EQ(status.message, "Normal");
}

TEST(TimeSyncStatusTest, the_reply_ratio_check_is_off_by_default)
{
  TimeSyncStatus diag("Time Sync", 10);
  diag.set_mode("MAVLINK", true);
  const auto converged = progress(5, 3, true, 42);
  for (int i = 0; i < 4; i++) {
    diag.request_sent();
  }
  diag.tick(1'000'000, 5'000'000'000, TimeSyncStatus::Sample::ACCEPTED, converged);
  auto status = run_diagnostic(diag);
  EXPECT_EQ(status.message, "Normal");
  EXPECT_EQ(value_of(status, "Replies per request"), "0.25");

  diag.set_min_reply_ratio(0.5);
  for (int i = 0; i < 4; i++) {
    diag.request_sent();
  }
  diag.tick(1'000'000, 5'100'000'000, TimeSyncStatus::Sample::ACCEPTED, converged);
  status = run_diagnostic(diag);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "Too few replies (2/8).");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  TestUAS::Init();
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
