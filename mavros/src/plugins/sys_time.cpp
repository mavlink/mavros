/**
 * @brief System Time plugin
 * @file sys_time.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016,2017,2021 Vladimir Ermakov, M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <string>
#include <vector>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "sensor_msgs/msg/time_reference.hpp"
#include "mavros_msgs/msg/timesync_status.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT

/**
 * Time syncronization status publisher
 *
 * Based on diagnostic_updater::FrequencyStatus
 */
class TimeSyncStatus : public diagnostic_updater::DiagnosticTask
{
public:
  TimeSyncStatus(const std::string & name, size_t win_size)
  : diagnostic_updater::DiagnosticTask(name),
    times_(win_size),
    seq_nums_(win_size),
    window_size_(win_size),
    min_freq_(0.01),
    max_freq_(10),
    tolerance_(0.1),
    last_rtt(0),
    rtt_sum(0),
    last_remote_ts(0),
    offset(0)
  {
    clear();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex);

    auto curtime = clock.now();
    count_ = 0;
    rtt_sum = 0;

    for (size_t i = 0; i < window_size_; i++) {
      times_[i] = curtime;
      seq_nums_[i] = count_;
    }

    hist_indx_ = 0;
  }

  void tick(int64_t rtt_ns, uint64_t remote_timestamp_ns, int64_t time_offset_ns)
  {
    std::lock_guard<std::mutex> lock(mutex);

    count_++;
    last_rtt = rtt_ns;
    rtt_sum += rtt_ns;
    last_remote_ts = remote_timestamp_ns;
    offset = time_offset_ns;
  }

  void set_timestamp(uint64_t remote_timestamp_ns)
  {
    std::lock_guard<std::mutex> lock(mutex);
    last_remote_ts = remote_timestamp_ns;
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mutex);

    auto curtime = clock.now();
    int curseq = count_;
    int events = curseq - seq_nums_[hist_indx_];
    double window = (curtime - times_[hist_indx_]).seconds();
    double freq = events / window;
    seq_nums_[hist_indx_] = curseq;
    times_[hist_indx_] = curtime;
    hist_indx_ = (hist_indx_ + 1) % window_size_;

    if (events == 0) {
      stat.summary(2, "No events recorded.");
    } else if (freq < min_freq_ * (1 - tolerance_)) {
      stat.summary(1, "Frequency too low.");
    } else if (freq > max_freq_ * (1 + tolerance_)) {
      stat.summary(1, "Frequency too high.");
    } else {
      stat.summary(0, "Normal");
    }

    stat.addf("Timesyncs since startup", "%d", count_);
    stat.addf("Frequency (Hz)", "%f", freq);
    stat.addf("Last RTT (ms)", "%0.6f", last_rtt / 1e6);
    stat.addf("Mean RTT (ms)", "%0.6f", (count_) ? rtt_sum / count_ / 1e6 : 0.0);
    stat.addf("Last remote time (s)", "%0.9f", last_remote_ts / 1e9);
    stat.addf("Estimated time offset (s)", "%0.9f", offset / 1e9);
  }

private:
  rclcpp::Clock clock;
  int count_;
  std::vector<rclcpp::Time> times_;
  std::vector<int> seq_nums_;
  int hist_indx_;
  std::mutex mutex;
  const size_t window_size_;
  const double min_freq_;
  const double max_freq_;
  const double tolerance_;
  int64_t last_rtt;
  int64_t rtt_sum;
  uint64_t last_remote_ts;
  int64_t offset;
};


/**
 * @brief System time plugin
 * @plugin sys_time
 */
class SystemTimePlugin : public plugin::Plugin
{
public:
  using TSM = uas::timesync_mode;

  explicit SystemTimePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "time"),
    dt_diag("Time Sync", 10),
    time_offset(0.0),
    time_skew(0.0),
    sequence(0),
    filter_alpha(0),
    filter_beta(0),
    high_rtt_count(0),
    high_deviation_count(0)
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "time_ref_source", "fcu", [&](const rclcpp::Parameter & p) {
        time_ref_source = p.as_string();
      });

    node_declare_and_watch_parameter(
      "timesync_mode", "MAVLINK", [&](const rclcpp::Parameter & p) {
        auto ts_mode = utils::timesync_mode_from_str(p.as_string());
        uas->set_timesync_mode(ts_mode);
        RCLCPP_INFO_STREAM(get_logger(), "TM: Timesync mode: " << utils::to_string(ts_mode));
      });

    node_declare_and_watch_parameter(
      "system_time_rate", 0.0, [&](const rclcpp::Parameter & p) {
        auto rate_d = p.as_double();

        if (rate_d == 0) {
          if (sys_time_timer) {
            sys_time_timer->cancel();
            sys_time_timer.reset();
          }
        } else {
          rclcpp::WallRate rate(rate_d);

          sys_time_timer =
          node->create_wall_timer(
            rate.period(),
            std::bind(&SystemTimePlugin::sys_time_cb, this));
        }
      });

    node_declare_and_watch_parameter(
      "timesync_rate", 0.0, [&](const rclcpp::Parameter & p) {
        auto rate_d = p.as_double();

        if (rate_d == 0) {
          if (timesync_timer) {
            timesync_timer->cancel();
            timesync_timer.reset();
            uas->diagnostic_updater.removeByName(dt_diag.getName());
          }
        } else {
          rclcpp::WallRate rate(rate_d);

          timesync_timer =
          node->create_wall_timer(
            rate.period(),
            std::bind(&SystemTimePlugin::timesync_cb, this));

          uas->diagnostic_updater.add(dt_diag);
        }
      });

    // Filter gains
    //
    // Alpha : Used to smooth the overall clock offset estimate. Smaller values will lead
    // to a smoother estimate, but track time drift more slowly, introducing a bias
    // in the estimate. Larger values will cause low-amplitude oscillations.
    //
    // Beta : Used to smooth the clock skew estimate. Smaller values will lead to a
    // tighter estimation of the skew (derivative), but will negatively affect how fast the
    // filter reacts to clock skewing (e.g cause by temperature changes to the oscillator).
    // Larger values will cause large-amplitude oscillations.
    node_declare_and_watch_parameter(
      "timesync_alpha_initial", 0.05, [&](const rclcpp::Parameter & p) {
        filter_alpha_initial = p.as_double();
        reset_filter();
      });
    node_declare_and_watch_parameter(
      "timesync_beta_initial", 0.05, [&](const rclcpp::Parameter & p) {
        filter_beta_initial = p.as_double();
        reset_filter();
      });
    node_declare_and_watch_parameter(
      "timesync_alpha_final", 0.003, [&](const rclcpp::Parameter & p) {
        filter_alpha_final = p.as_double();
        reset_filter();
      });
    node_declare_and_watch_parameter(
      "timesync_beta_final", 0.003, [&](const rclcpp::Parameter & p) {
        filter_beta_final = p.as_double();
        reset_filter();
      });

    // Filter gain scheduling
    //
    // The filter interpolates between the initial and final gains while the number of
    // exhanged timesync packets is less than convergence_window. A lower value will
    // allow the timesync to converge faster, but with potentially less accurate initial
    // offset and skew estimates.
    node_declare_and_watch_parameter(
      "convergence_window", 500, [&](const rclcpp::Parameter & p) {
        convergence_window = p.as_int();
      });

    // Outlier rejection and filter reset
    //
    // Samples with round-trip time higher than max_rtt_sample are not used to update the filter.
    // More than max_consecutive_high_rtt number of such events in a row will throw a warning
    // but not reset the filter.
    // Samples whose calculated clock offset is more than max_deviation_sample off from the current
    // estimate are not used to update the filter. More than max_consecutive_high_deviation number
    // of such events in a row will reset the filter. This usually happens only due to a time jump
    // on the remote system.
    node_declare_and_watch_parameter(
      "max_rtt_sample", 10, [&](const rclcpp::Parameter & p) {
        max_rtt_sample = p.as_int();        // in ms
      });
    node_declare_and_watch_parameter(
      "max_deviation_sample", 10, [&](const rclcpp::Parameter & p) {
        max_deviation_sample = p.as_int();  // in ms
      });
    node_declare_and_watch_parameter(
      "max_consecutive_high_rtt", 10, [&](const rclcpp::Parameter & p) {
        max_cons_high_rtt = p.as_int();
      });
    node_declare_and_watch_parameter(
      "max_consecutive_high_deviation", 10, [&](const rclcpp::Parameter & p) {
        max_cons_high_deviation = p.as_int();
      });

    auto sensor_qos = rclcpp::SensorDataQoS();

    time_ref_pub = node->create_publisher<sensor_msgs::msg::TimeReference>(
      "time_reference",
      sensor_qos);
    timesync_status_pub = node->create_publisher<mavros_msgs::msg::TimesyncStatus>(
      "timesync_status", sensor_qos);

    reset_filter();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&SystemTimePlugin::handle_system_time),
      make_handler(&SystemTimePlugin::handle_timesync),
    };
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub;
  rclcpp::Publisher<mavros_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_pub;

  rclcpp::TimerBase::SharedPtr sys_time_timer;
  rclcpp::TimerBase::SharedPtr timesync_timer;

  TimeSyncStatus dt_diag;

  std::string time_ref_source;

  // Estimated statistics
  double time_offset;
  double time_skew;

  // Filter parameters
  uint32_t sequence;
  double filter_alpha;
  double filter_beta;

  // Filter settings
  float filter_alpha_initial;
  float filter_beta_initial;
  float filter_alpha_final;
  float filter_beta_final;
  int convergence_window;

  // Outlier rejection
  int max_rtt_sample;
  int max_deviation_sample;
  int max_cons_high_rtt;
  int max_cons_high_deviation;
  int high_rtt_count;
  int high_deviation_count;

  void handle_system_time(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SYSTEM_TIME & mtime, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
    const bool fcu_time_valid = mtime.time_unix_usec > 1234567890ULL * 1000000;

    if (fcu_time_valid) {
      // continious publish for ntpd
      auto time_unix = sensor_msgs::msg::TimeReference();
      rclcpp::Time time_ref(
        mtime.time_unix_usec / 1000000,                 // t_sec
        (mtime.time_unix_usec % 1000000) * 1000);       // t_nsec

      time_unix.header.stamp = node->now();
      time_unix.time_ref = time_ref;
      time_unix.source = time_ref_source;

      time_ref_pub->publish(time_unix);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000, "TM: Wrong FCU time.");
    }
  }

  void handle_timesync(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TIMESYNC & tsync, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    uint64_t now_ns = node->now().nanoseconds();

    if (tsync.tc1 == 0) {
      send_timesync_msg(now_ns, tsync.ts1);
      return;
    } else if (tsync.tc1 > 0) {
      // Time offset between this system and the remote system is calculated assuming RTT for
      // the timesync packet is roughly equal both ways.
      add_timesync_observation((tsync.ts1 + now_ns - tsync.tc1 * 2) / 2, tsync.ts1, tsync.tc1);
    }
  }

  void sys_time_cb()
  {
    // For filesystem only
    uint64_t time_unix_usec = node->now().nanoseconds() / 1000;     // nano -> micro

    mavlink::common::msg::SYSTEM_TIME mtime {};
    mtime.time_unix_usec = time_unix_usec;

    uas->send_message(mtime);
  }

  void timesync_cb()
  {
    auto ts_mode = uas->get_timesync_mode();
    if (ts_mode == TSM::NONE || ts_mode == TSM::PASSTHROUGH) {
      // NOTE(vooon): nothing to do. keep timer running for possible mode change
    } else if (ts_mode == TSM::MAVLINK) {
      send_timesync_msg(0, node->now().nanoseconds());
    } else if (ts_mode == TSM::ONBOARD) {
      // Calculate offset between CLOCK_REALTIME (ros::WallTime) and CLOCK_MONOTONIC
      uint64_t realtime_now_ns = node->now().nanoseconds();
      uint64_t monotonic_now_ns = get_monotonic_now();

      add_timesync_observation(
        realtime_now_ns - monotonic_now_ns, realtime_now_ns,
        monotonic_now_ns);
    }
  }

  void send_timesync_msg(uint64_t tc1, uint64_t ts1)
  {
    mavlink::common::msg::TIMESYNC tsync {};
    tsync.tc1 = tc1;
    tsync.ts1 = ts1;

    uas->send_message(tsync);
  }

  void add_timesync_observation(int64_t offset_ns, uint64_t local_time_ns, uint64_t remote_time_ns)
  {
    uint64_t now_ns = node->now().nanoseconds();

    // Calculate the round trip time (RTT) it took the timesync
    // packet to bounce back to us from remote system
    uint64_t rtt_ns = now_ns - local_time_ns;

    // Calculate the difference of this sample from the current estimate
    uint64_t deviation = llabs(int64_t(time_offset) - offset_ns);

    if (rtt_ns < max_rtt_sample * 1000000ULL) {                 // Only use samples with low RTT
      if (sync_converged() && (deviation > max_deviation_sample * 1000000ULL)) {
        // Increment the counter if we have a good estimate and are
        // getting samples far from the estimate
        high_deviation_count++;

        // We reset the filter if we received consecutive samples
        // which violate our present estimate.
        // This is most likely due to a time jump on the offboard system.
        if (high_deviation_count > max_cons_high_deviation) {
          RCLCPP_ERROR(get_logger(), "TM: Time jump detected. Resetting time synchroniser.");

          // Reset the filter
          reset_filter();

          // Reset diagnostics
          dt_diag.clear();
          dt_diag.set_timestamp(remote_time_ns);
        }
      } else {
        // Filter gain scheduling
        if (!sync_converged()) {
          // Interpolate with a sigmoid function
          float progress = static_cast<float>(sequence) / convergence_window;
          float p = 1.0f - expf(0.5f * (1.0f - 1.0f / (1.0f - progress)));
          filter_alpha = p * filter_alpha_final + (1.0f - p) * filter_alpha_initial;
          filter_beta = p * filter_beta_final + (1.0f - p) * filter_beta_initial;
        } else {
          filter_alpha = filter_alpha_final;
          filter_beta = filter_beta_final;
        }

        // Perform filter update
        add_sample(offset_ns);

        // Save time offset for other components to use
        uas->set_time_offset(sync_converged() ? time_offset : 0);

        // Increment sequence counter after filter update
        sequence++;

        // Reset high deviation count after filter update
        high_deviation_count = 0;

        // Reset high RTT count after filter update
        high_rtt_count = 0;
      }
    } else {
      // Increment counter if round trip time is too high for accurate timesync
      high_rtt_count++;

      if (high_rtt_count > max_cons_high_rtt) {
        // Issue a warning to the user if the RTT is constantly high
        RCLCPP_WARN(get_logger(), "TM: RTT too high for timesync: %0.2f ms.", rtt_ns / 1000000.0);

        // Reset counter
        high_rtt_count = 0;
      }
    }

    // Publish timesync status
    auto timesync_status = mavros_msgs::msg::TimesyncStatus();
    timesync_status.header.stamp = node->now();
    timesync_status.remote_timestamp_ns = remote_time_ns;
    timesync_status.observed_offset_ns = offset_ns;
    timesync_status.estimated_offset_ns = time_offset;
    timesync_status.round_trip_time_ms = static_cast<float>(rtt_ns / 1000000.0);

    timesync_status_pub->publish(timesync_status);

    // Update diagnostics
    dt_diag.tick(rtt_ns, remote_time_ns, time_offset);
  }

  void add_sample(int64_t offset_ns)
  {
    /* Online exponential smoothing filter. The derivative of the estimate is also
     * estimated in order to produce an estimate without steady state lag:
     * https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
     */

    double time_offset_prev = time_offset;

    if (sequence == 0) {                                // First offset sample
      time_offset = offset_ns;
    } else {
      // Update the clock offset estimate
      time_offset = filter_alpha * offset_ns + (1.0 - filter_alpha) * (time_offset + time_skew);

      // Update the clock skew estimate
      time_skew = filter_beta * (time_offset - time_offset_prev) + (1.0 - filter_beta) * time_skew;
    }
  }

  void reset_filter()
  {
    // Do a full reset of all statistics and parameters
    sequence = 0;
    time_offset = 0.0;
    time_skew = 0.0;
    filter_alpha = filter_alpha_initial;
    filter_beta = filter_beta_initial;
    high_deviation_count = 0;
    high_rtt_count = 0;
  }

  inline bool sync_converged()
  {
    return sequence >= uint32_t(convergence_window);
  }

  uint64_t get_monotonic_now(void)
  {
    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC, &spec);

    return spec.tv_sec * 1000000000ULL + spec.tv_nsec;
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SystemTimePlugin)
