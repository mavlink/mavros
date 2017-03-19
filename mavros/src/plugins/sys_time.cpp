/**
 * @brief System Time plugin
 * @file sys_time.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016,2017 Vladimir Ermakov, M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Duration.h>

namespace mavros {
namespace std_plugins {
/**
 * Time syncronization status publisher
 *
 * Based on diagnistic_updater::FrequencyStatus
 */
class TimeSyncStatus : public diagnostic_updater::DiagnosticTask
{
public:
	TimeSyncStatus(const std::string &name, size_t win_size) :
		diagnostic_updater::DiagnosticTask(name),
		window_size_(win_size),
		min_freq_(0.01),
		max_freq_(10),
		tolerance_(0.1),
		times_(win_size),
		seq_nums_(win_size),
		last_dt(0),
		dt_sum(0),
		last_ts(0),
		offset(0)
	{
		clear();
	}

	void clear()
	{
		std::lock_guard<std::mutex> lock(mutex);

		ros::Time curtime = ros::Time::now();
		count_ = 0;
		dt_sum = 0;

		for (int i = 0; i < window_size_; i++)
		{
			times_[i] = curtime;
			seq_nums_[i] = count_;
		}

		hist_indx_ = 0;
	}

	void tick(int64_t dt, uint64_t timestamp_ns, int64_t time_offset_ns)
	{
		std::lock_guard<std::mutex> lock(mutex);

		count_++;
		last_dt = dt;
		dt_sum += dt;
		last_ts = timestamp_ns;
		offset = time_offset_ns;
	}

	void set_timestamp(uint64_t timestamp_ns)
	{
		std::lock_guard<std::mutex> lock(mutex);
		last_ts = timestamp_ns;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		std::lock_guard<std::mutex> lock(mutex);

		ros::Time curtime = ros::Time::now();
		int curseq = count_;
		int events = curseq - seq_nums_[hist_indx_];
		double window = (curtime - times_[hist_indx_]).toSec();
		double freq = events / window;
		seq_nums_[hist_indx_] = curseq;
		times_[hist_indx_] = curtime;
		hist_indx_ = (hist_indx_ + 1) % window_size_;

		if (events == 0) {
			stat.summary(2, "No events recorded.");
		}
		else if (freq < min_freq_ * (1 - tolerance_)) {
			stat.summary(1, "Frequency too low.");
		}
		else if (freq > max_freq_ * (1 + tolerance_)) {
			stat.summary(1, "Frequency too high.");
		}
		else {
			stat.summary(0, "Normal");
		}

		stat.addf("Timesyncs since startup", "%d", count_);
		stat.addf("Frequency (Hz)", "%f", freq);
		stat.addf("Last dt (ms)", "%0.6f", last_dt / 1e6);
		stat.addf("Mean dt (ms)", "%0.6f", (count_) ? dt_sum / count_ / 1e6 : 0.0);
		stat.addf("Last system time (s)", "%0.9f", last_ts / 1e9);
		stat.addf("Time offset (s)", "%0.9f", offset / 1e9);
	}

private:
	int count_;
	std::vector<ros::Time> times_;
	std::vector<int> seq_nums_;
	int hist_indx_;
	std::mutex mutex;
	const size_t window_size_;
	const double min_freq_;
	const double max_freq_;
	const double tolerance_;
	int64_t last_dt;
	int64_t dt_sum;
	uint64_t last_ts;
	int64_t offset;
};


/**
 * @brief System time plugin
 */
class SystemTimePlugin : public plugin::PluginBase {
public:
	SystemTimePlugin() : PluginBase(),
		nh("~"),
		dt_diag("Time Sync", 10),
		time_offset_ns(0),
		offset_avg_alpha(0)
	{ }

	using TSM = UAS::timesync_mode;

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		double conn_system_time_d;
		double conn_timesync_d;
		std::string ts_mode_str;

		ros::Duration conn_system_time;
		ros::Duration conn_timesync;

		if (nh.getParam("conn/system_time_rate", conn_system_time_d) && conn_system_time_d != 0.0) {
			conn_system_time = ros::Duration(ros::Rate(conn_system_time_d));
		}

		if (nh.getParam("conn/timesync_rate", conn_timesync_d) && conn_timesync_d != 0.0) {
			conn_timesync = ros::Duration(ros::Rate(conn_timesync_d));
		}

		nh.param<std::string>("time/time_ref_source", time_ref_source, "fcu");
		nh.param<std::string>("time/timesync_mode", ts_mode_str, "MAVLINK");
		nh.param("time/timesync_avg_alpha", offset_avg_alpha, 0.6);
		/*
		 * alpha for exponential moving average. The closer alpha is to 1.0,
		 * the faster the moving average updates in response to new offset samples (more jitter)
		 * We need a significant amount of smoothing , more so for lower message rates like 1Hz
		 */

		// Set timesync mode
		auto ts_mode = utils::timesync_mode_from_str(ts_mode_str);
		m_uas->set_timesync_mode(ts_mode);
		ROS_INFO_STREAM_NAMED("time", "TM: Timesync mode: " << utils::to_string(ts_mode));

		time_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", 10);

		// timer for sending system time messages
		if (!conn_system_time.isZero()) {
			sys_time_timer = nh.createTimer(conn_system_time,
						&SystemTimePlugin::sys_time_cb, this);
			sys_time_timer.start();
		}

		// timer for sending timesync messages
		if (!conn_timesync.isZero() && !(ts_mode == TSM::NONE || ts_mode == TSM::PASSTHROUGH)) {
			// enable timesync diag only if that feature enabled
			UAS_DIAG(m_uas).add(dt_diag);

			timesync_timer = nh.createTimer(conn_timesync,
						&SystemTimePlugin::timesync_cb, this);
			timesync_timer.start();
		}
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&SystemTimePlugin::handle_system_time),
			       make_handler(&SystemTimePlugin::handle_timesync),
		};
	}

private:
	ros::NodeHandle nh;
	ros::Publisher time_ref_pub;

	ros::Timer sys_time_timer;
	ros::Timer timesync_timer;

	TimeSyncStatus dt_diag;

	std::string time_ref_source;
	int64_t time_offset_ns;
	double offset_avg_alpha;

	void handle_system_time(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SYSTEM_TIME &mtime)
	{
		// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
		const bool fcu_time_valid = mtime.time_unix_usec > 1234567890ULL * 1000000;

		if (fcu_time_valid) {
			// continious publish for ntpd
			auto time_unix = boost::make_shared<sensor_msgs::TimeReference>();
			ros::Time time_ref(
						mtime.time_unix_usec / 1000000,		// t_sec
						(mtime.time_unix_usec % 1000000) * 1000);	// t_nsec

			time_unix->header.stamp = ros::Time::now();
			time_unix->time_ref = time_ref;
			time_unix->source = time_ref_source;

			time_ref_pub.publish(time_unix);
		}
		else {
			ROS_WARN_THROTTLE_NAMED(60, "time", "TM: Wrong FCU time.");
		}
	}

	void handle_timesync(const mavlink::mavlink_message_t *msg, mavlink::common::msg::TIMESYNC &tsync)
	{
		uint64_t now_ns = ros::Time::now().toNSec();

		if (tsync.tc1 == 0) {
			send_timesync_msg(now_ns, tsync.ts1);
			return;
		}
		else if (tsync.tc1 > 0) {
			// observation is the offset in nanoseconds
			add_timesync_observation((tsync.ts1 + now_ns - tsync.tc1 * 2) / 2, tsync.tc1);
		}
	}

	void sys_time_cb(const ros::TimerEvent &event)
	{
		// For filesystem only
		uint64_t time_unix_usec = ros::Time::now().toNSec() / 1000;	// nano -> micro

		mavlink::common::msg::SYSTEM_TIME mtime {};
		mtime.time_unix_usec = time_unix_usec;

		UAS_FCU(m_uas)->send_message_ignore_drop(mtime);
	}

	void timesync_cb(const ros::TimerEvent &event)
	{
		auto ts_mode = m_uas->get_timesync_mode();
		if (ts_mode == TSM::MAVLINK) {
			send_timesync_msg(0, ros::Time::now().toNSec());
		} else if (ts_mode == TSM::ONBOARD) {
			// Calculate offset between CLOCK_REALTIME (ros::WallTime) and CLOCK_MONOTONIC
			uint64_t realtime_now_ns = ros::Time::now().toNSec();
			uint64_t monotonic_now_ns = get_monotonic_now();

			add_timesync_observation(realtime_now_ns - monotonic_now_ns, realtime_now_ns);
		}
	}

	void send_timesync_msg(uint64_t tc1, uint64_t ts1)
	{
		mavlink::common::msg::TIMESYNC tsync {};
		tsync.tc1 = tc1;
		tsync.ts1 = ts1;

		UAS_FCU(m_uas)->send_message_ignore_drop(tsync);
	}

	void add_timesync_observation(int64_t offset_ns, uint64_t observation_time)
	{
		int64_t dt = time_offset_ns - offset_ns;

		if (std::abs(dt) > 10000000) {		// 10 millisecond skew
			time_offset_ns = offset_ns;	// hard-set it.
			m_uas->set_time_offset(time_offset_ns);

			dt_diag.clear();
			dt_diag.set_timestamp(observation_time);

			ROS_WARN_THROTTLE_NAMED(10, "time", "TM: Clock skew detected (%0.9f s). "
						"Hard syncing clocks.", dt / 1e9);
		}
		else {
			average_offset(offset_ns);
			dt_diag.tick(dt, observation_time, time_offset_ns);
			m_uas->set_time_offset(time_offset_ns);
		}
	}

	inline void average_offset(int64_t offset_ns)
	{
		time_offset_ns = (offset_avg_alpha * offset_ns) + (1.0 - offset_avg_alpha) * time_offset_ns;
	}

	uint64_t get_monotonic_now(void)
	{
		struct timespec spec;
		clock_gettime(CLOCK_MONOTONIC, &spec);

		return spec.tv_sec * 1000000000ULL + spec.tv_nsec;
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SystemTimePlugin, mavros::plugin::PluginBase)
