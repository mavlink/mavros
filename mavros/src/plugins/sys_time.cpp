/**
 * @brief System Time plugin
 * @file sys_time.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Duration.h>

namespace mavplugin {

/**
 * Time syncronization status publisher
 *
 * Based on diagnistic_updater::FrequencyStatus
 */
class TimeSyncStatus : public diagnostic_updater::DiagnosticTask
{
public:
	TimeSyncStatus(const std::string name, size_t win_size) :
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

	void clear() {
		lock_guard lock(mutex);
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

	void tick(int64_t dt, uint64_t timestamp_us, int64_t time_offset) {
		lock_guard lock(mutex);
		count_++;
		last_dt = dt;
		dt_sum += dt;
		last_ts = timestamp_us;
		offset = time_offset;
	}

	void set_timestamp(uint64_t timestamp_us) {
		lock_guard lock(mutex);
		last_ts = timestamp_us;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);
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

		stat.addf("Events in window", "%d", events);
		stat.addf("Events since startup", "%d", count_);
		stat.addf("Duration of window (s)", "%f", window);
		stat.addf("Actual frequency (Hz)", "%f", freq);
		stat.addf("Last dt (ms)", "%0.6f", last_dt / 1000000.0);
		stat.addf("Mean dt (ms)", "%0.6f", (count_)? dt_sum / count_ / 1000000.0 : 0.0);
		stat.addf("Last system time (s)", "%0.9f", last_ts / 1e9);
		stat.addf("Time offset (s)", "%0.9f", offset / 1e9);
		
	}

private:
	int count_;
	std::vector<ros::Time> times_;
	std::vector<int> seq_nums_;
	int hist_indx_;
	std::recursive_mutex mutex;
	const size_t window_size_;
	const double min_freq_;
	const double max_freq_;
	const double tolerance_;
	int64_t last_dt;
	int64_t dt_sum;
	uint64_t last_ts;
	int64_t offset;
};



class SystemTimePlugin : public MavRosPlugin {
public:
	SystemTimePlugin():
		uas(nullptr),
		dt_diag("Time Sync", 10),
		time_offset_ns(0),
		offset_avg_alpha(0)
	{};

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		double conn_system_time_d;
		double conn_timesync_d;

		uas = &uas_;

		nh.param("conn_system_time", conn_system_time_d, 0.0);
		nh.param("conn_timesync", conn_timesync_d, 0.0);
	
		nh.param<std::string>("frame_id", frame_id, "fcu");
		nh.param<std::string>("time_ref_source", time_ref_source, frame_id);
		nh.param("timesync_avg_alpha", offset_avg_alpha, 0.6);
		/*
		 * alpha for exponential moving average. The closer alpha is to 1.0,
		 * the faster the moving average updates in response to new offset samples (more jitter)
		 * We need a significant amount of smoothing , more so for lower message rates like 1Hz
		 */
		
		diag_updater.add(dt_diag);

		time_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", 10);

		// timer for sending system time messages
		if (conn_system_time_d > 0.0) {
			sys_time_timer = nh.createTimer(ros::Duration(conn_system_time_d),
					&SystemTimePlugin::sys_time_cb, this);
			sys_time_timer.start();
		}

		// timer for sending timesync messages
		if (conn_timesync_d > 0.0) {
			timesync_timer = nh.createTimer(ros::Duration(conn_timesync_d),
					&SystemTimePlugin::timesync_cb, this);
			timesync_timer.start();
		}
	}


	std::string const get_name() const {
		return "SystemTime";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_SYSTEM_TIME, &SystemTimePlugin::handle_system_time),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_TIMESYNC, &SystemTimePlugin::handle_timesync),
		};
	}

private:
	UAS *uas;
	ros::Publisher time_ref_pub;

	ros::Timer sys_time_timer;
	ros::Timer timesync_timer;

	TimeSyncStatus dt_diag;

	std::string frame_id;
	std::string time_ref_source;
	int64_t time_offset_ns;
	double offset_avg_alpha;

	void handle_system_time(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_system_time_t mtime;
		mavlink_msg_system_time_decode(msg, &mtime);

		// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
		const bool fcu_time_valid = mtime.time_unix_usec > 1234567890ULL * 1000000;
		
		if (fcu_time_valid) {
			// continious publish for ntpd
			sensor_msgs::TimeReferencePtr time_unix = boost::make_shared<sensor_msgs::TimeReference>();
			ros::Time time_ref(
					mtime.time_unix_usec / 1000000,			// t_sec
					(mtime.time_unix_usec % 1000000) * 1000);	// t_nsec

			time_unix->source = time_ref_source;
			time_unix->time_ref = time_ref;
			time_unix->header.stamp = ros::Time::now();

			time_ref_pub.publish(time_unix);
		}
		else {
			ROS_WARN_THROTTLE_NAMED(60, "time", "TM: Wrong FCU time.");
		}
		
	}

	void handle_timesync(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		
		mavlink_timesync_t tsync;
		mavlink_msg_timesync_decode(msg, &tsync);

		uint64_t now_ns = ros::Time::now().toNSec();
		
		if(tsync.tc1 == 0) { 
			send_timesync_msg(now_ns, tsync.ts1);
			return;
		}
		else if(tsync.tc1 > 0) { 

			int64_t offset_ns = (tsync.ts1 + now_ns - tsync.tc1*2)/2 ; 
			int64_t dt = time_offset_ns - offset_ns;
			
			if(std::abs(dt) > 10000000) { // 10 millisecond skew
				time_offset_ns = offset_ns ; // hard-set it.
				uas->set_time_offset(time_offset_ns);

				dt_diag.clear();
				dt_diag.set_timestamp(tsync.tc1);

				ROS_WARN_THROTTLE_NAMED(10, "time", "TM: Clock skew detected (%0.9f s). "
					"Hard syncing clocks.", dt / 1e9);
			}
			else {
				average_offset(offset_ns);
				dt_diag.tick(dt, tsync.tc1, time_offset_ns);

				uas->set_time_offset(time_offset_ns);
			} 

		}

	}

	void sys_time_cb(const ros::TimerEvent &event) {
		
		// For filesystem only
		mavlink_message_t msg;

		uint64_t time_unix_usec = ros::Time::now().toNSec() / 1000;  // nano -> micro
		
		mavlink_msg_system_time_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_unix_usec,
				0
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	void timesync_cb(const ros::TimerEvent &event) {
		
		send_timesync_msg( 0, ros::Time::now().toNSec());
		
	}

	void send_timesync_msg(uint64_t tc1, uint64_t ts1) {
		mavlink_message_t msg;

		mavlink_msg_timesync_pack_chan(UAS_PACK_CHAN(uas), &msg,
				tc1,
				ts1
				);
		UAS_FCU(uas)->send_message(&msg);
	}	

	void average_offset(int64_t offset_ns) {
		
		time_offset_ns = (offset_avg_alpha * offset_ns) + (1.0 - offset_avg_alpha) * time_offset_ns;
	
	}


};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SystemTimePlugin, mavplugin::MavRosPlugin)
