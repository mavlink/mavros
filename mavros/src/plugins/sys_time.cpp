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

class SystemTimePlugin : public MavRosPlugin {
public:
	SystemTimePlugin():
		uas(nullptr),
		time_offset_ms(0)
	{};

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		double conn_system_time_d;

		uas = &uas_;

		nh.param("conn_system_time", conn_system_time_d, 0.0);
		nh.param<std::string>("frame_id", frame_id, "fcu");
		nh.param<std::string>("time_ref_source", time_ref_source, frame_id);

		time_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", 10);
		time_offset_pub = nh.advertise<std_msgs::Duration>("time_offset", 10);

		// timer for sending time sync messages
		if (conn_system_time_d > 0.0) {
			sys_time_timer = nh.createTimer(ros::Duration(conn_system_time_d),
					&SystemTimePlugin::sys_time_cb, this);
			sys_time_timer.start();
		}
	}


	std::string const get_name() const {
		return "SystemTime";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_SYSTEM_TIME, &SystemTimePlugin::handle_system_time),
		};
	}

private:
	UAS *uas;
	ros::Publisher time_ref_pub;
	ros::Publisher time_offset_pub;
	ros::Timer sys_time_timer;

	std::string frame_id;
	std::string time_ref_source;
	uint64_t time_offset_ms;

	void handle_system_time(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_system_time_t mtime;
		mavlink_msg_system_time_decode(msg, &mtime);

		uint64_t now_ms = ros::Time::now().toNSec() / 1000000;

		// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
		const bool fcu_time_valid = mtime.time_unix_usec > 1234567890L * 1000000;
		const bool ros_time_valid = now_ms > 1234567890L * 1000;

		int64_t offset_ms = now_ms - mtime.time_boot_ms;
		int64_t dt = offset_ms - time_offset_ms;
		if (std::abs(dt) > 2000 /* milliseconds */) {
			ROS_WARN_THROTTLE_NAMED(10, "time", "TM: Large clock skew detected (%0.3f s). "
					"Resyncing clocks.", dt / 1000.0);
			time_offset_ms = offset_ms;
		}
		else {
			time_offset_ms = (time_offset_ms + offset_ms) / 2;
		}

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
			ROS_WARN_THROTTLE_NAMED(60, "time", "TM: Wrong GPS time.");
		}

		// offset publisher
		std_msgs::DurationPtr offset = boost::make_shared<std_msgs::Duration>();
		ros::Duration time_ref(
				time_offset_ms / 1000,			// t_sec
				(time_offset_ms % 1000) * 1000000);	// t_nsec

		offset->data = time_ref;

		uas->set_time_offset(time_offset_ms);
		time_offset_pub.publish(offset);
	}

	void sys_time_cb(const ros::TimerEvent &event) {
		mavlink_message_t msg;

		uint64_t time_unix_usec = ros::Time::now().toNSec() / 1000;  // nano -> micro

		mavlink_msg_system_time_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_unix_usec,
				0
				);
		UAS_FCU(uas)->send_message(&msg);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SystemTimePlugin, mavplugin::MavRosPlugin)
