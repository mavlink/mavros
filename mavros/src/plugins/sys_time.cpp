/**
 * @brief System Time plugin
 * @file sys_time.cpp
 * @author M.H.Kabir mhkabir98@gmail.com>
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

namespace mavplugin {

class SystemTimePlugin : public MavRosPlugin {
public:
	SystemTimePlugin():
	uas(nullptr)	
	 {
		uas = &uas_;		
	
		bool companion_reboot = true;
		uint64_t time_offset;
		bool fcu_unix_valid;
		bool ros_unix_valid;
		uint64_t time_unix_usec;

		//timer for sending time sync messages
		if (conn_system_time_d > 0.0) {
			sys_time_timer = nh.createTimer(ros::Duration(conn_system_time_d),
					&SystemStatusPlugin::sys_time_cb, this);
			sys_time_timer.start();
		}

	};

	void initialize(UAS &uas,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		nh.param<std::string>("frame_id", frame_id, "fcu"); 
		nh.param<std::string>("time_ref_source", time_ref_source, frame_id);		

		nh.param("conn_system_time", conn_system_time_d, 0.0);

		time_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", 10);
		time_offset_pub = nh.advertise<sensor_msgs::TimeReference>("time_offset", 10);
	};

	
	std::string const get_name() const {
		return "SystemTime";
	};

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_SYSTEM_TIME, &SystemTimePlugin::handle_system_time);
		};
	}

private:
	ros::Timer sys_time_timer;	

	ros::Publisher time_ref_pub;
	ros::Publisher time_offset_pub;

	void handle_system_time(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		
		mavlink_system_time_t mtime;
		mavlink_msg_system_time_decode(msg, &mtime);

		time_unix_usec = ros::Time::now().toNSec()/1000; 

		int64_t dt = ((time_unix_usec/1000) - mtime.time_boot_ms) - time_offset ;
		
		fcu_unix_valid = mtime.tv_sec > 1293840000; 
		ros_unix_valid = (time_unix_usec/1000) > 1293840000;

		if(dt > 2000 || dt < -2000) //2 sec
		{
		ROS_WARN_THROTTLE_NAMED(60, "time", "Companion reboot / clock skew");
		companion_reboot = true;
		}
		else
		{
		time_offset = (time_offset + ((time_unix_usec/1000) - mtime.time_boot_ms)))/2; 
		companion_reboot = false;
		}

		// px4 incoming msgs ADD
		if(companion_reboot)
		{
		ROS_WARN_THROTTLE_NAMED(60, "time", "Large clock skew detected. Resyncing clocks");
		time_offset = ((time_unix_usec/1000) - mtime.time_boot_ms));
		companion_reboot = false;
		}

		if(fcu_unix_valid) //continious publish for ntpd
		{
		sensor_msgs::TimeReferencePtr time_unix = boost::make_shared<sensor_msgs::TimeReference>();
		ros::Time time_ref(
				mtime.time_unix_usec / 1000000,			// t_sec
				(mtime.time_unix_usec % 1000000) * 1000);	// t_nsec

		time_unix->source = time_ref_source;
		time_unix->time_ref = time_ref;
		time_unix->header.frame_id = time_ref_source;
		time_unix->header.stamp = ros::Time::now();

		time_ref_pub.publish(time_unix);
		}

		//offset publisher
		sensor_msgs::TimeReferencePtr time_offset = boost::make_shared<sensor_msgs::TimeReference>();
		ros::Time time_ref(
				time_offset / 1000,	// t_sec
				time_offset * 1000);	// t_nsec

		time_offset->source = time_ref_source;
		time_offset->time_ref = time_ref;
		time_offset->header.frame_id = time_ref_source;
		time_offset->header.stamp = ros::Time::now();

		time_offset_pub.publish(time_offset);

	}

	void sys_time_cb(const ros::TimerEvent &event) {
		mavlink_message_t msg;

		time_unix_usec = ros::Time::now().toNSec() / 1000;
		
		mavlink_msg_system_time_pack_chan(UAS_PACK_CHAN(uas), &msg,
			time_unix_usec, /* nano -> micro */
			0
			);
		UAS_FCU(uas)->send_message(&msg);
		
	}

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SystemTimePlugin, mavplugin::MavRosPlugin)

