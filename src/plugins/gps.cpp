/**
 * @brief GPS publish plugin
 * @file gps.cpp
 * @author Vladimit Ermkov <voon341@gmail.com>
 */
/*
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

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>

namespace mavplugin {

class GPSPlugin : public MavRosPlugin {
public:
	GPSPlugin()
	{};

	void initialize(ros::NodeHandle &nh,
			const boost::shared_ptr<mavconn::MAVConnInterface> &mav_link,
			diagnostic_updater::Updater &diag_updater)
	{
		nh.param<std::string>("gps/frame_id", frame_id, "gps");
		nh.param<std::string>("gps/time_ref_source", time_ref_source, frame_id);

		fix_pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 10);
		time_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", 10);
		vel_pub = nh.advertise<geometry_msgs::TwistStamped>("gps_vel", 10);
	}

	std::string get_name() {
		return "GPS";
	}

	std::vector<uint8_t> get_supported_messages() {
		return {
			MAVLINK_MSG_ID_GPS_RAW_INT,
			MAVLINK_MSG_ID_GPS_STATUS,
			MAVLINK_MSG_ID_SYSTEM_TIME
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_GPS_RAW_INT:
			{
			}
			break;

		case MAVLINK_MSG_ID_GPS_STATUS:
			{
			}
			break;

		case MAVLINK_MSG_ID_SYSTEM_TIME:
			{
				mavlink_system_time_t mtime;
				mavlink_msg_system_time_decode(msg, &mtime);

				if (mtime.time_unix_usec == 0) {
					ROS_WARN_THROTTLE_NAMED(60, "mavros", "Wrong system time. Is GPS Ok? (boot_ms: %u)",
							mtime.time_boot_ms);
					return;
				}

				sensor_msgs::TimeReferencePtr time(new sensor_msgs::TimeReference);
				ros::Time time_ref(
						mtime.time_unix_usec / 1000000,			// t_sec
						(mtime.time_unix_usec % 1000000) * 1000);	// t_nsec

				time->source = time_ref_source;
				time->time_ref = time_ref;
				time->header.frame_id = time_ref_source;
				time->header.stamp = ros::Time::now();

				time_ref_pub.publish(time);
			}
			break;
		};
	}

private:
	std::string frame_id;
	std::string time_ref_source;

	ros::Publisher fix_pub;
	ros::Publisher time_ref_pub;
	ros::Publisher vel_pub;
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::GPSPlugin, mavplugin::MavRosPlugin)

