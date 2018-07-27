/**
 * @brief GPS RTK plugin
 * @file gps_rtk.cpp
 * @author Alexis Paques <alexis.paques@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/RTCM.h>
#include <algorithm>

namespace mavros {
namespace std_plugins {
/**
 * @brief GPS RTK plugin
 *
 * Publish the RTCM messages from ROS to the Pixhawk
 */
class GpsRtkPlugin : public plugin::PluginBase {
public:
	GpsRtkPlugin() : PluginBase(),
		gps_rtk_nh("~gps_rtk")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		gps_rtk_sub = gps_rtk_nh.subscribe("send_rtcm", 10, &GpsRtkPlugin::rtk_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {};
	}

private:
	ros::NodeHandle gps_rtk_nh;
	ros::Subscriber gps_rtk_sub;

	void rtk_cb(const mavros_msgs::RTCM::ConstPtr &msg)
	{
		const int maxMessageLength = 180;	// Should be replaced by MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN
		int seq = msg->header.seq;

		if (msg->data.size() > 4 * maxMessageLength) {
			ROS_FATAL("gps_rtk: RTCM message received is bigger than the maximal possible size");
			return;
		}

		if (msg->data.size() < maxMessageLength) {
			mavlink::common::msg::GPS_RTCM_DATA rtcm_data;
			rtcm_data.len = msg->data.size();
			rtcm_data.flags = (seq & 0x1F) << 3;
			memcpy(&rtcm_data.data, &msg->data, msg->data.size());
			UAS_FCU(m_uas)->send_message(rtcm_data);
		} else {
			uint8_t fragmentId = 0;
			int start = 0;
			while (start < msg->data.size()) {
				mavlink::common::msg::GPS_RTCM_DATA rtcm_data;
				int length = std::min((int)msg->data.size() - start, maxMessageLength);
				rtcm_data.flags = 1;				// LSB set indicates message is fragmented
				rtcm_data.flags |= fragmentId++ << 1;		// Next 2 bits are fragment id
				rtcm_data.flags |= (seq & 0x1F) << 3;		// Next 5 bits are sequence id
				rtcm_data.len = length;
				memcpy(&rtcm_data.data, &msg->data + start, length);
				UAS_FCU(m_uas)->send_message(rtcm_data);
				start += length;
			}
		}
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::GpsRtkPlugin, mavros::plugin::PluginBase)
