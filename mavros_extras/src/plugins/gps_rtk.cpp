/**
 * @brief GPS RTK plugin
 * @file gps_rtk.cpp
 * @author Alexis Paques <alexis.paques@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Alexis Paques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/RTCM.h>
#include <algorithm>

namespace mavros {
namespace extra_plugins {
/**
 * @brief GPS RTK plugin
 *
 * Publish the RTCM messages from ROS to the FCU
 */
class GpsRtkPlugin : public plugin::PluginBase {
public:
	GpsRtkPlugin() : PluginBase(),
		gps_rtk_nh("~gps_rtk")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		gps_rtk_sub = gps_rtk_nh.subscribe("send_rtcm", 10, &GpsRtkPlugin::rtcm_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {};
	}

private:
	ros::NodeHandle gps_rtk_nh;
	ros::Subscriber gps_rtk_sub;

	/* -*- callbacks -*- */
	/**
	 * @brief Handle mavros_msgs::RTCM message
	 * It converts the message to the MAVLink GPS_RTCM_DATA message for GPS injection.
	 * Message specification: https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA
	 * @param msg		Received ROS msg
	 */
	void rtcm_cb(const mavros_msgs::RTCM::ConstPtr &msg)
	{
		mavlink::common::msg::GPS_RTCM_DATA rtcm_data;
		const size_t max_frag_len = rtcm_data.data.size();

		uint8_t seq_u5 = uint8_t(msg->header.seq & 0x1F) << 3;

		if (msg->data.size() > 4 * max_frag_len) {
			ROS_FATAL("gps_rtk: RTCM message received is bigger than the maximal possible size.");
			return;
		}

		auto data_it = msg->data.begin();
		auto end_it = msg->data.end();

		if (msg->data.size() <= max_frag_len) {
			rtcm_data.len = msg->data.size();
			rtcm_data.flags = seq_u5;
			std::copy(data_it, end_it, rtcm_data.data.begin());
			std::fill(rtcm_data.data.begin() + rtcm_data.len, rtcm_data.data.end(), 0);
			UAS_FCU(m_uas)->send_message(rtcm_data);
		} else {
			for (uint8_t fragment_id = 0; fragment_id < 4 && data_it < end_it; fragment_id++) {
				uint8_t len = std::min((size_t) std::distance(data_it, end_it), max_frag_len);
				rtcm_data.flags = 1;				// LSB set indicates message is fragmented
				rtcm_data.flags |= fragment_id << 1;		// Next 2 bits are fragment id
				rtcm_data.flags |= seq_u5;		// Next 5 bits are sequence id
				rtcm_data.len = len;

				std::copy(data_it, data_it + len, rtcm_data.data.begin());
				std::fill(rtcm_data.data.begin() + len, rtcm_data.data.end(), 0);
				UAS_FCU(m_uas)->send_message(rtcm_data);
				std::advance(data_it, len);
			}
		}
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GpsRtkPlugin, mavros::plugin::PluginBase)
