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
#include <mavros_msgs/RTKBaseline.h>
#include <algorithm>

namespace mavros {
namespace extra_plugins {
/**
 * @brief GPS RTK plugin
 *
 * 1. Publish the RTCM messages from ROS to the FCU
 * 2. Publish RTK baseline data from the FCU to ROS
 */
class GpsRtkPlugin : public plugin::PluginBase {
public:
	GpsRtkPlugin() : PluginBase(),
		gps_rtk_nh("~gps_rtk")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		gps_rtk_sub = gps_rtk_nh.subscribe("send_rtcm", 10, &GpsRtkPlugin::rtcm_cb, this);
		rtk_baseline_pub_ = gps_rtk_nh.advertise<mavros_msgs::RTKBaseline>("rtk_baseline", 1, true);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			       make_handler( &GpsRtkPlugin::handle_baseline_msg )
		};
	}

private:
	ros::NodeHandle gps_rtk_nh;
	ros::Subscriber gps_rtk_sub;

	ros::Publisher rtk_baseline_pub_;
	mavros_msgs::RTKBaseline rtk_baseline_;

	/* -*- callbacks -*- */
	/**
	 * @brief Handle mavros_msgs::RTCM message
	 * It converts the message to the MAVLink GPS_RTCM_DATA message for GPS injection.
	 * Message specification: https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA
	 * @param msg		Received ROS msg
	 */
	void rtcm_cb(const mavros_msgs::RTCM::ConstPtr &msg)
	{
		mavlink::common::msg::GPS_RTCM_DATA rtcm_data = {};
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

	/* MAvlink msg handlers */
	/**
	 * @brief Publish GPS_RTK message (MAvlink Common) received from FCU.
	 * The message is already decoded by Mavlink, we only need to convert to ROS.
	 * Details and units: https://mavlink.io/en/messages/common.html#GPS_RTK
	 */

	void handle_baseline_msg( const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RTK &rtk_bsln )
	{
		/* Received a decoded packet containing mavlink's msg #127,#128 in Common.
		   Simply convert to ROS and publish.
		 */
		rtk_baseline_.time_last_baseline_ms = rtk_bsln.time_last_baseline_ms;
		rtk_baseline_.rtk_receiver_id = rtk_bsln.rtk_receiver_id;
		rtk_baseline_.wn = rtk_bsln.wn;				// week num.
		rtk_baseline_.tow = rtk_bsln.tow;			// ms
		rtk_baseline_.rtk_health = rtk_bsln.rtk_health;
		rtk_baseline_.rtk_rate = rtk_bsln.rtk_rate;
		rtk_baseline_.nsats = rtk_bsln.nsats;
		rtk_baseline_.baseline_coords_type = rtk_bsln.baseline_coords_type;	// 0: ECEF, 1: NED
		rtk_baseline_.baseline_a_mm = rtk_bsln.baseline_a_mm;
		rtk_baseline_.baseline_b_mm = rtk_bsln.baseline_b_mm;
		rtk_baseline_.baseline_c_mm = rtk_bsln.baseline_c_mm;
		rtk_baseline_.accuracy = rtk_bsln.accuracy;
		rtk_baseline_.iar_num_hypotheses = rtk_bsln.iar_num_hypotheses;

		rtk_baseline_.header.stamp = ros::Time::now();
		rtk_baseline_pub_.publish( rtk_baseline_ );
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GpsRtkPlugin, mavros::plugin::PluginBase)
