/**
 * @brief RTK plugin 
 * @file rtk.cpp
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

namespace mavros {
namespace std_plugins {
/**
 * @brief RTK plugin
 *
 * Publish the RTCM messages from ROS to the Pixhawk
 */
class RTKPlugin : public plugin::PluginBase {
public:
	RTKPlugin() : PluginBase(),
		rtk_nh("~rtk")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		rtk_sub = rtk_nh.subscribe("send_rtcm", 10, &RTKPlugin::rtk_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {
		};
	}

private:
	ros::NodeHandle rtk_nh;
	ros::Subscriber rtk_sub;
	unsigned int _sequenceId = 0;

	void rtk_cb(const mavros_msgs::RTCM::ConstPtr &msg)
	{
		mavlink::common::msg::GPS_RTCM_DATA rtcm_data;

		rtcm_data.len = msg->len;
		rtcm_data.flags = msg->flags;
		for(uint i=0; i < 180; i++) {
			rtcm_data.data[i] = msg->data[i];
		}
		UAS_FCU(m_uas)->send_message_ignore_drop(rtcm_data);
		_sequenceId += 1;
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::RTKPlugin, mavros::plugin::PluginBase)
