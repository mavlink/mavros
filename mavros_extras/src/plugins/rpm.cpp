/**
 * @brief RPM plugin
 * @file rpm.cpp
 * @author Pavlo Kolomiiets <pkolomiets@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Pavlo Kolomiiets.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/RPM.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Ardupilot rpm plugin.
 *
 * This plugin allows publishing RPM sensor data from Ardupilot FCU to ROS.
 *
 */
class RPMPlugin : public plugin::PluginBase {
public:
	RPMPlugin() : PluginBase(),
		nh("~")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		rpm_pub = nh.advertise<mavros_msgs::RPM>("rpm", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&RPMPlugin::handle_rpm)
		};
	}

private:
	ros::NodeHandle nh;

	std::vector<float> rpms;

	ros::Publisher rpm_pub;

	void handle_rpm(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::RPM &rpm) {

		rpms.resize(2);
		rpms[0] = rpm.rpm1;
		rpms[1] = rpm.rpm2;

		auto rpm_msg = boost::make_shared<mavros_msgs::RPM>();
		rpm_msg->header.stamp = ros::Time::now();
		rpm_msg->header.frame_id = "";
		rpm_msg->rpm = rpms;

		rpm_pub.publish(rpm_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RPMPlugin, mavros::plugin::PluginBase)
