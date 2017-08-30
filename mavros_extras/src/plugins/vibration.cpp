/**
 * @brief Vibration plugin
 * @file vibration.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/Vibration.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vibration plugin
 *
 * This plugin is intended to publish MAV vibration levels and accelerometer clipping from FCU.
 */
class VibrationPlugin : public plugin::PluginBase {
public:
	VibrationPlugin() : PluginBase(),
		vibe_nh("~vibration")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		vibe_nh.param<std::string>("frame_id", frame_id, "base_link");

		vibration_pub = vibe_nh.advertise<mavros_msgs::Vibration>("raw/vibration", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&VibrationPlugin::handle_vibration)
		};
	}

private:
	ros::NodeHandle vibe_nh;

	std::string frame_id;

	ros::Publisher vibration_pub;

	void handle_vibration(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VIBRATION &vibration)
	{
		auto vibe_msg = boost::make_shared<mavros_msgs::Vibration>();

		vibe_msg->header = m_uas->synchronized_header(frame_id, vibration.time_usec);

		Eigen::Vector3d vib_enu = {vibration.vibration_x, vibration.vibration_y, vibration.vibration_z};
		tf::vectorEigenToMsg(ftf::transform_frame_ned_enu(vib_enu), vibe_msg->vibration);

		vibe_msg->clipping[0] = vibration.clipping_0;
		vibe_msg->clipping[1] = vibration.clipping_1;
		vibe_msg->clipping[2] = vibration.clipping_2;

		vibration_pub.publish(vibe_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VibrationPlugin, mavros::plugin::PluginBase)
