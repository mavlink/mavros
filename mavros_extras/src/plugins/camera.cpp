/**
 * @brief Mouny Control plugin
 * @file mount_control.cpp
 * @author Jaeyoung Lim <jalim@ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2021 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CameraImageCaptured.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink enumerations
using mavlink::common::MAV_CMD;
using utils::enum_value;

/**
 * @brief Camera plugin plugin
 *
 * Plugin for interfacing on the mavlink camera protocol
 * @see command_cb()
 */
class CameraPlugin : public plugin::PluginBase {
public:
	CameraPlugin() : PluginBase(),
	nh("~"),
	camera_nh("~camera")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		camera_image_captured_pub = camera_nh.advertise<mavros_msgs::CameraImageCaptured>("image_captured", 10);

	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&CameraPlugin::handle_camera_image_captured)
		};
	}

private:
	ros::NodeHandle nh;
	ros::NodeHandle camera_nh;
	ros::Publisher camera_image_captured_pub;

	/**
	 * @brief Publish camera image capture information
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED
	 * @param msg   the mavlink message
	 * @param mo	received CAMERA_IMAGE_CAPTURED msg
	 */
	void handle_camera_image_captured(const mavlink::mavlink_message_t *msg, mavlink::common::msg::CAMERA_IMAGE_CAPTURED &mo)
	{
		mavros_msgs::CameraImageCaptured image_captured_msg;
		image_captured_msg.header.stamp = ros::Time::now();
		image_captured_msg.latitude = mo.lat;
		image_captured_msg.longitude = mo.lon;
		image_captured_msg.altitude = mo.alt;
		image_captured_msg.relative_alt = mo.relative_alt;
		image_captured_msg.file_url = std::string(std::begin(mo.file_url), std::end(mo.file_url));

		camera_image_captured_pub.publish(image_captured_msg);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CameraPlugin, mavros::plugin::PluginBase)
