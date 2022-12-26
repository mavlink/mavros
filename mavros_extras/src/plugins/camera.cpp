/**
 * @brief Camera plugin
 * @file camera.cpp
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
		auto ic = boost::make_shared<mavros_msgs::CameraImageCaptured>();

		ic->header.stamp = m_uas->synchronise_stamp(mo.time_boot_ms);
		ic->geo.latitude = mo.lat/ 1E7;
		ic->geo.longitude = mo.lon / 1E7;		// deg
		ic->geo.altitude = mo.alt / 1E3 + m_uas->geoid_to_ellipsoid_height(&ic->geo);	// in meters
		ic->relative_alt = mo.relative_alt / 1E3;
		auto q = ftf::mavlink_to_quaternion(mo.q);
		tf::quaternionEigenToMsg(q, ic->orientation);
		ic->file_url = mavlink::to_string(mo.file_url);

		camera_image_captured_pub.publish(ic);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CameraPlugin, mavros::plugin::PluginBase)
