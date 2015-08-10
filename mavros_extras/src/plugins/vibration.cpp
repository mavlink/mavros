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
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/Vibration.h>

namespace mavplugin {
/**
 * @brief Vibration plugin
 *
 * This plugin is intended to publish MAV vibration levels and accelerometer clipping from FCU.
 */
class VibrationPlugin : public MavRosPlugin {
public:
	VibrationPlugin() :
		vibe_nh("~vibration"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		vibe_nh.param<std::string>("frame_id", frame_id, "vibration");

		vibration_pub = vibe_nh.advertise<mavros_msgs::Vibration>("raw/vibration", 10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_VIBRATION, &VibrationPlugin::handle_vibration)
		};
	}

private:
	ros::NodeHandle vibe_nh;
	UAS *uas;

	std::string frame_id;

	ros::Publisher vibration_pub;

	void handle_vibration(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_vibration_t vibration;
		mavlink_msg_vibration_decode(msg, &vibration);

		auto vibe_msg = boost::make_shared<mavros_msgs::Vibration>();

		vibe_msg->header = uas->synchronized_header(frame_id, vibration.time_usec);

		// TODO no transform_frame?
		vibe_msg->vibration.x = vibration.vibration_x;
		vibe_msg->vibration.y = vibration.vibration_y;
		vibe_msg->vibration.z = vibration.vibration_z;
		vibe_msg->clipping[0] = vibration.clipping_0;
		vibe_msg->clipping[1] = vibration.clipping_1;
		vibe_msg->clipping[2] = vibration.clipping_2;

		vibration_pub.publish(vibe_msg);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VibrationPlugin, mavplugin::MavRosPlugin)
