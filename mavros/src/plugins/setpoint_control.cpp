/**
 * @brief SetpointPosition plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros/ControlSetpoint.h>

#include <geometry_msgs/Pose.h>


namespace mavplugin {
/**
 * @brief Setpoint control plugin
 *
 * Send setpoint to FCU for specified controller.
 */
class SetpointControlPlugin : public MavRosPlugin {
public:
	SetpointControlPlugin() :
		sp_nh("~setpoint_control"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;
		sp_nh.param("mav_control_target", mav_control_target, 100);

		control_sub = sp_nh.subscribe("setpoint", 10, &SetpointControlPlugin::setpoint_cb, this);
	}

        const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}


private:
	ros::NodeHandle sp_nh;
	UAS *uas;

	int mav_control_target;

	ros::Subscriber control_sub;

	void send_setpoint_control_target_ned(float x, float y, float z, float yaw)
	{
		mavlink_message_t msg;
		mavlink_msg_set_control_target_local_ned_pack_chan(UAS_PACK_CHAN(uas), &msg,
				x, y, z, yaw,
				UAS_PACK_TGT(uas),
				mav_control_target,
				MAV_FRAME_BODY_NED);
		UAS_FCU(uas)->send_message(&msg);
	}

	void setpoint_cb(const mavros::ControlSetpoint &msg)
	{
		float x, y, z, yaw;
		x = msg.x;
		y = msg.y;
		z = msg.z;
		yaw = msg.yaw;

		send_setpoint_control_target_ned(x,y,z,yaw);
	}

};
}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointControlPlugin, mavplugin::MavRosPlugin)
