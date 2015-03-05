/**
 * @brief SetpointVelocity plugin
 * @file setpoint_velocity.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/TwistStamped.h>

namespace mavplugin {
/**
 * @brief Setpoint velocity plugin
 *
 * Send setpoint velocities to FCU controller.
 */
class SetpointVelocityPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin> {
public:
	SetpointVelocityPlugin() :
		sp_nh("~setpoint_velocity"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		//cmd_vel usually is the topic used for velocity control in many controllers / planners
		vel_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointVelocityPlugin::vel_cb, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber vel_sub;

	/* -*- mid-level helpers -*- */

	/**
	 * Send velocity to FCU velocity controller
	 *
	 * Note: send only VX VY VZ. ENU frame.
	 */
	void send_setpoint_velocity(const ros::Time &stamp, float vx, float vy, float vz, float yaw_rate) {
		/* Documentation start from bit 1 instead 0,
		 * Ignore position and accel vectors, yaw
		 */
		uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);

		// ENU->NED. Issue #49.
		set_position_target_local_ned(stamp.toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_v_xyz_yr,
				0.0, 0.0, 0.0,
				vy, vx, -vz,
				0.0, 0.0, 0.0,
				0.0, yaw_rate);
	}

	/* -*- callbacks -*- */

	void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		send_setpoint_velocity(req->header.stamp,
				req->twist.linear.x,
				req->twist.linear.y,
				req->twist.linear.z,
				req->twist.angular.z);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointVelocityPlugin, mavplugin::MavRosPlugin)
