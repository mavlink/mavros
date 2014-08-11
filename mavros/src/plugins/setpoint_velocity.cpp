/**
 * @brief SetpointVelocity plugin
 * @file setpoint_velocity.cpp
 * @author Nuno Marques
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Twist.h>

#include "setpoint_mixin.h"

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
		uas(nullptr)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;
		sp_nh = ros::NodeHandle(nh, "setpoint");

		//cmd_vel usually is the topic used for velocity control in many controllers / planners
		vel_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointVelocityPlugin::vel_cb, this);
	}

	const std::string get_name() const {
		return "SetpointVelocity";
	}

	const std::vector<uint8_t> get_supported_messages() const {
		return { /* Rx disabled */ };
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber vel_sub;

	/* -*- mid-level helpers -*- */

	/**
	 * Send velocity to FCU velocity controller
	 *
	 * Note: send only VX VY VZ. ENU frame.
	 */
	void send_setpoint_velocity(float vx, float vy, float vz) {

		/* Documentation start from bit 1 instead 0,
		 * but implementation PX4 Firmware #1151 starts from 0
		 */
		uint16_t ignore_all_except_v_xyz = (7<<6)|(7<<0);

		// ENU->NED. Issue #49.
		set_position_target_local_ned(ros::Time::now().toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_v_xyz,
				0.0, 0.0, 0.0,
				vy, vx, -vz,
				0.0, 0.0, 0.0);
	}

	/* -*- callbacks -*- */

	void vel_cb(const geometry_msgs::Twist::ConstPtr &req) {
		send_setpoint_velocity(req->linear.x, req->linear.y, req->linear.z);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointVelocityPlugin, mavplugin::MavRosPlugin)
