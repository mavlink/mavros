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
		uas(nullptr),
		manual_def(false),
		srv_def(true)
		
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool manual_def = false;
		bool srv_def = true;
		double vx, vy, vz;		

		uas = &uas_;
		sp_nh = ros::NodeHandle(nh, "setpoint");

		if (sp_nh.getParam("vx", vx) &&
				sp_nh.getParam("vy", vy) &&
				sp_nh.getParam("vz", vz)) {
			manual_def = true;
			ROS_DEBUG_NAMED("setpoint", "SP_vel: Manual set: lin_vel(%f %f %f)",
					vx, vy, vz);
		}
		
		else
			manual_def = false;

		if (manual_def) // defined when running the node
			send_setpoint_velocity(ros::Time::now(),
						vx, vy, vz);
		
		else if (srv_def) // defined by mavsetp service
			vel_sub = sp_nh.subscribe("local/vel/set", 10, &SetpointVelocityPlugin::vel_cb, this);
		
		else // defined by a topic - usually a controller/planner topic - cmd_vel is a commin vel control topic
			vel_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointVelocityPlugin::vel_cb, this);
	}

	const std::string get_name() const {
		return "SetpointVelocity";
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber vel_sub;
	
	bool manual_def;
	bool srv_def;

	/* -*- mid-level helpers -*- */

	/**
	 * Send velocity to FCU velocity controller
	 *
	 * Note: send only VX VY VZ. ENU frame.
	 */
	void send_setpoint_velocity(const ros::Time &stamp, double vx, double vy, double vz) {

		/* Documentation start from bit 1 instead 0,
		 * but implementation PX4 Firmware #1151 starts from 0
		 */
		uint16_t ignore_all_except_v_xyz = (7<<6)|(7<<0);

		// ENU->NED. Issue #49.
		set_position_target_local_ned(stamp.toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_v_xyz,
				0.0, 0.0, 0.0,
				vy, vx, -vz,
				0.0, 0.0, 0.0);
	}

	/* -*- callbacks -*- */

	void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		send_setpoint_velocity(req->header.stamp,
					req->twist.linear.x,
					req->twist.linear.y,
					req->twist.linear.z);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointVelocityPlugin, mavplugin::MavRosPlugin)
