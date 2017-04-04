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
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint velocity plugin
 *
 * Send setpoint velocities to FCU controller.
 */
class SetpointVelocityPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin> {
public:
	SetpointVelocityPlugin() : PluginBase(),
		sp_nh("~setpoint_velocity")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		//cmd_vel usually is the topic used for velocity control in many controllers / planners
		vel_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointVelocityPlugin::vel_cb, this);
		vel_unstamped_sub = sp_nh.subscribe("cmd_vel_unstamped", 10, &SetpointVelocityPlugin::vel_unstamped_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber vel_sub;
	ros::Subscriber vel_unstamped_sub;

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send velocity to FCU velocity controller
	 *
	 * @warning Send only VX VY VZ. ENU frame.
	 */
	void send_setpoint_velocity(const ros::Time &stamp, Eigen::Vector3d &vel_enu, double yaw_rate)
	{
		using mavlink::common::MAV_FRAME;

		/**
		 * Documentation start from bit 1 instead 0;
		 * Ignore position and accel vectors, yaw.
		 */
		uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);

		auto vel = ftf::transform_frame_enu_ned(vel_enu);
		auto yr = ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(0.0, 0.0, yaw_rate));

		set_position_target_local_ned(stamp.toNSec() / 1000000,
				utils::enum_value(MAV_FRAME::LOCAL_NED),
				ignore_all_except_v_xyz_yr,
				Eigen::Vector3d::Zero(),
				vel,
				Eigen::Vector3d::Zero(),
				0.0, yr.z());
	}

	/* -*- callbacks -*- */

	void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		Eigen::Vector3d vel_enu;

		tf::vectorMsgToEigen(req->twist.linear, vel_enu);
		send_setpoint_velocity(req->header.stamp, vel_enu,
				req->twist.angular.z);
	}
	
	void vel_unstamped_cb(const geometry_msgs::Twist::ConstPtr &req) {
		Eigen::Vector3d vel_enu;

		tf::vectorMsgToEigen(req->linear, vel_enu);
		send_setpoint_velocity(ros::Time::now(), vel_enu,
				req->angular.z);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointVelocityPlugin, mavros::plugin::PluginBase)
