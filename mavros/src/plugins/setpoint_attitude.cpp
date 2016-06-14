/**
 * @brief SetpointAttitude plugin
 * @file setpoint_attitude.cpp
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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint attitude plugin
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointAttitudePlugin : public plugin::PluginBase,
	private plugin::SetAttitudeTargetMixin<SetpointAttitudePlugin>,
	private plugin::TF2ListenerMixin<SetpointAttitudePlugin> {
public:
	SetpointAttitudePlugin() : PluginBase(),
		sp_nh("~setpoint_attitude"),
		tf_rate(10.0),
		reverse_throttle(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool tf_listen;

		// main params
		sp_nh.param("reverse_throttle", reverse_throttle, false);
		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "aircraft");
		sp_nh.param("tf/rate_limit", tf_rate, 10.0);

		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("attitude",
					"Listen to desired attitude transform "
					<< tf_frame_id << " -> " << tf_child_frame_id);
			tf2_start("AttitudeSpTF", &SetpointAttitudePlugin::transform_cb);
		}
		else {
			twist_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointAttitudePlugin::twist_cb, this);
			pose_sub = sp_nh.subscribe("attitude", 10, &SetpointAttitudePlugin::pose_cb, this);
		}

		throttle_sub = sp_nh.subscribe("att_throttle", 10, &SetpointAttitudePlugin::throttle_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetAttitudeTargetMixin;
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber twist_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber throttle_sub;

	std::string tf_frame_id;
	std::string tf_child_frame_id;
	double tf_rate;
	bool reverse_throttle;

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send attitude setpoint to FCU attitude controller
	 *
	 * @note ENU frame.
	 */
	void send_attitude_target(const ros::Time &stamp, const Eigen::Affine3d &tr)
	{
		/* Thrust + RPY, also bits numbering started from 1 in docs
		 */
		const uint8_t ignore_all_except_q = (1 << 6) | (7 << 0);

		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))
					);

		set_attitude_target(stamp.toNSec() / 1000000,
				ignore_all_except_q,
				q,
				Eigen::Vector3d::Zero(),
				0.0);
	}

	/**
	 * @brief Send angular velocity setpoint to FCU attitude controller
	 *
	 * @note ENU frame.
	 */
	void send_attitude_ang_velocity(const ros::Time &stamp, const Eigen::Vector3d &ang_vel)
	{
		/* Q + Thrust, also bits noumbering started from 1 in docs
		 */
		const uint8_t ignore_all_except_rpy = (1 << 7) | (1 << 6);

		auto av = ftf::transform_frame_baselink_aircraft(ang_vel);

		set_attitude_target(stamp.toNSec() / 1000000,
				ignore_all_except_rpy,
				Eigen::Quaterniond::Identity(),
				av,
				0.0);
	}

	/**
	 * @brief Send throttle to FCU attitude controller
	 */
	void send_attitude_throttle(const float throttle)
	{
		// Q + RPY
		const uint8_t ignore_all_except_throttle = (1 << 7) | (7 << 0);

		set_attitude_target(ros::Time::now().toNSec() / 1000000,
				ignore_all_except_throttle,
				Eigen::Quaterniond::Identity(),
				Eigen::Vector3d::Zero(),
				throttle);
	}

	/* -*- callbacks -*- */

	void transform_cb(const geometry_msgs::TransformStamped &transform) {
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);

		send_attitude_target(transform.header.stamp, tr);
	}

	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		send_attitude_target(req->header.stamp, tr);
	}

	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		Eigen::Vector3d ang_vel;
		tf::vectorMsgToEigen(req->twist.angular, ang_vel);

		send_attitude_ang_velocity(req->header.stamp, ang_vel);
	}

	inline bool is_normalized(float throttle, const float min, const float max)
	{
		if (throttle < min) {
			ROS_WARN_NAMED("attitude", "Not normalized throttle! Thd(%f) < Min(%f)", throttle, min);
			return false;
		}
		else if (throttle > max) {
			ROS_WARN_NAMED("attitude", "Not normalized throttle! Thd(%f) > Max(%f)", throttle, max);
			return false;
		}

		return true;
	}

	void throttle_cb(const std_msgs::Float64::ConstPtr &req)
	{
		float throttle_normalized = req->data;

		/**
		 * && are lazy, is_normalized() should be called only if reverse_throttle are true.
		 */
		if (reverse_throttle && !is_normalized(throttle_normalized, -1.0, 1.0))
			return;
		else if (!is_normalized(throttle_normalized, 0.0, 1.0))
			return;

		send_attitude_throttle(throttle_normalized);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointAttitudePlugin, mavros::plugin::PluginBase)
