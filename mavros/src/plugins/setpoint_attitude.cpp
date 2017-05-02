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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/Float32Stamped.h>

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
		use_attitude(true),
		reverse_throttle(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool tf_listen;

		// main params
		sp_nh.param("use_attitude", use_attitude, true);
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

			//TODO: implement tf2_ros::MessageFilter< M >
		}
		else if (use_attitude) {
			//use message_filters to sync attitude and throttle msg coming from different topics
			message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(sp_nh, "attitude", 1);
			message_filters::Subscriber<mavros_msgs::Float32Stamped> throttle_sub(sp_nh, "throttle", 1);

			// matches messages, even if they have different time stamps, by using an adaptative algorithm <http://wiki.ros.org/message_filters/ApproximateTime>
			typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, mavros_msgs::Float32Stamped> syncp;
			message_filters::Synchronizer<syncp> sync(syncp(10), pose_sub, throttle_sub);
			sync.registerCallback(boost::bind(&SetpointAttitudePlugin::attitude_pose_cb, this, _1, _2));
		}
		else {
			twist_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointAttitudePlugin::twist_cb, this);
		}
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

	std::string tf_frame_id;
	std::string tf_child_frame_id;
	double tf_rate;
	bool use_attitude;
	bool reverse_throttle;

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send attitude setpoint and respective thrust to FCU attitude controller
	 */
	void send_attitude_target(const ros::Time &stamp, const Eigen::Affine3d &tr, const float throttle)
	{
		/* RPY, also bits numbering started from 1 in docs
		 */
		const uint8_t ignore_all_except_q_and_thrust = (7 << 0);

		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))
					);

		set_attitude_target(stamp.toNSec() / 1000000,
					ignore_all_except_q_and_thrust,
					q,
					Eigen::Vector3d::Zero(),
					throttle);
	}

	/**
	 * @brief Send angular velocity setpoint to FCU attitude controller
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

	/* -*- callbacks -*- */

	void transform_cb(const geometry_msgs::TransformStamped &transform) {	//TODO: Replace this function with a sync one
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);

		send_attitude_target(transform.header.stamp, tr, 0.0);
	}

	void attitude_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const mavros_msgs::Float32Stamped::ConstPtr &throttle_msg) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(pose_msg->pose, tr);

		float throttle_normalized = throttle_msg->data;

		/**
		 * && are lazy, is_normalized() should be called only if reverse_throttle are true.
		 */
		if (reverse_throttle && !is_normalized(throttle_normalized, -1.0, 1.0))
			return;
		else if (!is_normalized(throttle_normalized, 0.0, 1.0))
			return;

		send_attitude_target(pose_msg->header.stamp, tr, throttle_normalized);
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
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointAttitudePlugin, mavros::plugin::PluginBase)
