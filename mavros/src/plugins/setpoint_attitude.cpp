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
#include <mavros_msgs/Thrust.h>

namespace mavros {
namespace std_plugins {

using SyncPoseThrustPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, mavros_msgs::Thrust>;
using SyncTwistThrustPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, mavros_msgs::Thrust>;
using SyncPoseThrust = message_filters::Synchronizer<SyncPoseThrustPolicy>;
using SyncTwistThrust = message_filters::Synchronizer<SyncTwistThrustPolicy>;

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
		tf_rate(50.0),
		use_quaternion(false),
		reverse_thrust(false),
		tf_listen(false)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// main params
		sp_nh.param("use_quaternion", use_quaternion, false);
		sp_nh.param("reverse_thrust", reverse_thrust, false);
		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "target_attitude");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);

		// thrust msg subscriber to sync
		th_sub.subscribe(sp_nh, "thrust", 1);

		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("attitude",
						"Listen to desired attitude transform "
						<< tf_frame_id << " -> " << tf_child_frame_id);

			tf2_start<mavros_msgs::Thrust>("AttitudeSpTFSync", th_sub, &SetpointAttitudePlugin::transform_cb);
		}
		else if (use_quaternion) {
			/**
			 * @brief Use message_filters to sync attitude and thrust msg coming from different topics
			 */
			pose_sub.subscribe(sp_nh, "attitude", 1);

			/**
			 * @brief Matches messages, even if they have different time stamps,
			 * by using an adaptative algorithm <http://wiki.ros.org/message_filters/ApproximateTime>
			 */
			sync_pose.reset(new SyncPoseThrust(SyncPoseThrustPolicy(10), pose_sub, th_sub));
			sync_pose->registerCallback(boost::bind(&SetpointAttitudePlugin::attitude_pose_cb, this, _1, _2));
		}
		else {
			twist_sub.subscribe(sp_nh, "cmd_vel", 1);
			sync_twist.reset(new SyncTwistThrust(SyncTwistThrustPolicy(10), twist_sub, th_sub));
			sync_twist->registerCallback(boost::bind(&SetpointAttitudePlugin::attitude_twist_cb, this, _1, _2));
		}
	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetAttitudeTargetMixin;
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;

	message_filters::Subscriber<mavros_msgs::Thrust> th_sub;
	message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
	message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub;

	std::unique_ptr<SyncPoseThrust> sync_pose;
	std::unique_ptr<SyncTwistThrust> sync_twist;

	std::string tf_frame_id;
	std::string tf_child_frame_id;

	bool tf_listen;
	double tf_rate;

	bool use_quaternion;

	bool reverse_thrust;
	float normalized_thrust;

	/**
	 * @brief Function to verify if the thrust values are normalized;
	 * considers also the reversed trust values
	 */
	inline bool is_normalized(float thrust){
		if (reverse_thrust) {
			if (thrust < -1.0) {
				ROS_WARN_NAMED("attitude", "Not normalized reversed thrust! Thd(%f) < Min(%f)", thrust, -1.0);
				return false;
			}
		}
		else {
			if (thrust < 0.0) {
				ROS_WARN_NAMED("attitude", "Not normalized thrust! Thd(%f) < Min(%f)", thrust, 0.0);
				return false;
			}
		}

		if (thrust > 1.0) {
			ROS_WARN_NAMED("attitude", "Not normalized thrust! Thd(%f) > Max(%f)", thrust, 1.0);
			return false;
		}
		return true;
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send attitude setpoint and thrust to FCU attitude controller
	 */
	void send_attitude_quaternion(const ros::Time &stamp, const Eigen::Affine3d &tr, const float thrust)
	{
		/**
		 * @note RPY, also bits numbering started from 1 in docs
		 */
		const uint8_t ignore_all_except_q_and_thrust = (7 << 0);

		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))
					);

		set_attitude_target(stamp.toNSec() / 1000000,
					ignore_all_except_q_and_thrust,
					q,
					Eigen::Vector3d::Zero(),
					thrust);
	}

	/**
	 * @brief Send angular velocity setpoint and thrust to FCU attitude controller
	 */
	void send_attitude_ang_velocity(const ros::Time &stamp, const Eigen::Vector3d &ang_vel, const float thrust)
	{
		/**
		 * @note Q, also bits noumbering started from 1 in docs
		 */
		const uint8_t ignore_all_except_rpy = (1 << 7);

		auto av = ftf::transform_frame_ned_enu(ang_vel);

		set_attitude_target(stamp.toNSec() / 1000000,
					ignore_all_except_rpy,
					Eigen::Quaterniond::Identity(),
					av,
					thrust);
	}

	/* -*- callbacks -*- */

	void transform_cb(const geometry_msgs::TransformStamped &transform, const mavros_msgs::Thrust::ConstPtr &thrust_msg) {
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);

		send_attitude_quaternion(transform.header.stamp, tr, thrust_msg->thrust);
	}

	void attitude_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const mavros_msgs::Thrust::ConstPtr &thrust_msg) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(pose_msg->pose, tr);

		if (is_normalized(thrust_msg->thrust))
			send_attitude_quaternion(pose_msg->header.stamp, tr, thrust_msg->thrust);
	}

	void attitude_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req, const mavros_msgs::Thrust::ConstPtr &thrust_msg) {
		Eigen::Vector3d ang_vel;
		tf::vectorMsgToEigen(req->twist.angular, ang_vel);

		if (is_normalized(thrust_msg->thrust))
			send_attitude_ang_velocity(req->header.stamp, ang_vel, thrust_msg->thrust);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointAttitudePlugin, mavros::plugin::PluginBase)
