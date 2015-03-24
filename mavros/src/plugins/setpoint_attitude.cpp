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
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

namespace mavplugin {
/**
 * @brief Setpoint attitude plugin
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointAttitudePlugin : public MavRosPlugin,
	private TFListenerMixin<SetpointAttitudePlugin> {
public:
	SetpointAttitudePlugin() :
		sp_nh("~setpoint_attitude"),
		uas(nullptr),
		tf_rate(10.0),
		reverse_throttle(false)
	{ };

	void initialize(UAS &uas_)
	{
		bool pose_with_covariance;
		bool listen_tf;
		bool listen_twist;

		uas = &uas_;

		sp_nh.param("listen_twist", listen_twist, true);
		sp_nh.param("pose_with_covariance", pose_with_covariance, false);
		// may be used to mimic attitude of an object, a gesture, etc.
		sp_nh.param("listen_tf", listen_tf, false);
		sp_nh.param<std::string>("frame_id", frame_id, "local_origin");
		sp_nh.param<std::string>("child_frame_id", child_frame_id, "attitude");
		sp_nh.param("tf_rate_limit", tf_rate, 10.0);
		sp_nh.param("reverse_throttle", reverse_throttle, false);

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("attitude", "Listen to desired attitude transform " << frame_id
					<< " -> " << child_frame_id);
			tf_start("AttitudeSpTF", &SetpointAttitudePlugin::send_attitude_transform);
		}
		else if (listen_twist) {
			ROS_DEBUG_NAMED("attitude", "Setpoint attitude topic type: Twist");
			att_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointAttitudePlugin::twist_cb, this);
		}
		else if (pose_with_covariance) {
			ROS_DEBUG_NAMED("attitude", "Setpoint attitude topic type: PoseWithCovarianceStamped");
			att_sub = sp_nh.subscribe("attitude", 10, &SetpointAttitudePlugin::pose_cov_cb, this);
		}
		else {
			ROS_DEBUG_NAMED("attitude", "Setpoint attitude topic type: PoseStamped");
			att_sub = sp_nh.subscribe("attitude", 10, &SetpointAttitudePlugin::pose_cb, this);
		}

		throttle_sub = sp_nh.subscribe("att_throttle", 10, &SetpointAttitudePlugin::throttle_cb, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class TFListenerMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber att_sub;
	ros::Subscriber throttle_sub;

	std::string frame_id;
	std::string child_frame_id;

	double tf_rate;
	bool reverse_throttle;

	/* -*- low-level send -*- */

	void set_attitude_target(uint32_t time_boot_ms,
			uint8_t type_mask,
			float q[4],
			float roll_rate, float pitch_rate, float yaw_rate,
			float thrust) {
		mavlink_message_t msg;
		mavlink_msg_set_attitude_target_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms,
				UAS_PACK_TGT(uas),
				type_mask,
				q,
				roll_rate, pitch_rate, yaw_rate,
				thrust);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * Send attitude setpoint to FCU attitude controller
	 *
	 * ENU frame.
	 */
	void send_attitude_transform(const tf::Transform &transform, const ros::Time &stamp) {
		// Thrust + RPY, also bits noumbering started from 1 in docs
		const uint8_t ignore_all_except_q = (1 << 6) | (7 << 0);
		float q[4];

		// ENU->NED, description in #49.
		tf::Quaternion tf_q = transform.getRotation();
		q[0] = tf_q.w();
		q[1] = tf_q.y();
		q[2] = tf_q.x();
		q[3] = -tf_q.z();

		set_attitude_target(stamp.toNSec() / 1000000,
				ignore_all_except_q,
				q,
				0.0, 0.0, 0.0,
				0.0);
	}

	/**
	 * Send angular velocity setpoint to FCU attitude controller
	 *
	 * ENU frame.
	 */
	void send_attitude_ang_velocity(const ros::Time &stamp, const float vx, const float vy, const float vz) {
		// Q + Thrust, also bits noumbering started from 1 in docs
		const uint8_t ignore_all_except_rpy = (1 << 7) | (1 << 6);
		float q[4] = { 1.0, 0.0, 0.0, 0.0 };

		set_attitude_target(stamp.toNSec() / 1000000,
				ignore_all_except_rpy,
				q,
				vy, vx, -vz,
				0.0);
	}

	/**
	 * Send throttle to FCU attitude controller
	 */
	void send_attitude_throttle(const float throttle) {
		// Q + RPY
		const uint8_t ignore_all_except_throttle = (1 << 7) | (7 << 0);
		float q[4] = { 1.0, 0.0, 0.0, 0.0 };

		set_attitude_target(ros::Time::now().toNSec() / 1000000,
				ignore_all_except_throttle,
				q,
				0.0, 0.0, 0.0,
				throttle);
	}

	/* -*- callbacks -*- */

	void pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose.pose, transform);
		send_attitude_transform(transform, req->header.stamp);
	}

	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose, transform);
		send_attitude_transform(transform, req->header.stamp);
	}

	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		send_attitude_ang_velocity(
				req->header.stamp,
				req->twist.angular.x,
				req->twist.angular.y,
				req->twist.angular.z);
	}

	inline bool is_normalized(float throttle, const float min, const float max) {
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

	void throttle_cb(const std_msgs::Float64::ConstPtr &req) {
		float throttle_normalized = req->data;

		// note: && are lazy, is_normalized() should be called only if reverse_throttle are true.
		if (reverse_throttle && !is_normalized(throttle_normalized, -1.0, 1.0))
			return;
		else if (!is_normalized(throttle_normalized, 0.0, 1.0))
			return;

		send_attitude_throttle(throttle_normalized);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointAttitudePlugin, mavplugin::MavRosPlugin)
