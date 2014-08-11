/**
 * @brief SetpointAttitude plugin
 * @file setpoint_attitude.cpp
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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "setpoint_mixin.h"

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
		uas(nullptr),
		tf_rate(10.0)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool pose_with_covariance;
		bool listen_tf;
		bool listen_twist;

		uas = &uas_;
		sp_nh = ros::NodeHandle(nh, "setpoint");

		sp_nh.param("attitude/listen_twist", listen_twist, true);
		sp_nh.param("attitude/pose_with_covariance", pose_with_covariance, false);
		// may be used to mimic attitude of an object, a gesture, etc.
		sp_nh.param("attitude/listen_tf", listen_tf, false);
		sp_nh.param<std::string>("attitude/frame_id", frame_id, "local_origin");
		sp_nh.param<std::string>("attitude/child_frame_id", child_frame_id, "attitude");
		sp_nh.param("attitude/tf_rate_limit", tf_rate, 10.0);

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("attitude", "Listen to desired attitude transform " << frame_id
					<< " -> " << child_frame_id);
			tf_start("AttitudeSpTF", &SetpointAttitudePlugin::send_attitude_transform);
		}
		else if (listen_twist) {
			ROS_DEBUG_NAMED("attitude", "Setpoint attitude topic type: Twist");
			att_sub = sp_nh.subscribe("att_vel", 10, &SetpointAttitudePlugin::twist_cb, this);
		}
		else if (pose_with_covariance) {
			ROS_DEBUG_NAMED("attitude", "Setpoint attitude topic type: PoseWithCovarianceStamped");
			att_sub = sp_nh.subscribe("attitude", 10, &SetpointAttitudePlugin::pose_cov_cb, this);
		}
		else {
			ROS_DEBUG_NAMED("attitude", "Setpoint attitude topic type: PoseStamped");
			att_sub = sp_nh.subscribe("attitude", 10, &SetpointAttitudePlugin::pose_cb, this);
		}
	}

	const std::string get_name() const {
		return "SetpointAttitude";
	}

	const std::vector<uint8_t> get_supported_messages() const {
		return { /* Rx disabled */ };
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	}

private:
	friend class TFListenerMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber att_sub;

	std::string frame_id;
	std::string child_frame_id;

	double tf_rate;

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
		uas->mav_link->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * Send attitude setpoint to FCU attitude controller
	 *
	 * ENU frame.
	 */
	void send_attitude_transform(const tf::Transform &transform, const ros::Time &stamp) {
		// Thrust + RPY, also bits noumbering started from 1 in docs
		const uint8_t ignore_all_except_q = (1<<6)|(7<<0);
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
	void send_attitude_ang_velocity(const float vx, const float vy, const float vz) {
		// Q + Thrust, also bits noumbering started from 1 in docs
		const uint8_t ignore_all_except_rpy = (1<<7)|(1<<6);
		float q[4] = { 1.0, 0.0, 0.0, 0.0 };

		set_attitude_target(ros::Time::now().toNSec() / 1000000,
				ignore_all_except_rpy,
				q,
				vy, vx, -vz,
				0.0);
	}

	/**
	 * TODO: Thrust control message reception
	 */

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

	void twist_cb(const geometry_msgs::Twist::ConstPtr &req) {
		send_attitude_ang_velocity(
				req->angular.x,
				req->angular.y,
				req->angular.z);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointAttitudePlugin, mavplugin::MavRosPlugin)
