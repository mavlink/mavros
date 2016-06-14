/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


namespace mavros {
namespace extra_plugins{
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class MocapPoseEstimatePlugin : public plugin::PluginBase
{
public:
	MocapPoseEstimatePlugin() : PluginBase(),
		mp_nh("~mocap"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		bool use_tf;
		bool use_pose;

		PluginBase::initialize(uas_);


		/** @note For VICON ROS package, subscribe to TransformStamped topic */
		mp_nh.param("use_tf", use_tf, false);

		/** @note For Optitrack ROS package, subscribe to PoseStamped topic */
		mp_nh.param("use_pose", use_pose, true);

		if (use_tf && !use_pose) {
			mocap_tf_sub = mp_nh.subscribe("tf", 1, &MocapPoseEstimatePlugin::mocap_tf_cb, this);
		}
		else if (use_pose && !use_tf) {
			mocap_pose_sub = mp_nh.subscribe("pose", 1, &MocapPoseEstimatePlugin::mocap_pose_cb, this);
		}
		else {
			ROS_ERROR_NAMED("mocap", "Use one motion capture source.");
		}
	}

	Subscriptions get_subsctiptions() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;
	

	ros::Subscriber mocap_pose_sub;
	ros::Subscriber mocap_tf_sub;

	/* -*- low-level send -*- */
	void mocap_pose_send
		(uint64_t usec,
			float q[4],
			float x, float y, float z)
	{
		mavlink_message_t msg;
		mavlink_msg_att_pos_mocap_pack_chan(UAS_PACK_CHAN(m_uas), &msg,
				usec,
				q,
				x,
				y,
				z);
		UAS_FCU(m_uas)->send_message_ignore_drop(&msg);
	}

	/* -*- mid-level helpers -*- */
	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
		Eigen::Quaterniond q_enu;
		float q[4];

		tf::quaternionMsgToEigen(pose->pose.orientation, q_enu);
		UAS::quaternion_to_mavlink(
				UAS::transform_orientation_enu_ned(
					UAS::transform_orientation_baselink_aircraft(q_enu)),
				q);

		auto position = UAS::transform_frame_enu_ned(
				Eigen::Vector3d(
					pose->pose.position.x,
					pose->pose.position.y,
					pose->pose.position.z));

		mocap_pose_send(pose->header.stamp.toNSec() / 1000,
				q,
				position.x(),
				position.y(),
				position.z());
	}

	/* -*- callbacks -*- */
	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		Eigen::Quaterniond q_enu;
		float q[4];

		tf::quaternionMsgToEigen(trans->transform.rotation, q_enu);
		UAS::quaternion_to_mavlink(
				UAS::transform_orientation_enu_ned(
					UAS::transform_orientation_baselink_aircraft(q_enu)),
				q);

		auto position = UAS::transform_frame_enu_ned(
				Eigen::Vector3d(
					trans->transform.translation.x,
					trans->transform.translation.y,
					trans->transform.translation.z));

		mocap_pose_send(trans->header.stamp.toNSec() / 1000,
				q,
				position.x(),
				position.y(),
				position.z());
	}
};
}	// namespace extra_plugins
}	// namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MocapPoseEstimatePlugin, mavros::plugin::PluginBase)
