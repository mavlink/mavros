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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


namespace mavplugin {
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class MocapPoseEstimatePlugin : public MavRosPlugin
{
public:
	MocapPoseEstimatePlugin() :
		mp_nh("~mocap"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		bool use_tf;
		bool use_pose;

		uas = &uas_;

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

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;
	UAS *uas;

	ros::Subscriber mocap_pose_sub;
	ros::Subscriber mocap_tf_sub;

	/* -*- low-level send -*- */
	void mocap_pose_send
		(uint64_t usec,
			float q[4],
			float x, float y, float z)
	{
		mavlink_message_t msg;
		mavlink_msg_att_pos_mocap_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				q,
				x,
				y,
				z);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */
	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
		float q[4];

		tf::Quaternion qo;
		quaternionMsgToTF(pose->pose.orientation,qo);
		auto qt = UAS::transform_frame_enu_ned_attitude_q(qo);

		q[0] = qt.w();
		q[1] = qt.x();
		q[2] = qt.y();
		q[3] = qt.z();

		auto position = UAS::transform_frame_enu_ned_xyz(
					pose->pose.position.x,
					pose->pose.position.y,
					pose->pose.position.z);

		mocap_pose_send(pose->header.stamp.toNSec() / 1000,
				q,
				position.x(),
				position.y(),
				position.z());
	}

	/* -*- callbacks -*- */
	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		float q[4];
		
		tf::Transform tf;
		transformMsgToTF(trans->transform,tf);
		tf::Quaternion qo = tf.getRotation();
		auto qt = UAS::transform_frame_enu_ned_attitude_q(qo);

		q[0] = qt.w();
		q[1] = qt.x();
		q[2] = qt.y();
		q[3] = qt.z();

		auto position = UAS::transform_frame_enu_ned_xyz(
					trans->transform.translation.x,
					trans->transform.translation.y,
					trans->transform.translation.z);

		mocap_pose_send(trans->header.stamp.toNSec() / 1000,
				q,
				position.x(),
				position.y(),
				position.z());
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapPoseEstimatePlugin, mavplugin::MavRosPlugin)
