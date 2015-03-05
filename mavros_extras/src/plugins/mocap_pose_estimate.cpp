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
 * Sends mountion capture data to FCU.
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

		mp_nh.param("use_tf", use_tf, false);		// Vicon
		mp_nh.param("use_pose", use_pose, true);	// Optitrack


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

	// mavlink send

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


	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
		float q[4];
		q[0] =  pose->pose.orientation.y;	// w
		q[1] =  pose->pose.orientation.x;	// x
		q[2] = -pose->pose.orientation.z;	// y
		q[3] =  pose->pose.orientation.w;	// z
		// Convert to mavlink body frame
		mocap_pose_send(pose->header.stamp.toNSec() / 1000,
				q,
				pose->pose.position.x,
				-pose->pose.position.y,
				-pose->pose.position.z);
	}

	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		float q[4];
		q[0] =  trans->transform.rotation.y;	// w
		q[1] =  trans->transform.rotation.x;	// x
		q[2] = -trans->transform.rotation.z;	// y
		q[3] =  trans->transform.rotation.w;	// z
		// Convert to mavlink body frame
		mocap_pose_send(trans->header.stamp.toNSec() / 1000,
				q,
				trans->transform.translation.x,
				-trans->transform.translation.y,
				-trans->transform.translation.z);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapPoseEstimatePlugin, mavplugin::MavRosPlugin)
