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
		uas(nullptr)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool use_tf;
		bool use_pose;

		uas = &uas_;
		mp_nh = ros::NodeHandle(nh, "mocap");

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
	UAS *uas;

	ros::NodeHandle mp_nh;
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
