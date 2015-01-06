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
 * Copyright 2014 Vladimir Ermakov.
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

		mp_nh.param("use_tf", use_tf, false);  // Vicon
		mp_nh.param("use_pose", use_pose, true);  // Optitrack


		if (use_tf && !use_pose)
		{
			mocap_tf_sub = mp_nh.subscribe("tf", 1, &MocapPoseEstimatePlugin::mocap_tf_cb, this);
		}
		else if (use_pose && !use_tf)
		{
			mocap_pose_sub = mp_nh.subscribe("pose", 1, &MocapPoseEstimatePlugin::mocap_pose_cb, this);
		}
		else
		{
			ROS_ERROR_NAMED("mocap", "Use one motion capture source.");
		}
	}

	const std::string get_name() const
	{
		return "MocapPoseEstimate";
	}

	const message_map get_rx_handlers()
	{
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
		float x, float y, float z,
		float roll, float pitch, float yaw)
	{
		mavlink_message_t msg;
		mavlink_msg_vicon_position_estimate_pack_chan(UAS_PACK_CHAN(uas), &msg,
			usec,
			x,
			y,
			z,
			roll,
			pitch,
			yaw);
		UAS_FCU(uas)->send_message(&msg);
	}


	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
		tf::Quaternion quat;
		tf::quaternionMsgToTF(pose->pose.orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		// Convert to mavlink body frame
		mocap_pose_send(pose->header.stamp.toNSec() / 1000,
			pose->pose.position.x,
			-pose->pose.position.y,
			-pose->pose.position.z,
			roll, -pitch, -yaw); 
	}

	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		tf::Quaternion quat;
		tf::quaternionMsgToTF(trans->transform.rotation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		// Convert to mavlink body frame
		mocap_pose_send(trans->header.stamp.toNSec() / 1000,
			trans->transform.translation.x,
			-trans->transform.translation.y,
			-trans->transform.translation.z,
			roll, -pitch, -yaw); 
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapPoseEstimatePlugin, mavplugin::MavRosPlugin)
