/**
 * @brief SetpointTRAJECTORY plugin
 * @file setpoint_trajectory.cpp
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint TRAJECTORY plugin
 *
 * Receive trajectory setpoints and send setpoint_raw setpoints along the trajectory.
 */
class SetpointTrajectoryPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointTrajectoryPlugin>,
	private plugin::SetPositionTargetGlobalIntMixin<SetpointTrajectoryPlugin>,
	private plugin::SetAttitudeTargetMixin<SetpointTrajectoryPlugin> {
public:
	SetpointTrajectoryPlugin() : PluginBase(),
		sp_nh("~setpoint_trajectory"),
		TRAJ_SAMPLING_DT(TRAJ_SAMPLING_MS / 1000.0)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		sp_nh.param<std::string>("frame_id", frame_id, "map");

		local_sub = sp_nh.subscribe("local", 10, &SetpointTrajectoryPlugin::local_cb, this);
		desired_pub = sp_nh.advertise<nav_msgs::Path>("desired", 10);

		sp_timer = sp_nh.createTimer(ros::Duration(0.01), &SetpointTrajectoryPlugin::reference_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	friend class SetPositionTargetGlobalIntMixin;
	friend class SetAttitudeTargetMixin;
	ros::NodeHandle sp_nh;

	ros::Timer sp_timer;
	ros::Time refstart_time;

	ros::Subscriber local_sub;
	ros::Publisher desired_pub;

	trajectory_msgs::MultiDOFJointTrajectory::ConstPtr trajectory_target_msg;

	std::string frame_id;

	static constexpr int TRAJ_SAMPLING_MS = 100;

	const ros::Duration TRAJ_SAMPLING_DT;

	void publish_path(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &req){
		nav_msgs::Path msg;

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = frame_id;
		for(size_t i = 0; i < req->points.size(); i++){
			geometry_msgs::PoseStamped pose_msg;
			if(!req->points[i].transforms.empty()){
				pose_msg.pose.position.x = req->points[i].transforms[0].translation.x;
				pose_msg.pose.position.y = req->points[i].transforms[0].translation.y;
				pose_msg.pose.position.z = req->points[i].transforms[0].translation.z;
				pose_msg.pose.orientation.w = req->points[i].transforms[0].rotation.w;
				pose_msg.pose.orientation.x = req->points[i].transforms[0].rotation.x;
				pose_msg.pose.orientation.y = req->points[i].transforms[0].rotation.y;
				pose_msg.pose.orientation.z = req->points[i].transforms[0].rotation.z;
				msg.poses.emplace_back(pose_msg);
			}
		}
		desired_pub.publish(msg);
	}

	/* -*- callbacks -*- */

	void local_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &req)
	{
		trajectory_target_msg = req;
		refstart_time = ros::Time::now();
		publish_path(req);
	}

	void reference_cb(const ros::TimerEvent &event)
	{
		if(trajectory_target_msg) {
			ros::Duration curr_time_from_start;
			curr_time_from_start = ros::Time::now() - refstart_time;
		
			for(size_t i = 0; i < trajectory_target_msg->points.size(); i++){

				Eigen::Vector3d position, velocity, af;
				Eigen::Quaterniond attitude;
				float yaw, yaw_rate;
				trajectory_msgs::MultiDOFJointTrajectoryPoint pt = trajectory_target_msg->points[i];
				uint16_t type_mask;

				if(pt.time_from_start.toSec() >= curr_time_from_start.toSec() ) { //TODO: Better logic to handle this case?
					if(!pt.transforms.empty()){
					position << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
					attitude = Eigen::Quaterniond(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y, pt.transforms[0].rotation.z);
					} else {
						type_mask = type_mask || mavros_msgs::PositionTarget::IGNORE_PX || mavros_msgs::PositionTarget::IGNORE_PY || mavros_msgs::PositionTarget::IGNORE_PZ;
					}

					if(!pt.velocities.empty()) velocity << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;
					else type_mask = type_mask || mavros_msgs::PositionTarget::IGNORE_VX || mavros_msgs::PositionTarget::IGNORE_VY || mavros_msgs::PositionTarget::IGNORE_VZ;
					
					if(!pt.accelerations.empty()) af << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
					else type_mask = type_mask || mavros_msgs::PositionTarget::IGNORE_VX || mavros_msgs::PositionTarget::IGNORE_VY || mavros_msgs::PositionTarget::IGNORE_VZ;

					// Transform frame ENU->NED
					position = ftf::transform_frame_enu_ned(position);
					velocity = ftf::transform_frame_enu_ned(velocity);
					af = ftf::transform_frame_enu_ned(af);
					yaw = ftf::quaternion_get_yaw(attitude);

					set_position_target_local_ned(
								trajectory_target_msg->header.stamp.toNSec() / 1000000,
								1,
								0,
								position,
								velocity,
								af,
								yaw, 0);
					if(i == trajectory_target_msg->points.size()-1)  trajectory_target_msg.reset(); //End of trajectory
					break;
				}
			}
		}
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointTrajectoryPlugin, mavros::plugin::PluginBase)
