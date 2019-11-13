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
#include <mavros_msgs/SetMavFrame.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_srvs/Trigger.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;

/**
 * @brief Setpoint TRAJECTORY plugin
 *
 * Receive trajectory setpoints and send setpoint_raw setpoints along the trajectory.
 */
class SetpointTrajectoryPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointTrajectoryPlugin> {
public:
	SetpointTrajectoryPlugin() : PluginBase(),
		sp_nh("~setpoint_trajectory")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		sp_nh.param<std::string>("frame_id", frame_id, "map");

		local_sub = sp_nh.subscribe("local", 10, &SetpointTrajectoryPlugin::local_cb, this);
		desired_pub = sp_nh.advertise<nav_msgs::Path>("desired", 10);

		trajectory_reset_srv = sp_nh.advertiseService("reset", &SetpointTrajectoryPlugin::reset_cb, this);
		mav_frame_srv = sp_nh.advertiseService("mav_frame", &SetpointTrajectoryPlugin::set_mav_frame_cb, this);
		
		sp_timer = sp_nh.createTimer(ros::Duration(0.01), &SetpointTrajectoryPlugin::reference_cb, this, true);

		// mav_frame
		std::string mav_frame_str;
		if (!sp_nh.getParam("mav_frame", mav_frame_str)) {
			mav_frame = MAV_FRAME::LOCAL_NED;
		} else {
			mav_frame = utils::mav_frame_from_str(mav_frame_str);
		}
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	using lock_guard = std::lock_guard<std::mutex>;
	std::mutex mutex;

	ros::NodeHandle sp_nh;

	ros::Timer sp_timer;

	ros::Subscriber local_sub;
	ros::Publisher desired_pub;

	ros::ServiceServer trajectory_reset_srv;
	ros::ServiceServer mav_frame_srv;

	trajectory_msgs::MultiDOFJointTrajectory::ConstPtr trajectory_target_msg;
	std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::const_iterator setpoint_target;
	std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::const_iterator next_setpoint_target;

	std::string frame_id;
	MAV_FRAME mav_frame;
	ftf::StaticTF transform;

	void reset_timer(ros::Duration duration){
		sp_timer.stop();
		sp_timer.setPeriod(duration);
		sp_timer.start();		
	}

	void publish_path(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &req){
		nav_msgs::Path msg;

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = frame_id;
		for (const auto &p : req->points) {
			if (p.transforms.empty())
				continue;
	
			geometry_msgs::PoseStamped pose_msg;
			pose_msg.pose.position.x = p.transforms[0].translation.x;
			pose_msg.pose.position.y = p.transforms[0].translation.y;
			pose_msg.pose.position.z = p.transforms[0].translation.z;
			pose_msg.pose.orientation = p.transforms[0].rotation;
			msg.poses.emplace_back(pose_msg);
		}
		desired_pub.publish(msg);
	}

	/* -*- callbacks -*- */

	void local_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &req)
	{
		lock_guard lock(mutex);

		if(static_cast<MAV_FRAME>(mav_frame) == MAV_FRAME::BODY_NED || static_cast<MAV_FRAME>(mav_frame) == MAV_FRAME::BODY_OFFSET_NED){
			transform = ftf::StaticTF::BASELINK_TO_AIRCRAFT;
		} else {
			transform = ftf::StaticTF::ENU_TO_NED;
		}

		trajectory_target_msg = req;

		// Read first duration of the setpoint and set the timer
		setpoint_target = req->points.cbegin();
		reset_timer(setpoint_target->time_from_start);
		publish_path(req);
	}

	void reference_cb(const ros::TimerEvent &event)
	{
		using mavlink::common::POSITION_TARGET_TYPEMASK;
		lock_guard lock(mutex);

		if(!trajectory_target_msg)
			return;

		Eigen::Vector3d position, velocity, af;
		Eigen::Quaterniond attitude;
		float yaw, yaw_rate;
		uint16_t type_mask = 0;		
		if(!setpoint_target->transforms.empty()){
			position = ftf::detail::transform_static_frame(ftf::to_eigen(setpoint_target->transforms[0].translation), transform);
			attitude = ftf::detail::transform_orientation(ftf::to_eigen(setpoint_target->transforms[0].rotation), transform);

		} else {
			type_mask = type_mask | uint16_t(POSITION_TARGET_TYPEMASK::X_IGNORE)
							| uint16_t(POSITION_TARGET_TYPEMASK::Y_IGNORE) 
							| uint16_t(POSITION_TARGET_TYPEMASK::Z_IGNORE)
							| uint16_t(POSITION_TARGET_TYPEMASK::YAW_IGNORE);

		}

		if(!setpoint_target->velocities.empty()){
			velocity = ftf::detail::transform_static_frame(ftf::to_eigen(setpoint_target->velocities[0].linear), transform);
			yaw_rate = setpoint_target->velocities[0].angular.z;

		} else {
			type_mask = type_mask | uint16_t(POSITION_TARGET_TYPEMASK::VX_IGNORE)
							| uint16_t(POSITION_TARGET_TYPEMASK::VY_IGNORE) 
							| uint16_t(POSITION_TARGET_TYPEMASK::VZ_IGNORE)
							| uint16_t(POSITION_TARGET_TYPEMASK::YAW_RATE_IGNORE);

		}
		
		if(!setpoint_target->accelerations.empty()){
			af = ftf::detail::transform_static_frame(ftf::to_eigen(setpoint_target->accelerations[0].linear), transform);

		} else {
			type_mask = type_mask | uint16_t(POSITION_TARGET_TYPEMASK::AX_IGNORE) 
							| uint16_t(POSITION_TARGET_TYPEMASK::AY_IGNORE) 
							| uint16_t(POSITION_TARGET_TYPEMASK::AZ_IGNORE);

		}

		set_position_target_local_ned(
					ros::Time::now().toNSec() / 1000000,
					utils::enum_value(mav_frame),
					type_mask,
					position,
					velocity,
					af,
					ftf::quaternion_get_yaw(attitude),
					yaw_rate);
	
		next_setpoint_target = setpoint_target + 1;
		if (next_setpoint_target != trajectory_target_msg->points.cend()) {
			reset_timer(setpoint_target->time_from_start);
			setpoint_target = next_setpoint_target;
		} else {
			trajectory_target_msg.reset();
		}

		return;
	}

	bool reset_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
	{
		if(trajectory_target_msg){
			trajectory_target_msg.reset();
			res.success = true;
		} else {
			res.success = false;
			res.message = "Trajectory reset denied: Empty trajectory";
		}

		return true;
	}

	bool set_mav_frame_cb(mavros_msgs::SetMavFrame::Request &req, mavros_msgs::SetMavFrame::Response &res)
	{
		mav_frame = static_cast<MAV_FRAME>(req.mav_frame);
		const std::string mav_frame_str = utils::to_string(mav_frame);
		sp_nh.setParam("mav_frame", mav_frame_str);
		res.success = true;
		return true;
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointTrajectoryPlugin, mavros::plugin::PluginBase)
