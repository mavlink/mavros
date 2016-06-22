/**
 * @brief SetpointRAW plugin
 * @file setpoint_raw.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint RAW plugin
 *
 * Send position setpoints and publish current state (return loop).
 * User can decide what set of filed needed for operation via IGNORE bits.
 */
class SetpointRawPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointRawPlugin>,
	private plugin::SetPositionTargetGlobalIntMixin<SetpointRawPlugin>,
	private plugin::SetAttitudeTargetMixin<SetpointRawPlugin> {
public:
	SetpointRawPlugin() : PluginBase(),
		sp_nh("~setpoint_raw")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool tf_listen;

		local_sub = sp_nh.subscribe("local", 10, &SetpointRawPlugin::local_cb, this);
		global_sub = sp_nh.subscribe("global", 10, &SetpointRawPlugin::global_cb, this);
		attitude_sub = sp_nh.subscribe("attitude", 10, &SetpointRawPlugin::attitude_cb, this);
		target_local_pub = sp_nh.advertise<mavros_msgs::PositionTarget>("target_local", 10);
		target_global_pub = sp_nh.advertise<mavros_msgs::GlobalPositionTarget>("target_global", 10);
		target_attitude_pub = sp_nh.advertise<mavros_msgs::AttitudeTarget>("target_attitude", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&SetpointRawPlugin::handle_position_target_local_ned),
			make_handler(&SetpointRawPlugin::handle_position_target_global_int),
			make_handler(&SetpointRawPlugin::handle_attitude_target),
		};
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	friend class SetPositionTargetGlobalIntMixin;
	friend class SetAttitudeTargetMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber local_sub, global_sub, attitude_sub;
	ros::Publisher target_local_pub, target_global_pub, target_attitude_pub;

	/* -*- message handlers -*- */
	void handle_position_target_local_ned(const mavlink::mavlink_message_t *msg, mavlink::common::msg::POSITION_TARGET_LOCAL_NED &tgt)
	{
		// Transform desired position,velocities,and accels from ENU to NED frame
		auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.x, tgt.y, tgt.z));
		auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
		float yaw = ftf::transform_frame_yaw_ned_enu(tgt.yaw);
		float yaw_rate = ftf::transform_frame_yaw_ned_enu(tgt.yaw_rate);

		auto target = boost::make_shared<mavros_msgs::PositionTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		tf::pointEigenToMsg(position, target->position);
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = yaw;
		target->yaw_rate = yaw_rate;

		target_local_pub.publish(target);
	}

	void handle_position_target_global_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::POSITION_TARGET_GLOBAL_INT &tgt)
	{
		// Transform desired velocities from ENU to NED frame
		auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
		float yaw = ftf::transform_frame_yaw_ned_enu(tgt.yaw);
		float yaw_rate = ftf::transform_frame_yaw_ned_enu(tgt.yaw_rate);

		auto target = boost::make_shared<mavros_msgs::GlobalPositionTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		target->latitude = tgt.lat_int / 1e7;
		target->longitude = tgt.lon_int / 1e7;
		target->altitude = tgt.alt;
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = yaw;
		target->yaw_rate = yaw_rate;

		target_global_pub.publish(target);
	}

	void handle_attitude_target(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE_TARGET &tgt)
	{
		// Transform orientation from baselink -> ENU
		// to aircraft -> NED
		auto orientation = ftf::transform_orientation_ned_enu(
						   ftf::transform_orientation_baselink_aircraft(
							   Eigen::Quaterniond(tgt.q[0], tgt.q[1], tgt.q[2], tgt.q[3])));

		auto body_rate = ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(tgt.body_roll_rate, tgt.body_pitch_rate, tgt.body_yaw_rate));

		auto target = boost::make_shared<mavros_msgs::AttitudeTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->type_mask = tgt.type_mask;
		tf::quaternionEigenToMsg(orientation, target->orientation);
		tf::vectorEigenToMsg(body_rate, target->body_rate);
		target->thrust = tgt.thrust;

		target_attitude_pub.publish(target);
	}

	/* -*- callbacks -*- */

	void local_cb(const mavros_msgs::PositionTarget::ConstPtr &req)
	{
		Eigen::Vector3d position, velocity, af;
		float yaw, yaw_rate;

		tf::pointMsgToEigen(req->position, position);
		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		position = ftf::transform_frame_enu_ned(position);
		velocity = ftf::transform_frame_enu_ned(velocity);
		af = ftf::transform_frame_enu_ned(af);
		yaw = ftf::transform_frame_yaw_enu_ned(req->yaw);
		yaw_rate = ftf::transform_frame_yaw_enu_ned(req->yaw_rate);

		set_position_target_local_ned(
				req->header.stamp.toNSec() / 1000000,
				req->coordinate_frame,
				req->type_mask,
				position,
				velocity,
				af,
				yaw, yaw_rate);
	}

	void global_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr &req)
	{
		Eigen::Vector3d velocity, af;
		float yaw, yaw_rate;

		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		velocity = ftf::transform_frame_enu_ned(velocity);
		af = ftf::transform_frame_enu_ned(af);
		yaw = ftf::transform_frame_yaw_enu_ned(req->yaw);
		yaw_rate = ftf::transform_frame_yaw_enu_ned(req->yaw_rate);

		set_position_target_global_int(
				req->header.stamp.toNSec() / 1000000,
				req->coordinate_frame,
				req->type_mask,
				req->latitude * 1e7,
				req->longitude * 1e7,
				req->altitude,
				velocity,
				af,
				yaw, yaw_rate);
	}

	void attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr &req)
	{
		Eigen::Quaterniond desired_orientation;
		Eigen::Vector3d baselink_angular_rate;

		tf::quaternionMsgToEigen(req->orientation, desired_orientation);

		// Transform desired orientation to represent aircraft->NED,
		// MAVROS operates on orientation of base_link->ENU
		auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
			ftf::transform_orientation_baselink_aircraft(desired_orientation));

		auto body_rate = ftf::transform_frame_baselink_aircraft(baselink_angular_rate);

		tf::vectorMsgToEigen(req->body_rate, body_rate);

		set_attitude_target(
				req->header.stamp.toNSec() / 1000000,
				req->type_mask,
				ned_desired_orientation,
				body_rate,
				req->thrust);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointRawPlugin, mavros::plugin::PluginBase)
