/**
 * @brief SetpointRAW plugin
 * @file setpoint_raw.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

namespace mavplugin {
/**
 * @brief Setpoint RAW plugin
 *
 * Send position setpoints and publish current state (return loop).
 * User can decide what set of filed needed for operation via IGNORE bits.
 */
class SetpointRawPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointRawPlugin> {
public:
	SetpointRawPlugin() :
		sp_nh("~setpoint_raw"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		bool tf_listen;

		uas = &uas_;

		local_sub = sp_nh.subscribe("local", 10, &SetpointRawPlugin::local_cb, this);
		global_sub = sp_nh.subscribe("global", 10, &SetpointRawPlugin::global_cb, this);
		target_local_pub = sp_nh.advertise<mavros_msgs::PositionTarget>("target_local", 10);
		target_global_pub = sp_nh.advertise<mavros_msgs::GlobalPositionTarget>("target_global", 10);
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, &SetpointRawPlugin::handle_position_target_local_ned),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, &SetpointRawPlugin::handle_position_target_global_int),
		};
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber local_sub, global_sub;
	ros::Publisher target_local_pub, target_global_pub;

	/* -*- message handlers -*- */
	void handle_position_target_local_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_position_target_local_ned_t tgt;
		mavlink_msg_position_target_local_ned_decode(msg, &tgt);

		// Transform frame NED->ENU
		auto position = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.x, tgt.y, tgt.z));
		auto velocity = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));

		auto target = boost::make_shared<mavros_msgs::PositionTarget>();

		target->header.stamp = uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		tf::pointEigenToMsg(position, target->position);
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = tgt.yaw;
		target->yaw_rate = tgt.yaw_rate;

		target_local_pub.publish(target);
	}

	void handle_position_target_global_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_position_target_global_int_t tgt;
		mavlink_msg_position_target_global_int_decode(msg, &tgt);

		// Transform frame NED->ENU
		auto velocity = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = UAS::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));

		auto target = boost::make_shared<mavros_msgs::GlobalPositionTarget>();

		target->header.stamp = uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		target->latitude = tgt.lat_int / 1e7;
		target->longitude = tgt.lon_int / 1e7;
		target->altitude = tgt.alt;
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = tgt.yaw;
		target->yaw_rate = tgt.yaw_rate;

		target_global_pub.publish(target);
	}

	/* -*- low-level send -*- */
	//! Message specification: @p https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT
	void set_position_target_global_int(uint32_t time_boot_ms, uint8_t coordinate_frame, uint8_t type_mask,
			int32_t lat_int, int32_t lon_int, float alt,
			Eigen::Vector3d &velocity,
			Eigen::Vector3d &af,
			float yaw, float yaw_rate) {
		mavlink_message_t msg;
		mavlink_msg_set_position_target_global_int_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms,
				UAS_PACK_TGT(uas),
				coordinate_frame,
				type_mask,
				lat_int, lon_int, alt,
				velocity.x(), velocity.y(), velocity.z(),
				af.x(), af.y(), af.z(),
				yaw, yaw_rate);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	void local_cb(const mavros_msgs::PositionTarget::ConstPtr &req) {
		Eigen::Vector3d position, velocity, af;

		tf::pointMsgToEigen(req->position, position);
		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		position = UAS::transform_frame_enu_ned(position);
		velocity = UAS::transform_frame_enu_ned(velocity);
		af = UAS::transform_frame_enu_ned(af);

		set_position_target_local_ned(
				req->header.stamp.toNSec() / 1000000,
				req->coordinate_frame,
				req->type_mask,
				position.x(), position.y(), position.z(),
				velocity.x(), velocity.y(), velocity.z(),
				af.x(), af.y(), af.z(),
				req->yaw, req->yaw_rate);
	}

	void global_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr &req) {
		Eigen::Vector3d velocity, af;

		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		velocity = UAS::transform_frame_enu_ned(velocity);
		af = UAS::transform_frame_enu_ned(af);

		set_position_target_global_int(
				req->header.stamp.toNSec() / 1000000,
				req->coordinate_frame,
				req->type_mask,
				req->latitude * 1e7,
				req->longitude * 1e7,
				req->altitude,
				velocity, af,
				req->yaw, req->yaw_rate);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointRawPlugin, mavplugin::MavRosPlugin)
