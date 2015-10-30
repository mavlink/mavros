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

		setpoint_sub = sp_nh.subscribe("local", 10, &SetpointRawPlugin::setpoint_cb, this);
		target_pub = sp_nh.advertise<mavros_msgs::PositionTarget>("target", 10);
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, &SetpointRawPlugin::handle_position_target_local_ned),
		};
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber setpoint_sub;
	ros::Publisher target_pub;

	/* -*- message handlers -*- */
	void handle_position_target_local_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_position_target_local_ned_t tgt;
		mavlink_msg_position_target_local_ned_decode(msg, &tgt);

		/* TODO */
	}

	/* -*- callbacks -*- */

	void setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr &req) {
		Eigen::Vector3d position, velocity, af;

		// convert to Eigen
		tf::pointMsgToEigen(req->position, position);
		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		position = UAS::transform_frame_enu_ned(position);
		velocity = UAS::transform_frame_enu_ned(velocity);
		af = UAS::transform_frame_enu_ned(af);

		// Send message
		set_position_target_local_ned(
				req->header.stamp.toNSec() / 1000000,
				req->coordinate_frame,
				req->type_mask,
				position.x(), position.y(), position.z(),
				velocity.x(), velocity.y(), velocity.z(),
				af.x(), af.y(), af.z(),
				req->yaw, req->yaw_rate);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointRawPlugin, mavplugin::MavRosPlugin)
