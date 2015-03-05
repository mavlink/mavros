/**
 * @brief SetpointAcceleration plugin
 * @file setpoint_accel.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Vector3Stamped.h>

namespace mavplugin {
/**
 * @brief Setpoint acceleration/force plugin
 *
 * Send setpoint accelerations/forces to FCU controller.
 */
class SetpointAccelerationPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointAccelerationPlugin> {
public:
	SetpointAccelerationPlugin() :
		sp_nh("~setpoint_accel"),
		uas(nullptr),
		send_force(false)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		sp_nh.param("send_force", send_force, false);

		accel_sub = sp_nh.subscribe("accel", 10, &SetpointAccelerationPlugin::accel_cb, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber accel_sub;

	bool send_force;

	/* -*- mid-level helpers -*- */

	/**
	 * Send acceleration/force to FCU acceleration controller
	 *
	 * Note: send only AFX AFY AFZ. ENU frame.
	 */
	void send_setpoint_acceleration(const ros::Time &stamp, float afx, float afy, float afz) {
		/* Documentation start from bit 1 instead 0.
		 * Ignore position and velocity vectors, yaw and yaw rate
		 */
		uint16_t ignore_all_except_a_xyz = (3 << 10) | (7 << 3) | (7 << 0);

		if (send_force)
			ignore_all_except_a_xyz |= (1 << 9);

		// ENU->NED. Issue #49.
		set_position_target_local_ned(stamp.toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_a_xyz,
				0.0, 0.0, 0.0,
				0.0, 0.0, 0.0,
				afy, afx, -afz,
				0.0, 0.0);
	}

	/* -*- callbacks -*- */

	void accel_cb(const geometry_msgs::Vector3Stamped::ConstPtr &req) {
		send_setpoint_acceleration(req->header.stamp,
				req->vector.x,
				req->vector.y,
				req->vector.z);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointAccelerationPlugin, mavplugin::MavRosPlugin)
