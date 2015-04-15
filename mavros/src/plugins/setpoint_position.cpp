/**
 * @brief SetpointPosition plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>

namespace mavplugin {
/**
 * @brief Setpoint position plugin
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointPositionPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointPositionPlugin>,
	private TFListenerMixin<SetpointPositionPlugin> {
public:
	SetpointPositionPlugin() :
		sp_nh("~setpoint_position"),
		uas(nullptr),
		tf_rate(10.0)
	{ };

	void initialize(UAS &uas_)
	{
		bool listen_tf;

		uas = &uas_;

		sp_nh.param("listen_tf", listen_tf, false);
		sp_nh.param<std::string>("frame_id", frame_id, "local_origin");
		sp_nh.param<std::string>("child_frame_id", child_frame_id, "setpoint");
		sp_nh.param("tf_rate_limit", tf_rate, 50.0);

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("setpoint", "Listen to position setpoint transform " << frame_id
					<< " -> " << child_frame_id);
			tf_start("PositionSpTF", &SetpointPositionPlugin::send_setpoint_transform);
		}
		else {
			setpoint_sub = sp_nh.subscribe("local", 10, &SetpointPositionPlugin::setpoint_cb, this);
		}
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	friend class TFListenerMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber setpoint_sub;

	std::string frame_id;
	std::string child_frame_id;

	double tf_rate;

	/* -*- mid-level helpers -*- */

	/**
	 * Send transform to FCU position controller
	 *
	 * Note: send only XYZ, Yaw
	 */
	void send_setpoint_transform(const tf::Transform &transform, const ros::Time &stamp) {
		// ENU frame
		tf::Vector3 origin = transform.getOrigin();
		tf::Quaternion q = transform.getRotation();

		/* Documentation start from bit 1 instead 0,
		 * Ignore velocity and accel vectors, yaw rate
		 */
		uint16_t ignore_all_except_xyz_y = (1 << 11) | (7 << 6) | (7 << 3);

		if (uas->is_px4()) {
			/**
			 * Current PX4 has bug: it cuts throttle if there no velocity sp
			 * Issue #273.
			 *
			 * @todo Revesit this quirk later. Should be fixed in firmware.
			 */
			ignore_all_except_xyz_y = (1 << 11) | (7 << 6);
		}

		// ENU->NED. Issue #49.
		set_position_target_local_ned(stamp.toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_xyz_y,
				origin.y(), origin.x(), -origin.z(),
				0.0, 0.0, 0.0,
				0.0, 0.0, 0.0,
				tf::getYaw(q), 0.0);
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose, transform);
		send_setpoint_transform(transform, req->header.stamp);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointPositionPlugin, mavplugin::MavRosPlugin)
