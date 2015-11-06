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
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>

namespace mavplugin {
/**
 * @brief Setpoint position plugin
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointPositionPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointPositionPlugin>,
	private TF2ListenerMixin<SetpointPositionPlugin> {
public:
	SetpointPositionPlugin() :
		sp_nh("~setpoint_position"),
		uas(nullptr),
		tf_rate(10.0)
	{ };

	void initialize(UAS &uas_)
	{
		bool tf_listen;

		uas = &uas_;

		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "aircraft");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);

		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("setpoint", "Listen to position setpoint transform " << tf_frame_id
					<< " -> " << tf_child_frame_id);
			tf2_start("PositionSpTF", &SetpointPositionPlugin::transform_cb);
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
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber setpoint_sub;

	std::string tf_frame_id;
	std::string tf_child_frame_id;
	double tf_rate;

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send setpoint to FCU position controller.
	 *
	 * @warning Send only XYZ, Yaw. ENU frame.
	 */
	void send_position_target(const ros::Time &stamp, const Eigen::Affine3d &tr) {
		/* Documentation start from bit 1 instead 0;
		 * Ignore velocity and accel vectors, yaw rate.
		 *
		 * In past versions on PX4 there been bug described in #273.
		 * If you got similar issue please try update firmware first.
		 */
		const uint16_t ignore_all_except_xyz_y = (1 << 11) | (7 << 6) | (7 << 3);

		auto p = UAS::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto q = UAS::transform_orientation_enu_ned(
				UAS::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

		set_position_target_local_ned(stamp.toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_xyz_y,
				p.x(), p.y(), p.z(),
				0.0, 0.0, 0.0,
				0.0, 0.0, 0.0,
				UAS::quaternion_get_yaw(q), 0.0);
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */
	void transform_cb(const geometry_msgs::TransformStamped &transform) {
		Eigen::Affine3d tr;
		// TODO: later, when tf2 5.12 will be released need to revisit and replace this to
		// tf2::convert()
		tf::transformMsgToEigen(transform.transform, tr);

		send_position_target(transform.header.stamp, tr);
	}

	void setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		send_position_target(req->header.stamp, tr);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointPositionPlugin, mavplugin::MavRosPlugin)
