/**
 * @brief SetpointPosition plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/SetMavFrame.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <GeographicLib/Geocentric.hpp>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief Setpoint position plugin
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointPositionPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointPositionPlugin>,
	private plugin::SetPositionTargetGlobalIntMixin<SetpointPositionPlugin>,
	private plugin::TF2ListenerMixin<SetpointPositionPlugin> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SetpointPositionPlugin() : PluginBase(),
		sp_nh("~setpoint_position"),
		spg_nh("~"),
		tf_rate(50.0),
		tf_listen(false)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "target_position");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);

		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("setpoint", "Listen to position setpoint transform "
					<< tf_frame_id << " -> " << tf_child_frame_id);
			tf2_start("PositionSpTF", &SetpointPositionPlugin::transform_cb);
		}
		else {
			setpoint_sub = sp_nh.subscribe("local", 10, &SetpointPositionPlugin::setpoint_cb, this);
			// Subscriber for goal gps
			setpointg_sub = sp_nh.subscribe("global", 10, &SetpointPositionPlugin::setpointg_cb, this);
			// Subscriber for goal gps but will convert it to local coordinates
			setpointg2l_sub = sp_nh.subscribe("global_to_local", 10, &SetpointPositionPlugin::setpointg2l_cb, this);
			// subscriber for current gps state, mavros/global_position/global.
			gps_sub = spg_nh.subscribe("global_position/global", 10, &SetpointPositionPlugin::gps_cb, this);
			// Subscribe for current local ENU pose.
			local_sub = spg_nh.subscribe("local_position/pose", 10, &SetpointPositionPlugin::local_cb, this);
		}
		mav_frame_srv = sp_nh.advertiseService("mav_frame", &SetpointPositionPlugin::set_mav_frame_cb, this);

		// mav_frame
		std::string mav_frame_str;
		if (!sp_nh.getParam("mav_frame", mav_frame_str)) {
			mav_frame = MAV_FRAME::LOCAL_NED;
		} else {
			mav_frame = utils::mav_frame_from_str(mav_frame_str);
		}
	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	friend class SetPositionTargetGlobalIntMixin;
	friend class TF2ListenerMixin;

	ros::NodeHandle sp_nh;
	ros::NodeHandle spg_nh;		//!< to get local position and gps coord which are not under sp_h()
	ros::Subscriber setpoint_sub;
	ros::Subscriber setpointg_sub;	//!< Global setpoint
	ros::Subscriber setpointg2l_sub;//!< Global setpoint converted to local setpoint
	ros::Subscriber gps_sub;	//!< current GPS
	ros::Subscriber local_sub;	//!< current local ENU
	ros::ServiceServer mav_frame_srv;

	/* Stores current gps state. */
	//sensor_msgs::NavSatFix current_gps_msg;
	Eigen::Vector3d current_gps;		//!< geodetic coordinates LLA
	Eigen::Vector3d current_local_pos;	//!< Current local position in ENU
	uint32_t old_gps_stamp = 0;		//!< old time gps time stamp in [ms], to check if new gps msg is received

	std::string tf_frame_id;
	std::string tf_child_frame_id;

	bool tf_listen;
	double tf_rate;

	MAV_FRAME mav_frame;

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send setpoint to FCU position controller.
	 *
	 * @warning Send only XYZ, Yaw. ENU frame.
	 */
	void send_position_target(const ros::Time &stamp, const Eigen::Affine3d &tr)
	{
		using mavlink::common::MAV_FRAME;

		/* Documentation start from bit 1 instead 0;
		 * Ignore velocity and accel vectors, yaw rate.
		 *
		 * In past versions on PX4 there been bug described in #273.
		 * If you got similar issue please try update firmware first.
		 */
		const uint16_t ignore_all_except_xyz_y = (1 << 11) | (7 << 6) | (7 << 3);

		auto p = [&] () {
				if (static_cast<MAV_FRAME>(mav_frame) == MAV_FRAME::BODY_NED || static_cast<MAV_FRAME>(mav_frame) == MAV_FRAME::BODY_OFFSET_NED) {
					return ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(tr.translation()));
				} else {
					return ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
				}
			} ();

		auto q = [&] () {
				if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
					return ftf::transform_orientation_absolute_frame_aircraft_baselink(Eigen::Quaterniond(tr.rotation()));
				} else {
					return ftf::transform_orientation_enu_ned(
						ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));
				}
			} ();

		set_position_target_local_ned(stamp.toNSec() / 1000000,
			utils::enum_value(mav_frame),
			ignore_all_except_xyz_y,
			p,
			Eigen::Vector3d::Zero(),
			Eigen::Vector3d::Zero(),
			ftf::quaternion_get_yaw(q), 0.0);
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */
	void transform_cb(const geometry_msgs::TransformStamped &transform)
	{
		Eigen::Affine3d tr;
		// TODO: later, when tf2 5.12 will be released need to revisit and replace this to
		// tf2::convert()
		tf::transformMsgToEigen(transform.transform, tr);

		send_position_target(transform.header.stamp, tr);
	}

	void setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr &req)
	{
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		send_position_target(req->header.stamp, tr);
	}

	/**
	 * Gets setpoint position setpoint and send SET_POSITION_TARGET_GLOBAL_INT
	 */
	void setpointg_cb(const geographic_msgs::GeoPoseStamped::ConstPtr &req)
	{
		using mavlink::common::POSITION_TARGET_TYPEMASK;

		uint16_t type_mask = uint16_t(POSITION_TARGET_TYPEMASK::VX_IGNORE)
					| uint16_t(POSITION_TARGET_TYPEMASK::VY_IGNORE)
					| uint16_t(POSITION_TARGET_TYPEMASK::VZ_IGNORE)
					| uint16_t(POSITION_TARGET_TYPEMASK::AX_IGNORE)
					| uint16_t(POSITION_TARGET_TYPEMASK::AY_IGNORE)
					| uint16_t(POSITION_TARGET_TYPEMASK::AZ_IGNORE);

		Eigen::Quaterniond attitude;
		tf::quaternionMsgToEigen(req->pose.orientation, attitude);
		Eigen::Quaterniond q = ftf::transform_orientation_enu_ned(
						ftf::transform_orientation_baselink_aircraft(attitude));

		set_position_target_global_int(
					req->header.stamp.toNSec() / 1000000,
					uint8_t(MAV_FRAME::GLOBAL_INT),
					type_mask,
					req->pose.position.latitude * 1e7,
					req->pose.position.longitude * 1e7,
					req->pose.position.altitude,
					Eigen::Vector3d::Zero(),
					Eigen::Vector3d::Zero(),
					ftf::quaternion_get_yaw(q),
					0);
	}

	/**
	 * Gets gps setpoint, converts it to local ENU, and sends it to FCU
	 */
	void setpointg2l_cb(const geographic_msgs::GeoPoseStamped::ConstPtr &req)
	{
		/**
		 * The idea is to convert the change in LLA(goal_gps-current_gps) to change in ENU
		 * 1- convert current/goal gps points to current/goal ECEF points
		 * 2- claculate offset in ECEF frame
		 * 3- converts ECEF offset to ENU offset given current gps LLA
		 * 4- adds ENU offset to current local ENU to that will be sent to FCU
		 */

		GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

		Eigen::Vector3d goal_gps(req->pose.position.latitude, req->pose.position.longitude, req->pose.position.altitude);

		// current gps -> curent ECEF
		Eigen::Vector3d current_ecef;
		earth.Forward(current_gps.x(), current_gps.y(), current_gps.z(),
			current_ecef.x(), current_ecef.y(), current_ecef.z());

		// goal gps -> goal ECEF
		Eigen::Vector3d goal_ecef;
		earth.Forward(goal_gps.x(), goal_gps.y(), goal_gps.z(),
			goal_ecef.x(), goal_ecef.y(), goal_ecef.z());

		// get ENU offset from ECEF offset
		Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;
		Eigen::Vector3d enu_offset = ftf::transform_frame_ecef_enu(ecef_offset, current_gps);

		// prepare yaw angle
		Eigen::Affine3d sp;	// holds position setpoint
		Eigen::Quaterniond q;	// holds desired yaw

		tf::quaternionMsgToEigen(req->pose.orientation, q);

		// set position setpoint
		sp.translation() = current_local_pos + enu_offset;
		// set desired orientation
		sp.linear() = q.toRotationMatrix();

		// Only send if current gps is updated, to avoid divergence
		if ((req->header.stamp.toNSec() / 1000000) > old_gps_stamp) {
			old_gps_stamp = req->header.stamp.toNSec() / 1000000;
			send_position_target(req->header.stamp, sp);
		}
		else {
			ROS_WARN_THROTTLE_NAMED(10, "spgp", "SPG: sp not sent.");
		}
	}

	/**
	 * Current GPS coordinates
	 */
	void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
	{
		current_gps = {msg->latitude, msg->longitude, msg->altitude};
	}

	/**
	 * current local position in ENU
	 */
	void local_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		current_local_pos = ftf::to_eigen(msg->pose.position);
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
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointPositionPlugin, mavros::plugin::PluginBase)
