/**
 * @brief Odometry plugin
 * @file odom.cpp
 * @author James Goppert <james.goppert8@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 James Goppert
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Odometry plugin
 *
 * Send odometry info
 * to FCU position and attitude estimators
 */
class OdometryPlugin : public plugin::PluginBase {
public:
	OdometryPlugin() : PluginBase(),
		_nh("~odometry")
	{}

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// subscribers
		_odom_sub = _nh.subscribe("odom", 10, &OdometryPlugin::odom_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle _nh;
	ros::Subscriber _odom_sub;

	/* -*- callbacks -*- */

	void odom_cb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		Eigen::Affine3d tr;
		Eigen::Vector3d lin_vel_enu;
		Eigen::Vector3d ang_vel_enu;
		tf::poseMsgToEigen(odom->pose.pose, tr);
		tf::vectorMsgToEigen(odom->twist.twist.linear, lin_vel_enu);
		tf::vectorMsgToEigen(odom->twist.twist.angular, ang_vel_enu);

		// apply frame transforms
		auto pos_ned = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto lin_vel_ned = ftf::transform_frame_enu_ned(lin_vel_enu);
		auto ang_vel_ned = ftf::transform_frame_baselink_aircraft(ang_vel_enu);
		auto q_ned = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

		uint64_t stamp = odom->header.stamp.toNSec() / 1e3;

		// send LOCAL_POSITION_NED_COV
		mavlink::common::msg::LOCAL_POSITION_NED_COV lpos{};
		lpos.time_usec = stamp;
		// [[[cog:
		// for f in "xyz":
		//     cog.outl("lpos.%s = pos_ned.%s();" % (f, f))
		// for f in "xyz":
		//     cog.outl("lpos.v%s = lin_vel_ned.%s();" % (f, f))
		// for f in "xyz":
		//     cog.outl("lpos.a%s = 0.0;" % f)
		// ]]]
		lpos.x = pos_ned.x();
		lpos.y = pos_ned.y();
		lpos.z = pos_ned.z();
		lpos.vx = lin_vel_ned.x();
		lpos.vy = lin_vel_ned.y();
		lpos.vz = lin_vel_ned.z();
		lpos.ax = 0.0;
		lpos.ay = 0.0;
		lpos.az = 0.0;
		// [[[end]]] (checksum: e8d5d7d2428935f24933f5321183cea9)

		auto cov_lin_vel = ftf::transform_frame_enu_ned(odom->pose.covariance);
		ftf::covariance_to_mavlink(cov_lin_vel, lpos.covariance);

		UAS_FCU(m_uas)->send_message_ignore_drop(lpos);

		// send ATTITUDE_QUATERNION_COV
		mavlink::common::msg::ATTITUDE_QUATERNION_COV att{};
		att.time_usec = stamp;
		// [[[cog:
		// for a, b in zip("xyz", ('rollspeed', 'pitchspeed', 'yawspeed')):
		//     cog.outl("att.%s = ang_vel_ned.%s();" % (b, a))
		// ]]]
		att.rollspeed = ang_vel_ned.x();
		att.pitchspeed = ang_vel_ned.y();
		att.yawspeed = ang_vel_ned.z();
		// [[[end]]] (checksum: e100d5c18a64c243df616f342f712ca1)

		ftf::quaternion_to_mavlink(q_ned, att.q);

		auto cov_ang_vel = ftf::transform_frame_aircraft_baselink(odom->pose.covariance);
		ftf::covariance_to_mavlink(cov_ang_vel, att.covariance);

		UAS_FCU(m_uas)->send_message_ignore_drop(att);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
