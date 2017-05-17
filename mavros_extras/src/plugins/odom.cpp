/**
 * @brief Odometry plugin
 * @file odom.cpp
 * @author James Goppert <james.goppert@gmail.com>
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
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::MAV_ESTIMATOR_TYPE;

/**
 * @brief Odometry plugin
 *
 * Send odometry info
 * to FCU position and attitude estimators
 */
class OdometryPlugin : public plugin::PluginBase {
public:
	OdometryPlugin() : PluginBase(),
		odom_nh("~odometry"),
		estimator_type(MAV_ESTIMATOR_TYPE::VIO)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// general params
		int et_i;
		odom_nh.param<int>("estimator_type", et_i, utils::enum_value(MAV_ESTIMATOR_TYPE::VIO));
		estimator_type = static_cast<MAV_ESTIMATOR_TYPE>(et_i);

		ROS_DEBUG_STREAM_NAMED("odom", "Odometry: estimator type: " << utils::to_string(estimator_type));

		// subscribers
		odom_sub = odom_nh.subscribe("odom", 10, &OdometryPlugin::odom_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle odom_nh;
	ros::Subscriber odom_sub;

	MAV_ESTIMATOR_TYPE estimator_type;

	/* -*- callbacks -*- */

	void odom_cb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		Eigen::Affine3d tr;
		Eigen::Vector3d lin_vel_enu;
		Eigen::Vector3d ang_vel_enu;

		tf::poseMsgToEigen(odom->pose.pose, tr);
		tf::vectorMsgToEigen(odom->twist.twist.linear, lin_vel_enu);
		tf::vectorMsgToEigen(odom->twist.twist.angular, ang_vel_enu);

		// get current ENU FCU orientation
		Eigen::Quaterniond enu_orientation;
		tf::quaternionMsgToEigen(m_uas->get_attitude_orientation(), enu_orientation);

		// Build 9x9 covariance matrix to be transformed and sent
		ftf::Covariance9d cov_full{};	// zero initialized
		ftf::EigenMapCovariance9d cov_full_map(cov_full.data());

		// 9x9 covariance matrix contruct
		cov_full_map.block<6, 6>(0, 0) = ftf::EigenMapConstCovariance6d(odom->pose.covariance.data());
		cov_full_map.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

		/* -*- vector transforms -*- */
		// body frame rotations must be aware of current attitude of the vehicle
		auto q_ned_current = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(enu_orientation));

		// apply coordinate frame transforms
		auto pos_ned = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto lin_vel_ned = ftf::transform_frame_enu_ned(lin_vel_enu);
		auto q_ned = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

		// angular velocity - WRT body frame
		auto ang_vel_ned = ftf::transform_frame_enu_ned(
					ftf::transform_frame_baselink_aircraft(
						ftf::transform_frame_ned_aircraft(ang_vel_enu, q_ned_current)));

		/* -*- covariance transforms -*- */
		/**
		 * @brief Pose+Accel 9-D Covariance matrix
		 * WRT world frame (half upper right triangular)
		 */
		auto cov_full_tf = ftf::transform_frame_enu_ned(cov_full);

		ftf::EigenMapConstCovariance9d cov_tf_map(cov_full_tf.data());
		auto urt_view = Eigen::Matrix<double, 9, 9>(cov_full_map.triangularView<Eigen::Upper>());
		ROS_DEBUG_STREAM_NAMED("odom", "Odometry: pose+accel covariance matrix: " << std::endl << cov_tf_map);
		ROS_DEBUG_STREAM_NAMED("odom", "Odometry: Cov URT: " << std::endl << urt_view);

		/**
		 * @brief Velocity 6-D Covariance matrix
		 * WRT world frame (half upper right triangular)
		 */
		auto cov_vel = ftf::transform_frame_enu_ned(
					ftf::transform_frame_baselink_aircraft(
						ftf::transform_frame_ned_aircraft(odom->twist.covariance, q_ned_current)));

		ftf::EigenMapConstCovariance6d cov_vel_map(cov_vel.data());
		ROS_DEBUG_STREAM_NAMED("odom", "Odometry: velocity covariance matrix: " << std::endl << cov_vel_map);

		// get timestamp
		uint64_t stamp_usec = odom->header.stamp.toNSec() / 1e3;
		const auto zerov3f = Eigen::Vector3f::Zero();

		/* -*- LOCAL_POSITION_NED_COV parser -*- */
		mavlink::common::msg::LOCAL_POSITION_NED_COV lpos {};

		lpos.time_usec = stamp_usec;
		lpos.estimator_type = utils::enum_value(estimator_type);
		ftf::covariance9d_urt_to_mavlink(cov_full_tf, lpos.covariance);

		// [[[cog:
		// for a, b in (('', 'pos_ned'), ('v', 'lin_vel_ned'), ('a', 'zerov3f')):
		//     for f in 'xyz':
		//         cog.outl("lpos.{a}{f} = {b}.{f}();".format(**locals()))
		// ]]]
		lpos.x = pos_ned.x();
		lpos.y = pos_ned.y();
		lpos.z = pos_ned.z();
		lpos.vx = lin_vel_ned.x();
		lpos.vy = lin_vel_ned.y();
		lpos.vz = lin_vel_ned.z();
		lpos.ax = zerov3f.x();
		lpos.ay = zerov3f.y();
		lpos.az = zerov3f.z();
		// [[[end]]] (checksum: 4620cfc77f3e5d768d7c23da5f56ddc6)

		// send LOCAL_POSITION_NED_COV
		UAS_FCU(m_uas)->send_message_ignore_drop(lpos);

		/* -*- ATTITUDE_QUATERNION_COV parser -*- */
		mavlink::common::msg::ATTITUDE_QUATERNION_COV att {};

		att.time_usec = stamp_usec;
		ftf::quaternion_to_mavlink(q_ned, att.q);
		ftf::covariance_to_mavlink(cov_vel, att.covariance);

		// [[[cog:
		// for a, b in zip("xyz", ('rollspeed', 'pitchspeed', 'yawspeed')):
		//     cog.outl("att.{b} = ang_vel_ned.{a}();".format(**locals()))
		// ]]]
		att.rollspeed = ang_vel_ned.x();
		att.pitchspeed = ang_vel_ned.y();
		att.yawspeed = ang_vel_ned.z();
		// [[[end]]] (checksum: e100d5c18a64c243df616f342f712ca1)

		// send ATTITUDE_QUATERNION_COV
		UAS_FCU(m_uas)->send_message_ignore_drop(att);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
