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
		estimator_type(3)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// general params
		odom_nh.param("estimator_type", estimator_type, 3); // defaulted to VIO type

		ROS_DEBUG_STREAM_NAMED("odom", "Odometry: estimator type: " << utils::to_string_enum<MAV_ESTIMATOR_TYPE>(estimator_type));

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

	int estimator_type;

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
		auto q_ned = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));
		auto ang_vel_ned = ftf::transform_frame_aircraft_ned(ftf::transform_frame_baselink_aircraft(ang_vel_enu), q_ned);

		uint64_t stamp = odom->header.stamp.toNSec() / 1e3;

		// send LOCAL_POSITION_NED_COV
		mavlink::common::msg::LOCAL_POSITION_NED_COV lpos{};

		lpos.time_usec = stamp;
		lpos.estimator_type = estimator_type;
		const auto zero = Eigen::Vector3d::Zero();

		// [[[cog:
		// for a, b in (('', 'pos_ned'), ('v', 'lin_vel_ned'), ('a', 'zero')):
		//     for f in 'xyz':
		//         cog.outl("lpos.{a}{f} = {b}.{f}();".format(**locals()))
		// ]]]
		lpos.x = pos_ned.x();
		lpos.y = pos_ned.y();
		lpos.z = pos_ned.z();
		lpos.vx = lin_vel_ned.x();
		lpos.vy = lin_vel_ned.y();
		lpos.vz = lin_vel_ned.z();
		lpos.ax = zero.x();
		lpos.ay = zero.y();
		lpos.az = zero.z();
		// [[[end]]] (checksum: 9488aaf03177126873421eb108d5ac77)

		/**
		 * The transform of the pose covariance matrix is splited into two different
		 * transform, as position is relative to world frame and
		 * rotation is relative to body frame
		 */
		Eigen::MatrixXd cov_pos(3, 3);
		Eigen::MatrixXd cov_rot(3, 3);

		auto cov_pose_ = odom->pose.covariance.data();

		cov_pos <<
			cov_pose_[0] , cov_pose_[1] , cov_pose_[2] ,
			cov_pose_[3] , cov_pose_[4] , cov_pose_[5] ,
			cov_pose_[6] , cov_pose_[7] , cov_pose_[8] ;

		cov_rot <<
			cov_pose_[21] , cov_pose_[22] , cov_pose_[23] ,
			cov_pose_[25] , cov_pose_[26] , cov_pose_[27] ,
			cov_pose_[33] , cov_pose_[34] , cov_pose_[35] ;

		auto cov_pos_out = ftf::transform_frame_enu_ned(cov_pos);
		auto cov_rot_out = ftf::transform_frame_aircraft_ned(ftf::transform_frame_aircraft_baselink(cov_rot), q_ned);

		ftf::Covariance3d cov_pose;
		ftf::EigenMapCovariance3d cov_pos_rpy(cov_pose.data());

		cov_pos_rpy << cov_pos_out, Eigen::MatrixXd::Zero(3, 3),
			      Eigen::MatrixXd::Zero(3, 3), cov_rot_out;

		ftf::covariance_to_mavlink(cov_pose, lpos.covariance);

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

		/**
		 * The transform to the covariance matrix is splited into two different
		 * transform, as linear velocity is relative to world frame and
		 * angular velocity is relative to body frame
		 */
		Eigen::MatrixXd cov_ang_vel(3, 3);
		Eigen::MatrixXd cov_lin_vel(3, 3);

		auto cov_twist = odom->twist.covariance.data();

		cov_lin_vel <<
			cov_twist[0] , cov_twist[1] , cov_twist[2] ,
			cov_twist[3] , cov_twist[4] , cov_twist[5] ,
			cov_twist[6] , cov_twist[7] , cov_twist[8] ;

		cov_ang_vel <<
			cov_twist[21] , cov_twist[22] , cov_twist[23] ,
			cov_twist[25] , cov_twist[26] , cov_twist[27] ,
			cov_twist[33] , cov_twist[34] , cov_twist[35] ;

		auto cov_lin_vel_out = ftf::transform_frame_enu_ned(cov_lin_vel);
		auto cov_ang_vel_out = ftf::transform_frame_aircraft_ned(ftf::transform_frame_aircraft_baselink(cov_ang_vel), q_ned);

		ftf::Covariance3d cov_vel;
		ftf::EigenMapCovariance3d cov_vel_(cov_vel.data());

		cov_vel_ << cov_lin_vel_out, Eigen::MatrixXd::Zero(3, 3),
			      Eigen::MatrixXd::Zero(3, 3), cov_ang_vel_out;

		ftf::covariance_to_mavlink(cov_vel, att.covariance);

		UAS_FCU(m_uas)->send_message_ignore_drop(att);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
