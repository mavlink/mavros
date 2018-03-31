/**
 * @brief Odometry plugin
 * @file odom.cpp
 * @author James Goppert <james.goppert@gmail.com>
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 James Goppert, Nuno Marques
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::MAV_ESTIMATOR_TYPE;

/**
 * @brief Odometry plugin
 *
 * Send odometry info to FCU position and attitude estimators
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

		// frame tf params
		odom_nh.param<std::string>("frame_tf/desired_frame", desired_frame, "ned");

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

	//! child if for tf
	std::string desired_frame;

	/* -*- callbacks -*- */

	void odom_cb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		/** Lookup transforms
		 *  @todo Implement in a more general fashion in the API IOT apply frame transforms
		 *  This should also run in parallel on a thread
		 */
		Eigen::Affine3d tf_child2ned;
		Eigen::Affine3d tf_child2fcu_frd;
		Eigen::Affine3d tf_parent2ned;
		try {
			// transform lookup WRT world frame
			tf_child2ned = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
							odom->child_frame_id, "local_origin_" + desired_frame, ros::Time(0)));
			tf_child2fcu_frd = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
							odom->child_frame_id, "fcu_frd", ros::Time(0)));

			// transform lookup WRT body frame
			tf_parent2ned = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
							odom->header.frame_id, "local_origin_" + desired_frame, ros::Time(0)));
		} catch (tf2::TransformException &ex) {
			ROS_WARN("odom: %s",ex.what());
			ros::Duration(1.0).sleep();
			return;
		}

		/** Build 9x9 covariance matrix to be transformed and sent
		 */
		ftf::Covariance9d cov_full {};	// zero initialized
		ftf::EigenMapCovariance9d cov_full_map(cov_full.data());

		cov_full_map.block<6, 6>(0, 0) = ftf::EigenMapConstCovariance6d(odom->pose.covariance.data());
		cov_full_map.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

		/** Build 6x6 covariance matrix to be transformed and sent
		 */
		ftf::Covariance6d cov_vel {};	// zero initialized
		ftf::EigenMapCovariance6d cov_vel_map(cov_vel.data());

		/** Apply linear transforms
		 */
		Eigen::Vector3d pos_ned(tf_parent2ned.linear() * ftf::to_eigen(odom->pose.pose.position));
		Eigen::Vector3d lin_vel_ned(tf_child2ned.linear() * ftf::to_eigen(odom->twist.twist.linear));
		Eigen::Vector3d ang_vel_fcu_frd(tf_child2fcu_frd.linear() * ftf::to_eigen(odom->twist.twist.angular));
		Eigen::Quaterniond q_parent2child(ftf::to_eigen(odom->pose.pose.orientation));
		Eigen::Affine3d tf_ned2fcu_frd = tf_parent2ned * q_parent2child * tf_child2fcu_frd.inverse();
		Eigen::Quaterniond q_ned2fcu_frd(tf_ned2fcu_frd.linear());

		/** Apply covariance transforms */
		/** Pose+Accel 9-D Covariance matrix
		 *  WRT world frame (half upper right triangular)
		 */
		Eigen::Matrix<double, 9, 9> r_9d;
		r_9d.setZero(); // initialize with zeros
		r_9d.block<3, 3>(0, 0) = r_9d.block<3, 3>(3, 3) = tf_parent2ned.linear();
		r_9d.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

		cov_full_map = r_9d * cov_full_map * r_9d.transpose();

		auto urt_view = Eigen::Matrix<double, 9, 9>(cov_full_map.triangularView<Eigen::Upper>());
		ROS_DEBUG_STREAM_NAMED("odom", "Odometry: pose+accel covariance matrix: " << std::endl << cov_full_map);
		ROS_DEBUG_STREAM_NAMED("odom", "Odometry: Cov URT: " << std::endl << urt_view);

		/** Velocity 6-D Covariance matrix
		 *  WRT body frame
		 */
		Eigen::Matrix<double, 6, 6> r_vel;
		r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = tf_child2ned.linear();

		cov_vel_map = r_vel * cov_vel_map * r_vel.transpose();

		ROS_DEBUG_STREAM_NAMED("odom", "Odometry: velocity covariance matrix: " << std::endl << cov_vel_map);

		// get timestamp
		uint64_t stamp_usec = odom->header.stamp.toNSec() / 1e3;

		const auto zerov3f = Eigen::Vector3f::Zero();

		/* -*- LOCAL_POSITION_NED_COV parser -*- */
		mavlink::common::msg::LOCAL_POSITION_NED_COV lpos {};

		lpos.time_usec = stamp_usec;
		lpos.estimator_type = utils::enum_value(estimator_type);
		ftf::covariance_urt_to_mavlink(cov_full_map, lpos.covariance);

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
		ftf::quaternion_to_mavlink(q_ned2fcu_frd, att.q);
		ftf::covariance_to_mavlink(cov_vel, att.covariance);

		// [[[cog:
		// for a, b in zip("xyz", ('rollspeed', 'pitchspeed', 'yawspeed')):
		//     cog.outl("att.{b} = ang_vel_fcu_frd.{a}();".format(**locals()))
		// ]]]
		att.rollspeed = ang_vel_fcu_frd.x();
		att.pitchspeed = ang_vel_fcu_frd.y();
		att.yawspeed = ang_vel_fcu_frd.z();
		// [[[end]]] (checksum: e100d5c18a64c243df616f342f712ca1)

		// send ATTITUDE_QUATERNION_COV
		UAS_FCU(m_uas)->send_message_ignore_drop(att);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
