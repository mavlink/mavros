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
#include <tf2_eigen/tf2_eigen.h>

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
		// lookup transforms
		Eigen::Affine3d tf_child2ned;
		Eigen::Affine3d tf_parent2ned;
		try {
			tf_child2ned = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
				odom->child_frame_id, "local_origin_ned", ros::Time(0)));
			tf_parent2ned = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
				odom->header.frame_id, "local_origin_ned", ros::Time(0)));
		} catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			return;
		}

		// unpack data into Eigen
		Eigen::Vector3d pos_parent(
			odom->pose.pose.position.x,
			odom->pose.pose.position.y,
			odom->pose.pose.position.z);

		Eigen::Vector3d lin_vel_child(
			odom->twist.twist.linear.x,
			odom->twist.twist.linear.y,
			odom->twist.twist.linear.z);

		Eigen::Vector3d ang_vel_child(
			odom->twist.twist.angular.x,
			odom->twist.twist.angular.y,
			odom->twist.twist.angular.z);

		Eigen::Quaterniond q_parent2child(
			odom->pose.pose.orientation.w,
			odom->pose.pose.orientation.x,
			odom->pose.pose.orientation.y,
			odom->pose.pose.orientation.z);

		Eigen::Vector3d lin_vel_NED;
		Eigen::Vector3d ang_vel_ned;
		Eigen::Vector3d pos_NED;
		Eigen::Quaterniond q_parent2ned;

		Eigen::Matrix<double, 6, 6> pose_cov_parent;
		Eigen::Matrix<double, 9, 9> pose_cov_ned;
		Eigen::Matrix<double, 6, 6> twist_cov_child;
		Eigen::Matrix<double, 3, 3> att_cov_ned;

		for (int i=0; i<6; i++) for (int j=0; j<6; j++) {
			pose_cov_parent(i, j) = odom->pose.covariance[i*6 + j];
			twist_cov_child(i, j) = odom->pose.covariance[i*6 + j];
		}

		// apply transforms
		lin_vel_NED = tf_child2ned.linear()*lin_vel_child;
		ang_vel_ned = tf_child2ned.linear()*ang_vel_child;
		pos_NED = tf_parent2ned.linear()*pos_parent;
		q_parent2ned = tf_parent2ned.linear()*q_parent2child;

		// constrauct 9x6 matrix to rotate pose from parent to NED
		// note it is 9x6 since the lpos matrix has accel states
		// while ROS odometry message doesn't
		Eigen::Matrix<double, 9, 6> RPose;
		RPose.block<3, 3>(0, 0) = tf_parent2ned.linear();
		RPose.block<3, 3>(3, 3) = tf_parent2ned.linear();

		// get timestamp
		uint64_t stamp_usec = odom->header.stamp.toNSec() / 1e3;

		/* -*- LOCAL_POSITION_NED_COV parser -*- */
		mavlink::common::msg::LOCAL_POSITION_NED_COV lpos {};

		lpos.time_usec = stamp_usec;
		pose_cov_ned = RPose*pose_cov_parent*RPose.transpose();
		lpos.estimator_type = utils::enum_value(estimator_type);
		for (int i=0; i<6; i++) for (int j=0; j<6; j++) {
			lpos.covariance[i*6 + j] = pose_cov_ned(i, j);
		}

		lpos.x = pos_NED.x();
		lpos.y = pos_NED.y();
		lpos.z = pos_NED.z();
		lpos.vx = lin_vel_NED.x();
		lpos.vy = lin_vel_NED.y();
		lpos.vz = lin_vel_NED.z();
		lpos.ax = 0;
		lpos.ay = 0;
		lpos.az = 0;

		// send LOCAL_POSITION_NED_COV
		UAS_FCU(m_uas)->send_message_ignore_drop(lpos);

		/* -*- ATTITUDE_QUATERNION_COV parser -*- */
		mavlink::common::msg::ATTITUDE_QUATERNION_COV att {};

		att.time_usec = stamp_usec;
		ftf::quaternion_to_mavlink(q_parent2ned, att.q);
		att_cov_ned = tf_child2ned.linear()*twist_cov_child.block<3, 3>(0, 0)*tf_child2ned.linear().transpose();
		for (int i=0; i<3; i++) for (int j=0; j<3; j++) {
			att.covariance[i*3 + j] = att_cov_ned(i, j);
		}

		att.rollspeed = ang_vel_ned.x();
		att.pitchspeed = ang_vel_ned.y();
		att.yawspeed = ang_vel_ned.z();

		// send ATTITUDE_QUATERNION_COV
		UAS_FCU(m_uas)->send_message_ignore_drop(att);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
