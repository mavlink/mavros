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
 * Copyright 2017 James Goppert
 * Copyright 2017,2018 Nuno Marques
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/algorithm/string.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::MAV_FRAME;
using mavlink::common::MAV_ESTIMATOR_TYPE;
using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

/**
 * @brief Odometry plugin
 *
 * Sends odometry data to the FCU estimator and
 * publishes odometry data that comes from FCU.
 *
 * This plugin is following ROS REP 147. Pose is expressed in parent frame. (Quaternion rotates from child to parent)
 * The twist is expressed in the child frame.
 *
 * @see odom_cb()	transforming and sending odometry to fcu
 * @see handle_odom()	receiving and transforming odometry from fcu
 */
class OdometryPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW		// XXX(vooon): added to try to fix #1223. Not sure that it is needed because class do not have Eigen:: fields.

	OdometryPlugin() : PluginBase(),
		odom_nh("~odometry"),
		fcu_odom_parent_id_des("map"),
		fcu_odom_child_id_des("base_link")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// frame params:
		odom_nh.param<std::string>("fcu/odom_parent_id_des", fcu_odom_parent_id_des, "map");
		odom_nh.param<std::string>("fcu/odom_child_id_des", fcu_odom_child_id_des, "base_link");

		// publishers
		odom_pub = odom_nh.advertise<nav_msgs::Odometry>("in", 10);

		// subscribers
		odom_sub = odom_nh.subscribe("out", 1, &OdometryPlugin::odom_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&OdometryPlugin::handle_odom)
		};
	}

private:
	ros::NodeHandle odom_nh;			//!< node handler
	ros::Publisher odom_pub;			//!< nav_msgs/Odometry publisher
	ros::Subscriber odom_sub;			//!< nav_msgs/Odometry subscriber

	std::string fcu_odom_parent_id_des;			//!< desired orientation of the fcu odometry message's parent frame
	std::string fcu_odom_child_id_des;			//!< desired orientation of the fcu odometry message's child frame

	/**
	 * @brief Lookup static transform with error handling
	 * @param[in] &target The parent frame of the transformation you want to get
	 * @param[in] &source The child frame of the transformation you want to get
	 * @param[in,out] &tf_source2target The affine transform from the source to target
	 */
	void lookup_static_transform(const std::string &target, const std::string &source,
				Eigen::Affine3d &tf_source2target)
	{
		try {
			// transform lookup at current time.
			tf_source2target = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
							target, source, ros::Time(0)));
		} catch (tf2::TransformException &ex) {
			ROS_ERROR_THROTTLE_NAMED(1, "odom", "ODOM: Ex: %s", ex.what());
			return;
		}
	}

	/**
	 * @brief Handle ODOMETRY MAVlink message.
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#ODOMETRY
	 *
	 * Callback for mavlink ODOMETRY messages sent from the FCU. According to the mavlink specification,
	 * all quantities are for the child frame (fcu_frd), expressed in the parent frame (local_origin_ned).
	 * To be compliant with ROS REP 147 for the published nav_msgs/Odometry, the data will be appropriately
	 * transformed and published. Frames for the publish message should be specified in specified
	 *  in the rosparams "odometry/fcu/odom_*_id_des" (set in px4_config.yaml).
	 *
	 * @param msg	Received Mavlink msg
	 * @param odom_msg	ODOMETRY msg
	 */
	void handle_odom(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ODOMETRY &odom_msg)
	{
		/**
		 * Required rotations to transform the FCU's odometry msg tto desired parent and child frame
		 */
		Eigen::Affine3d tf_parent2parent_des;
		Eigen::Affine3d tf_child2child_des;

		lookup_static_transform(fcu_odom_parent_id_des, "map_ned", tf_parent2parent_des);
		lookup_static_transform( fcu_odom_child_id_des, "base_link_frd", tf_child2child_des);

		//! Build 6x6 pose covariance matrix to be transformed and sent
		Matrix6d cov_pose = Matrix6d::Zero();
		ftf::mavlink_urt_to_covariance_matrix(odom_msg.pose_covariance, cov_pose);

		//! Build 6x6 velocity covariance matrix to be transformed and sent
		Matrix6d cov_vel = Matrix6d::Zero();
		ftf::mavlink_urt_to_covariance_matrix(odom_msg.velocity_covariance, cov_vel);

		Eigen::Vector3d position {};		//!< Position vector. WRT frame_id
		Eigen::Quaterniond orientation {};	//!< Attitude quaternion. WRT frame_id
		Eigen::Vector3d lin_vel {};		//!< Linear velocity vector. WRT child_frame_id
		Eigen::Vector3d ang_vel {};		//!< Angular velocity vector. WRT child_frame_id
		Matrix6d r_pose = Matrix6d::Zero();	//!< Zero initialized pose 6-D Covariance matrix. WRT frame_id
		Matrix6d r_vel = Matrix6d::Zero();	//!< Zero initialized velocity 6-D Covariance matrix. WRT child_frame_id

		auto odom = boost::make_shared<nav_msgs::Odometry>();

		odom->header = m_uas->synchronized_header(fcu_odom_parent_id_des, odom_msg.time_usec);
		odom->child_frame_id = fcu_odom_child_id_des;

		/**
		 * Position parsing to desired parent
		 */
		position = Eigen::Vector3d(tf_parent2parent_des.linear() * Eigen::Vector3d(odom_msg.x, odom_msg.y, odom_msg.z));
		tf::pointEigenToMsg(position, odom->pose.pose.position);

		/**
		 * Orientation parsing. Quaternion has to be the rotation from desired child frame to desired parent frame
		 */
		Eigen::Quaterniond q_child2parent(ftf::mavlink_to_quaternion(odom_msg.q));
		Eigen::Affine3d tf_childDes2parentDes = tf_parent2parent_des * q_child2parent * tf_child2child_des.inverse();
		orientation = Eigen::Quaterniond(tf_childDes2parentDes.linear());
		tf::quaternionEigenToMsg(orientation, odom->pose.pose.orientation);

		/**
		 * Velocities parsing
		 * Linear and angular velocities are transforned to the desired child_frame.
		 */
		lin_vel = Eigen::Vector3d(tf_child2child_des.linear() * Eigen::Vector3d(odom_msg.vx, odom_msg.vy, odom_msg.vz));
		ang_vel = Eigen::Vector3d(tf_child2child_des.linear() * Eigen::Vector3d(odom_msg.rollspeed, odom_msg.pitchspeed, odom_msg.yawspeed));
		tf::vectorEigenToMsg(lin_vel, odom->twist.twist.linear);
		tf::vectorEigenToMsg(ang_vel, odom->twist.twist.angular);

		/**
		 * Covariances parsing
		 */
		//! Transform pose covariance matrix
		r_pose.block<3, 3>(0, 0) = r_pose.block<3, 3>(3, 3) = tf_parent2parent_des.linear();
		cov_pose = r_pose * cov_pose * r_pose.transpose();
		Eigen::Map<Matrix6d>(odom->pose.covariance.data(), cov_pose.rows(), cov_pose.cols()) = cov_pose;

		//! Transform twist covariance matrix
		r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = tf_child2child_des.linear();
		cov_vel = r_vel * cov_vel * r_vel.transpose();
		Eigen::Map<Matrix6d>(odom->twist.covariance.data(), cov_vel.rows(), cov_vel.cols()) = cov_vel;

		//! Publish the data
		odom_pub.publish(odom);
	}

	/**
	 * @brief Sends odometry data msgs to the FCU.
	 *
	 * Callback to odometry that should go to FCU. The frame_ids in the odom message
	 * have to fit the frames that are are added to the tf tree. The odometry message
	 * gets rotated such that the parent frame is "odom_ned" and the child frame is "base_link_frd".
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#ODOMETRY
	 * @param req	received Odometry msg
	 */
	void odom_cb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		/**
		 * Required affine rotations to apply transforms
		 */
		Eigen::Affine3d tf_parent2parent_des;
		Eigen::Affine3d tf_child2child_des;

		lookup_static_transform("odom_ned", odom->header.frame_id, tf_parent2parent_des);
		lookup_static_transform("base_link_frd", odom->child_frame_id, tf_child2child_des);

		//! Build 6x6 pose covariance matrix to be transformed and sent
		ftf::Covariance6d cov_pose = odom->pose.covariance;
		ftf::EigenMapCovariance6d cov_pose_map(cov_pose.data());

		//! Build 6x6 velocity covariance matrix to be transformed and sent
		ftf::Covariance6d cov_vel = odom->twist.covariance;
		ftf::EigenMapCovariance6d cov_vel_map(cov_vel.data());

		/** Apply transforms:
		 * According to nav_msgs/Odometry.
		 */
		Eigen::Vector3d position {};		//!< Position vector. WRT frame_id
		Eigen::Quaterniond orientation {};	//!< Attitude quaternion. WRT frame_id
		Eigen::Vector3d lin_vel {};		//!< Linear velocity vector. WRT child_frame_id
		Eigen::Vector3d ang_vel {};		//!< Angular velocity vector. WRT child_frame_id
		Matrix6d r_pose = Matrix6d::Zero();	//!< Zero initialized pose 6-D Covariance matrix. WRT frame_id
		Matrix6d r_vel = Matrix6d::Zero();	//!< Zero initialized velocity 6-D Covariance matrix. WRT child_frame_id

		mavlink::common::msg::ODOMETRY msg {};
		msg.frame_id = utils::enum_value(MAV_FRAME::LOCAL_FRD);
		msg.child_frame_id = utils::enum_value(MAV_FRAME::BODY_FRD);
		msg.estimator_type = utils::enum_value(MAV_ESTIMATOR_TYPE::VISION);

		/**
		 * Position parsing from odometry's parent frame to "LOCAL_FRD" frame.
		 */
		position = Eigen::Vector3d(tf_parent2parent_des.linear() * ftf::to_eigen(odom->pose.pose.position));

		/**
		 * Orientation parsing.
		 */
		Eigen::Quaterniond q_child2parent(ftf::to_eigen(odom->pose.pose.orientation));
		Eigen::Affine3d tf_childDes2parentDes = tf_parent2parent_des * q_child2parent * tf_child2child_des.inverse();
		orientation = Eigen::Quaterniond(tf_childDes2parentDes.linear());

		/**
		 * Linear and angular velocities are transformed to base_link_frd
		 */
		lin_vel = Eigen::Vector3d(tf_child2child_des.linear() * ftf::to_eigen(odom->twist.twist.linear));
		ang_vel = Eigen::Vector3d(tf_child2child_des.linear() * ftf::to_eigen(odom->twist.twist.angular));

		/** Apply covariance transforms */
		r_pose.block<3, 3>(0, 0) = r_pose.block<3, 3>(3, 3) = tf_parent2parent_des.linear();
		r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = tf_child2child_des.linear();
		cov_pose_map = r_pose * cov_pose_map * r_pose.transpose();
		cov_vel_map  = r_vel  * cov_vel_map  * r_vel.transpose();

		ROS_DEBUG_STREAM_NAMED("odom", "ODOM: output: pose covariance matrix:" << std::endl << cov_pose_map);
		ROS_DEBUG_STREAM_NAMED("odom", "ODOM: output: velocity covariance matrix:" << std::endl << cov_vel_map);

		/* -*- ODOMETRY msg parser -*- */
		msg.time_usec = odom->header.stamp.toNSec() / 1e3;

		// [[[cog:
		// for a, b in (('', 'position'), ('v', 'lin_vel')):
		//     for f in 'xyz':
		//         cog.outl("msg.{a}{f} = {b}.{f}();".format(**locals()))
		// for a, b in zip("xyz", ('rollspeed', 'pitchspeed', 'yawspeed')):
		//     cog.outl("msg.{b} = ang_vel.{a}();".format(**locals()))
		// ]]]
		msg.x = position.x();
		msg.y = position.y();
		msg.z = position.z();
		msg.vx = lin_vel.x();
		msg.vy = lin_vel.y();
		msg.vz = lin_vel.z();
		msg.rollspeed = ang_vel.x();
		msg.pitchspeed = ang_vel.y();
		msg.yawspeed = ang_vel.z();
		// [[[end]]] (checksum: ead24a1a6a14496c9de6c1951ccfbbd7)

		ftf::quaternion_to_mavlink(orientation, msg.q);
		ftf::covariance_urt_to_mavlink(cov_pose_map, msg.pose_covariance);
		ftf::covariance_urt_to_mavlink(cov_vel_map, msg.velocity_covariance);

		// send ODOMETRY msg
		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
