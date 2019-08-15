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
using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

/**
 * @brief Odometry plugin
 *
 * Sends odometry data to the FCU position and attitude estimators.
 * @see odom_cb()
 */
class OdometryPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW 	// XXX(vooon): added to try to fix #1223. Not sure that it is needed because class do not have Eigen:: fields.

	OdometryPlugin() : PluginBase(),
		odom_nh("~odometry"),
		fcu_odom_parent_id_des("local_origin"),
		fcu_odom_child_id_des("fcu"),
		ext_odom_parent_id("camera_odom_frame"),
		ext_odom_child_id("camera_pose_frame")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// frame params:
		odom_nh.param<std::string>("in/fcu_odom_parent_id_des", fcu_odom_parent_id_des, "local_origin");
		odom_nh.param<std::string>("out/fcu_odom_child_id_des", fcu_odom_child_id_des, "fcu");
		odom_nh.param<std::string>("in/ext_odom_parent_id", ext_odom_parent_id, "camera_odom_frame");
		odom_nh.param<std::string>("out/ext_odom_child_id", ext_odom_child_id, "camera_pose_frame");

		// boost::algorithm::to_lower(local_frame_out);
		// boost::algorithm::to_lower(body_frame_orientation_out_desired);

		// publishers
		odom_pub = odom_nh.advertise<nav_msgs::Odometry>("in", 10);
		odom_mav_pub = odom_nh.advertise<nav_msgs::Odometry>("out_to_mavlink", 10);


		// subscribers
		odom_sub = odom_nh.subscribe("out", 10, &OdometryPlugin::odom_cb, this);

		// //! Map from param string to local MAV_FRAME
		// const std::unordered_map<std::string, MAV_FRAME> lf_map {
		// 	{ "vision_enu", MAV_FRAME::VISION_ENU },
		// 	{ "vision_ned", MAV_FRAME::VISION_NED },
		// 	{ "mocap_enu", MAV_FRAME::MOCAP_ENU },
		// 	{ "mocap_ned", MAV_FRAME::MOCAP_NED }
		// };

		// //! Map from param string to body MAV_FRAME
		// const std::unordered_map<std::string, MAV_FRAME> bf_map {
		// 	{ "frd", MAV_FRAME::BODY_FRD },
		// 	{ "flu", MAV_FRAME::BODY_FLU }
		// };

		// // Determine input frame_id naming
		// if (local_frame_in == "local_origin_ned")
		// 	local_frame_orientation_in = local_frame_in;
		// else
		// 	ROS_FATAL_NAMED("odom", "ODOM: invalid input local frame \"%s\"", local_frame_in.c_str());

		// // Determine output frame_id naming - considering the ROS msg frame_id
		// // as "odom" by default
		// auto lf_it = lf_map.find(local_frame_out);
		// if (lf_it != lf_map.end()) {
		// 	lf_id = lf_it->second;
		// 	auto orient = local_frame_out.substr(local_frame_out.length() - 3);
		// 	if (orient != "enu") {
		// 		local_frame_orientation_out = "local_origin_" + orient;
		// 	} else {
		// 		local_frame_orientation_out = "local_origin";
		// 	}
		// }
		// else
		// 	ROS_FATAL_NAMED("odom", "ODOM: invalid ouput local frame \"%s\"", local_frame_out.c_str());

		// // Determine input child_frame_id naming
		// auto bf_it_in = bf_map.find(body_frame_orientation_in_desired);
		// if (bf_it_in != bf_map.end()) {
		// 	if (body_frame_orientation_in_desired != "flu")
		// 		body_frame_orientation_in_desired = "fcu_" + body_frame_orientation_in_desired;
		// 	else
		// 		body_frame_orientation_in_desired = "fcu";
		// }
		// else
		// 	ROS_FATAL_NAMED("odom", "ODOM: invalid input body frame orientation \"%s\"", body_frame_orientation_in_desired.c_str());

		// // Determine output child_frame_id naming
		// auto bf_it = bf_map.find(body_frame_orientation_out_desired);
		// if (bf_it != bf_map.end()) {
		// 	bf_id = bf_it->second;
		// 	if (body_frame_orientation_out_desired != "flu")
		// 		body_frame_orientation_out_desired = "fcu_" + body_frame_orientation_out_desired;
		// 	else
		// 		body_frame_orientation_in_desired = "fcu";
		// }
		// else
		// 	ROS_FATAL_NAMED("odom", "ODOM: invalid output body frame orientation \"%s\"", body_frame_orientation_out_desired.c_str());
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&OdometryPlugin::handle_odom)
		};
	}

private:
	ros::NodeHandle odom_nh;			//!< node handler
	ros::Publisher odom_pub;			//!< nav_msgs/Odometry publisher
	ros::Subscriber odom_sub;			//!< nav_msgs/Odometry subscriber
	ros::Publisher odom_mav_pub;			//!< nav_msgs/Odometry publisher

	std::string fcu_odom_parent_id_des;			//!< desorientation of the child frame (input data)
	std::string fcu_odom_child_id_des;			//!< orientation of the body frame (input data)
	std::string ext_odom_parent_id;				//!< parent frame of the ext odometry msg
	std::string ext_odom_child_id;				//!< child frame of the ext odometry msg

	// MAV_FRAME lf_id;				//!< local frame (pose) ID
	// MAV_FRAME bf_id;				//!< body frame (pose) ID

	/**
	 * @brief Lookup transform with error handling
	 * @param[in] &frameA The parent frame of the transformation you want to get
	 * @param[in] &frameB The child frame of the transformation you want to get
	 * @param[in,out] &tf_A2B The affine transform from the frameA to frameB
	 */
	void transform_lookup(const std::string &target, const std::string &source,
		 Eigen::Affine3d &tf_source2target)
	{
		try {
			// transform lookup WRT local frame
			tf_source2target = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
				target, source,
				ros::Time(0)));
		} catch (tf2::TransformException &ex) {
			ROS_ERROR_THROTTLE_NAMED(1, "odom", "ODOM: Ex: %s", ex.what());
			return;
		}
	}

	/**
	 * @brief Handle ODOMETRY MAVlink message.
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#ODOMETRY
	 * @param msg	Received Mavlink msg
	 * @param att	ATTITUDE msg
	 */
	void handle_odom(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ODOMETRY &odom_msg)
	{

		/**
		 * Required affine rotations to apply transforms
		 */
		Eigen::Affine3d tf_parent2parentDes;
		Eigen::Affine3d tf_child2childDes;

		transform_lookup( fcu_odom_parent_id_des, "local_origin_ned", tf_parent2parentDes );
		transform_lookup( fcu_odom_child_id_des,  "fcu_frd",          tf_child2childDes );

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
		 * Position parsing to parentDes
		 */
		position = Eigen::Vector3d(tf_parent2parentDes.linear() * Eigen::Vector3d(odom_msg.x, odom_msg.y, odom_msg.z));
		tf::pointEigenToMsg(position, odom->pose.pose.position);

		/**
		 * Orientation parsing
		 */
		Eigen::Quaterniond q_parent2child(ftf::mavlink_to_quaternion(odom_msg.q));
		Eigen::Affine3d tf_parentDes2childDes = tf_parent2parentDes * q_parent2child * tf_child2childDes.inverse();
		orientation = Eigen::Quaterniond(tf_parentDes2childDes.linear());
		tf::quaternionEigenToMsg(orientation, odom->pose.pose.orientation);

		r_pose.block<3, 3>(0, 0) = r_pose.block<3, 3>(3, 3) = tf_parent2parentDes.linear();

		/**
		 * Velocities parsing
		 * Linear and angular velocities have to be in the desired child_frame.
		 */
		
		lin_vel = Eigen::Vector3d(tf_child2childDes.linear() * Eigen::Vector3d(odom_msg.vx, odom_msg.vy, odom_msg.vz));
		ang_vel = Eigen::Vector3d(tf_child2childDes.linear() * Eigen::Vector3d(odom_msg.rollspeed, odom_msg.pitchspeed, odom_msg.yawspeed));
		r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = tf_child2childDes.linear();
	
 

		tf::vectorEigenToMsg(lin_vel, odom->twist.twist.linear);
		tf::vectorEigenToMsg(ang_vel, odom->twist.twist.angular);

		/**
		 * Covariances parsing
		 */
		//! Transform pose covariance matrix
		cov_pose = r_pose * cov_pose * r_pose.transpose();
		Eigen::Map<Matrix6d>(odom->pose.covariance.data(), cov_pose.rows(), cov_pose.cols()) = cov_pose;

		//! Transform twist covariance matrix
		cov_vel = r_vel * cov_vel * r_vel.transpose();
		Eigen::Map<Matrix6d>(odom->twist.covariance.data(), cov_vel.rows(), cov_vel.cols()) = cov_vel;

		//! Publish the data
		odom_pub.publish(odom);
	}

	/**
	 * @brief Sends odometry data msgs to the FCU.
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#ODOMETRY
	 * @param req	received Odometry msg
	 */
	void odom_cb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		/**
		 * Required affine rotations to apply transforms
		 */
		Eigen::Affine3d tf_parent2parentDes;
		Eigen::Affine3d tf_child2childDes;

		transform_lookup( "vision_ned", ext_odom_parent_id, tf_parent2parentDes );
		transform_lookup( "fcu_frd",    ext_odom_child_id,  tf_child2childDes );

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

		/**
		 * Position and orientation are in the same frame as frame_id.
		 * For a matter of simplicity, and given the existent MAV_FRAME
		 * enum values, the default frame_id will be a local frame of
		 * reference, so the pose is WRT a local frame.
		 */
		position = Eigen::Vector3d(tf_parent2parentDes.linear() * ftf::to_eigen(odom->pose.pose.position));

		// Orientation represented by a quaternion rotation from the local frame to XYZ body frame
		Eigen::Quaterniond q_parent2child(ftf::to_eigen(odom->pose.pose.orientation));
		Eigen::Affine3d tf_parentDes2childDes = tf_parent2parentDes * q_parent2child * tf_child2childDes.inverse();
		orientation = Eigen::Quaterniond(tf_parentDes2childDes.linear());

		r_pose.block<3, 3>(0, 0) = r_pose.block<3, 3>(3, 3) = tf_parent2parentDes.linear();

		msg.frame_id = utils::enum_value(MAV_FRAME::VISION_NED);

		/**
		 * Linear and angular velocities has to be in the desired child frame
		 * Same logic here applies as above.
		 */
		lin_vel = Eigen::Vector3d(tf_child2childDes.linear() * ftf::to_eigen(odom->twist.twist.linear));
		ang_vel = Eigen::Vector3d(tf_child2childDes.linear() * ftf::to_eigen(odom->twist.twist.angular));
		r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = tf_child2childDes.linear();

		msg.child_frame_id = utils::enum_value(MAV_FRAME::BODY_FRD);
	
		/** Apply covariance transforms */
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

		nav_msgs::Odometry debug_odom;
		debug_odom.header = odom->header;
		debug_odom.header.frame_id = "vision_ned";
		debug_odom.child_frame_id = "fcu_frd";
		tf::pointEigenToMsg(position,debug_odom.pose.pose.position);
		tf::quaternionEigenToMsg(orientation,debug_odom.pose.pose.orientation);
		tf::vectorEigenToMsg(lin_vel,debug_odom.twist.twist.linear);
		tf::vectorEigenToMsg(ang_vel,debug_odom.twist.twist.angular);

		odom_mav_pub.publish(debug_odom);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
