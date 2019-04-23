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
		body_frame_orientation_in_desired("flu"),
		body_frame_orientation_out_desired("frd"),
		child_frame_id("base_link"),
		frame_id("odom"),
		local_frame_in("local_origin_ned"),
		local_frame_out("vision_ned")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		odom_nh.param<std::string>("frame_id", frame_id, "odom");
		odom_nh.param<std::string>("child_frame_id", child_frame_id, "base_link");

		// frame tf params:
		odom_nh.param<std::string>("in/frame_tf/local_frame", local_frame_in, "local_origin_ned");
		odom_nh.param<std::string>("out/frame_tf/local_frame", local_frame_out, "vision_ned");
		odom_nh.param<std::string>("in/frame_tf/body_frame_orientation", body_frame_orientation_in_desired, "flu");
		odom_nh.param<std::string>("out/frame_tf/body_frame_orientation", body_frame_orientation_out_desired, "frd");

		boost::algorithm::to_lower(local_frame_out);
		boost::algorithm::to_lower(body_frame_orientation_out_desired);

		// publishers
		odom_pub = odom_nh.advertise<nav_msgs::Odometry>("in", 10);

		// subscribers
		odom_sub = odom_nh.subscribe("out", 10, &OdometryPlugin::odom_cb, this);

		//! Map from param string to local MAV_FRAME
		const std::unordered_map<std::string, MAV_FRAME> lf_map {
			{ "vision_enu", MAV_FRAME::VISION_ENU },
			{ "vision_ned", MAV_FRAME::VISION_NED },
			{ "mocap_enu", MAV_FRAME::MOCAP_ENU },
			{ "mocap_ned", MAV_FRAME::MOCAP_NED }
		};

		//! Map from param string to body MAV_FRAME
		const std::unordered_map<std::string, MAV_FRAME> bf_map {
			{ "ned", MAV_FRAME::BODY_NED },
			{ "frd", MAV_FRAME::BODY_FRD },
			{ "flu", MAV_FRAME::BODY_FLU }
		};

		// Determine input frame_id naming
		if (local_frame_in == "local_origin_ned")
			local_frame_orientation_in = local_frame_in;
		else
			ROS_FATAL_NAMED("odom", "ODOM: invalid input local frame \"%s\"", local_frame_in.c_str());

		// Determine output frame_id naming - considering the ROS msg frame_id
		// as "odom" by default
		auto lf_it = lf_map.find(local_frame_out);
		if (lf_it != lf_map.end()) {
			lf_id = lf_it->second;
			auto orient = local_frame_out.substr(local_frame_out.length() - 3);
			if (orient != "enu") {
				local_frame_orientation_out = "local_origin_" + orient;
			} else {
				local_frame_orientation_out = "local_origin";
			}
		}
		else
			ROS_FATAL_NAMED("odom", "ODOM: invalid ouput local frame \"%s\"", local_frame_out.c_str());

		// Determine input child_frame_id naming
		auto bf_it_in = bf_map.find(body_frame_orientation_in_desired);
		if (bf_it_in != bf_map.end()) {
			if (body_frame_orientation_in_desired != "flu")
				body_frame_orientation_in_desired = "fcu_" + body_frame_orientation_in_desired;
			else
				body_frame_orientation_in_desired = "fcu";
		}
		else
			ROS_FATAL_NAMED("odom", "ODOM: invalid input body frame orientation \"%s\"", body_frame_orientation_in_desired.c_str());

		// Determine output child_frame_id naming
		auto bf_it = bf_map.find(body_frame_orientation_out_desired);
		if (bf_it != bf_map.end()) {
			bf_id = bf_it->second;
			if (body_frame_orientation_out_desired != "flu")
				body_frame_orientation_out_desired = "fcu_" + body_frame_orientation_out_desired;
			else
				body_frame_orientation_in_desired = "fcu";
		}
		else
			ROS_FATAL_NAMED("odom", "ODOM: invalid output body frame orientation \"%s\"", body_frame_orientation_out_desired.c_str());
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

	std::string local_frame_in;			//!< orientation and source of the input local frame
	std::string local_frame_out;			//!< orientation and source of the output local frame
	std::string local_frame_orientation_in;		//!< orientation of the local frame (input data)
	std::string local_frame_orientation_out;	//!< orientation of the local frame (output data)
	std::string body_frame_orientation_in_desired;	//!< orientation of the body frame (input data)
	std::string body_frame_orientation_out_desired;	//!< orientation of the body frame (output data)
	std::string frame_id;				//!< parent frame identifier
	std::string child_frame_id;			//!< child frame identifier

	MAV_FRAME lf_id;				//!< local frame (pose) ID
	MAV_FRAME bf_id;				//!< body frame (pose) ID

	/**
	 * @brief Lookup transforms
	 * @todo Implement in a more general fashion in the API IOT apply frame transforms
	 * This should also run in parallel on a thread
	 * @param[in] &frame_id The parent frame of reference
	 * @param[in] &child_frame_id The child frame of reference
	 * @param[in] &local_frame_orientation The desired local frame orientation
	 * @param[in] &body_frame_orientation The desired body frame orientation
	 * @param[in,out] &tf_parent2local The affine transform from the parent frame to the local frame
	 * @param[in,out] &tf_child2local The affine transform from the child frame to the local frame
	 * @param[in,out] &tf_parent2body The affine transform from the parent frame to the body frame
	 * @param[in,out] &tf_child2body The affine transform from the child frame to the body frame
	 */
	void transform_lookup(const std::string &frame_id, const std::string &child_frame_id,
		const std::string &local_frame_orientation, const std::string &body_frame_orientation,
		Eigen::Affine3d &tf_parent2local, Eigen::Affine3d &tf_child2local,
		Eigen::Affine3d &tf_parent2body, Eigen::Affine3d &tf_child2body)
	{
		try {
			// transform lookup WRT local frame
			tf_parent2local = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
				frame_id, local_frame_orientation,
				ros::Time(0)));
			tf_child2local = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
				child_frame_id, local_frame_orientation,
				ros::Time(0)));

			// transform lookup WRT body frame
			tf_parent2body = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
				frame_id, body_frame_orientation,
				ros::Time(0)));
			tf_child2body = tf2::transformToEigen(m_uas->tf2_buffer.lookupTransform(
				child_frame_id, body_frame_orientation,
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
		/*** Send fcu_frd <-> local_origin_ned transform ***/
		geometry_msgs::TransformStamped transform;
		transform.header.stamp = m_uas->synchronise_stamp(odom_msg.time_usec);
		transform.header.frame_id = "local_origin_ned";
		transform.child_frame_id = "fcu_frd";

		tf::vectorEigenToMsg(Eigen::Vector3d(odom_msg.x, odom_msg.y, odom_msg.z), transform.transform.translation);
		tf::quaternionEigenToMsg(ftf::mavlink_to_quaternion(odom_msg.q), transform.transform.rotation);

		m_uas->tf2_broadcaster.sendTransform(transform);
		/***************************************************/

		/**
		 * Required affine rotations to apply transforms
		 */
		Eigen::Affine3d tf_parent2local;
		Eigen::Affine3d tf_child2local;
		Eigen::Affine3d tf_parent2body;
		Eigen::Affine3d tf_child2body;

		//! Lookup to the required trans
		transform_lookup("local_origin_ned", "fcu_frd",
			local_frame_orientation_in, body_frame_orientation_in_desired,
			tf_parent2local, tf_child2local, tf_parent2body, tf_child2body);

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

		odom->header = m_uas->synchronized_header(frame_id, odom_msg.time_usec);
		odom->child_frame_id = child_frame_id;

		/**
		 * Position parsing
		 */
		position = Eigen::Vector3d(tf_parent2local.linear() * Eigen::Vector3d(odom_msg.x, odom_msg.y, odom_msg.z));
		tf::pointEigenToMsg(position, odom->pose.pose.position);

		/**
		 * Orientation parsing
		 */
		Eigen::Quaterniond q_parent2child(ftf::mavlink_to_quaternion(odom_msg.q));
		Eigen::Affine3d tf_local2body = tf_parent2local * q_parent2child * tf_child2body.inverse();
		orientation = Eigen::Quaterniond(tf_local2body.linear());
		tf::quaternionEigenToMsg(orientation, odom->pose.pose.orientation);

		r_pose.block<3, 3>(0, 0) = r_pose.block<3, 3>(3, 3) = tf_parent2local.linear();

		/**
		 * Velocities parsing
		 * Linear and angular velocities are in the same frame as child_frame_id.
		 */
		auto set_tf = [&](Eigen::Affine3d affineTf) {
				lin_vel = Eigen::Vector3d(affineTf.linear() * Eigen::Vector3d(odom_msg.vx, odom_msg.vy, odom_msg.vz));
				ang_vel = Eigen::Vector3d(affineTf.linear() * Eigen::Vector3d(odom_msg.rollspeed, odom_msg.pitchspeed, odom_msg.yawspeed));
				r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = affineTf.linear();
			};

		if (odom_msg.child_frame_id == odom_msg.frame_id) {
			// the child_frame_id would be the same reference frame as frame_id
			set_tf(tf_child2local);
		}
		else {
			// the child_frame_id would be the WRT a body frame reference
			set_tf(tf_child2body);
		}

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
		Eigen::Affine3d tf_parent2local;
		Eigen::Affine3d tf_child2local;
		Eigen::Affine3d tf_parent2body;
		Eigen::Affine3d tf_child2body;

		//! Build 6x6 pose covariance matrix to be transformed and sent
		transform_lookup(odom->header.frame_id, odom->child_frame_id,
			local_frame_orientation_out, body_frame_orientation_out_desired,
			tf_parent2local, tf_child2local, tf_parent2body, tf_child2body);

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
		position = Eigen::Vector3d(tf_parent2local.linear() * ftf::to_eigen(odom->pose.pose.position));

		// Orientation represented by a quaternion rotation from the local frame to XYZ body frame
		Eigen::Quaterniond q_parent2child(ftf::to_eigen(odom->pose.pose.orientation));
		Eigen::Affine3d tf_local2body = tf_parent2local * q_parent2child * tf_child2body.inverse();
		orientation = Eigen::Quaterniond(tf_local2body.linear());

		r_pose.block<3, 3>(0, 0) = r_pose.block<3, 3>(3, 3) = tf_parent2local.linear();

		msg.frame_id = utils::enum_value(lf_id);

		/**
		 * Linear and angular velocities are in the same frame as child_frame_id.
		 * Same logic here applies as above.
		 */
		auto set_tf = [&](Eigen::Affine3d affineTf, MAV_FRAME frame_id) {
				lin_vel = Eigen::Vector3d(affineTf.linear() * ftf::to_eigen(odom->twist.twist.linear));
				ang_vel = Eigen::Vector3d(affineTf.linear() * ftf::to_eigen(odom->twist.twist.angular));
				r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = affineTf.linear();

				msg.child_frame_id = utils::enum_value(frame_id);
			};

		if (odom->child_frame_id == "world" || odom->child_frame_id == "odom") {
			// the child_frame_id would be the same reference frame as frame_id
			set_tf(tf_child2local, lf_id);
		}
		else {
			// the child_frame_id would be the WRT a body frame reference
			set_tf(tf_child2body, bf_id);
		}

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
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
