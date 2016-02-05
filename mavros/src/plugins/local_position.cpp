/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
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
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

namespace mavplugin {
/**
 * @brief Local position plugin.
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class LocalPositionPlugin : public MavRosPlugin {
public:
	LocalPositionPlugin() :
		lp_nh("~local_position"),
		uas(nullptr),
		tf_send(false)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		// header frame_id.
		// default to map (world-fixed,ENU as per REP-105).
		lp_nh.param<std::string>("frame_id", frame_id, "map");
		// Important tf subsection
		// Report the transform from world to base_link here.
		lp_nh.param("tf/send", tf_send, true);
		lp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		lp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");
		// Debug tf info
		// broadcast the following transform: (can be expanded to more if desired)
		// NED -> aircraft
		lp_nh.param("tf/send_fcu",tf_send_fcu,false);

		local_position = lp_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
		local_velocity = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);
		local_odom = lp_nh.advertise<nav_msgs::Odometry>("odom",10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_LOCAL_POSITION_NED, &LocalPositionPlugin::handle_local_position_ned)
		};
	}

private:
	ros::NodeHandle lp_nh;
	UAS *uas;

	ros::Publisher local_position;
	ros::Publisher local_velocity;
	ros::Publisher local_odom;

	std::string frame_id;		//!< frame for Pose
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF
	bool tf_send;
	bool tf_send_fcu;	//!< report NED->aircraft in tf tree

	void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(msg, &pos_ned);

		//--------------- Transform FCU position and Velocity Data ---------------//
		auto enu_position = UAS::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
		auto enu_velocity = UAS::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

		//--------------- Get Odom Information ---------------//
		// Note this orientation describes baselink->ENU transform
		auto enu_orientation_msg = uas->get_attitude_orientation();
		auto baselink_angular_msg = uas->get_attitude_angular_velocity();
		Eigen::Quaterniond enu_orientation;
		tf::quaternionMsgToEigen(enu_orientation_msg,enu_orientation);
		auto baselink_linear = UAS::transform_frame_enu_baselink(enu_velocity,enu_orientation.inverse());

		//--------------- Generate Message Pointers ---------------//
		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		auto twist = boost::make_shared<geometry_msgs::TwistStamped>();
		auto odom = boost::make_shared<nav_msgs::Odometry>();

		pose->header = uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
		twist->header = pose->header;

		tf::pointEigenToMsg(enu_position, pose->pose.position);
		pose->pose.orientation = enu_orientation_msg;

		tf::vectorEigenToMsg(enu_velocity,twist->twist.linear);
		twist->twist.angular = baselink_angular_msg;

		odom->header.stamp = pose->header.stamp;
		odom->header.frame_id = tf_frame_id;
		odom->child_frame_id = tf_child_frame_id;
		tf::vectorEigenToMsg(baselink_linear,odom->twist.twist.linear);
		odom->twist.twist.angular = baselink_angular_msg;
		odom->pose.pose = pose->pose;

		//--------------- Publish Data ---------------//
		local_odom.publish(odom);
		local_position.publish(pose);
		local_velocity.publish(twist);

		if (tf_send) {
			geometry_msgs::TransformStamped transform;

			transform.header.stamp = pose->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;

			transform.transform.rotation = enu_orientation_msg;
			tf::vectorEigenToMsg(enu_position, transform.transform.translation);

			uas->tf2_broadcaster.sendTransform(transform);
		}
		if (tf_send_fcu) {
			//--------------- Report NED->aircraft transform ---------------//
			geometry_msgs::TransformStamped ned_aircraft_tf;

			ned_aircraft_tf.header.stamp = pose->header.stamp;
			ned_aircraft_tf.header.frame_id = "NED";
			ned_aircraft_tf.child_frame_id = "aircraft";

			//Don't just report the data from the mavlink message,
			//actually perform rotations to see if anything is
			//wrong.
			auto ned_position = UAS::transform_frame_enu_ned(enu_position);
			tf::vectorEigenToMsg(ned_position, ned_aircraft_tf.transform.translation);

			auto ned_orientation = UAS::transform_orientation_enu_ned(
					UAS::transform_orientation_baselink_aircraft(enu_orientation));
			tf::quaternionEigenToMsg(ned_orientation,ned_aircraft_tf.transform.rotation);
			uas->tf2_broadcaster.sendTransform(ned_aircraft_tf);
		}
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LocalPositionPlugin, mavplugin::MavRosPlugin)


