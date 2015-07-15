/**
 * @brief Landing target plugin
 * @file landing_target.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace mavplugin {
/**
 * @brief Landing Target plugin
 *
 * This plugin is intended to publish the location of a landing area captured from a downward facing camera
 * to the FCU and/or receive IRLock infrared tracking data.
 */
class LandingTargetPlugin : public MavRosPlugin,
	private TF2ListenerMixin<LandingTargetPlugin> {
public:
	LandingTargetPlugin() :
		sp_nh("~landing_target"),
		uas(nullptr),
		tf_rate(10.0),
		send_tf(true),
		listen_tf(false),
		frame(1)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		sp_nh.param<std::string>("frame_id", frame_id, "landing_target");
		sp_nh.param<std::string>("mav_frame", mav_frame, "LOCAL_NED");
		sp_nh.param("target_size/xy", target_size_x, 1.0);	// in meters
		sp_nh.param("target_size/z", target_size_y, 1.0);
		// tf subsection
		sp_nh.param("tf/send", send_tf, true);
		sp_nh.param("tf/listen", listen_tf, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "landing_target");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "camera_center");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);

		frame = UAS::frame_from_str(mav_frame);	// MAV_FRAME index based on given frame name

		if (frame == -1) {
			ROS_ERROR_NAMED("landing_target", "LT: invalid MAV_FRAME %s. Please check valid frame names!", mav_frame.c_str());
			return;
		}

		land_target_pub = sp_nh.advertise<geometry_msgs::PoseStamped>("landing_target", 10);
		target_size_pub = sp_nh.advertise<geometry_msgs::Vector3>("target_size", 10);

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("landing_target", "Listen to landing_target transform " << tf_frame_id
												      << " -> " << tf_child_frame_id);
			tf2_start("LandingTargetTF", &LandingTargetPlugin::transform_cb);
		}
		else {
			land_target_sub = sp_nh.subscribe("target", 10, &LandingTargetPlugin::land_target_cb, this);
		}
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_LANDING_TARGET, &LandingTargetPlugin::handle_landing_target)
		};
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	bool send_tf;
	bool listen_tf;
	double tf_rate;
	ros::Time last_transform_stamp;

	std::string frame_id;
	std::string tf_frame_id;
	std::string tf_child_frame_id;

	ros::Publisher land_target_pub;
	ros::Publisher target_size_pub;
	ros::Subscriber land_target_sub;

	double target_size_x, target_size_y;

	std::string mav_frame;
	int frame;	

	/* -*- low-level send -*- */
	void landing_target(uint64_t time_usec,
			float angle_x,
			float angle_y,
			float distance,
			float size_x,
			float size_y,
			uint8_t target_num,
			uint8_t frame) {
		mavlink_message_t msg;
		mavlink_msg_landing_target_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_usec,
				angle_x,
				angle_y,
				distance,
				size_x,
				size_y,
				target_num,
				frame);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/*	AUXILIAR MATH: (for spherical coordinates)
	 *
	 *	d = sqrt(x^2 + y^2 + z^2)
	 *	theta = atan(y / x)
	 *	phi = atan(sqrt(x^2 + y^2) / z)
	 *
	 *	where,	theta	= angle_x	(note: notation in Mavlink/Firmware may lead to wrongly consider rectangular coordinates)
	 *		phi	= angle_y
	 *
	 *	Conversion from spherical to rectangular coordinates:
	 *	x = d * sin(phi) * cos(theta)
	 *	y = d * sin(phi) * sin(theta)
	 *	z = d * cos(phi)
	 *
	 */

	/**
	 * Send landing target transform to FCU
	 */
	void send_landing_target(const ros::Time &stamp, const Eigen::Affine3d &transf) {
		// origin position in ROS ENU frame
		auto pos = UAS::transform_frame_enu_ned(Eigen::Vector3d(transf.translation()));

		float distance = sqrt(pos.x() * pos.x() + pos.y() * pos.y() + pos.z() * pos.z());
		float phi = atan(sqrt(pos.x() * pos.x() + pos.y() * pos.y()) / pos.z());		// = angle_x
		float theta = atan(pos.y() / pos.x());		// = angle_y

		float size_x_rad = target_size_x * phi;		// assuming this is the arc length of the circle in XY-axis
		float size_y_rad = target_size_y * theta;	// assuming this is the arc length of the circle in Z-axis

		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "landing_target", "LT: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		landing_target( stamp.toNSec() / 1000,
				phi, theta,
				distance,
				size_x_rad,
				size_y_rad,
				0,		// TODO: update number depending on received frame_id
				frame);		// by default, in LOCAL_NED
	}

	void handle_landing_target(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_landing_target_t land_target;
		mavlink_msg_landing_target_decode(msg, &land_target);

		float distance = land_target.distance;
		float phi = land_target.angle_x;
		float theta = land_target.angle_y;

		auto position = UAS::transform_frame_ned_enu(Eigen::Vector3d(distance * sin(phi) * sin(theta),	// right now in NED
					distance * sin(phi) * sin(theta),
					distance * cos(phi)));
		auto orientation = Eigen::Quaterniond(1,0,0,0);	// TODO : Set pose depending on MAV_FRAME enum

		ROS_DEBUG_THROTTLE_NAMED(10, "land_target", "Landing target: "
				"frame: %s angle offset:(X: %1.3frad, Y: %1.3frad) "
				"distance: %1.3fm position:(%1.3f, %1.3f, %1.3f)",
				mavros::UAS::str_frame(static_cast<enum MAV_FRAME>(land_target.frame)).c_str(),
				phi, theta, distance, position.x(), position.y(), position.z());

		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		pose->header = uas->synchronized_header(frame_id, land_target.time_usec);

		tf::pointEigenToMsg(position, pose->pose.position);
		tf::quaternionEigenToMsg(orientation, pose->pose.orientation);

		land_target_pub.publish(pose);

		if (send_tf) {
			geometry_msgs::TransformStamped transform;

			transform.header.stamp = pose->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;

			transform.transform.rotation = pose->pose.orientation;
			tf::vectorEigenToMsg(position, transform.transform.translation);

			uas->tf2_broadcaster.sendTransform(transform);
		}

		auto tg_size = Eigen::Vector3d(land_target.size_x / phi, land_target.size_x / (M_PI - phi), land_target.size_y / theta);
		auto tg_size_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();

		tg_size_msg->header = pose->header;
		tf::vectorEigenToMsg(tg_size, tg_size_msg->vector);

		target_size_pub.publish(tg_size_msg);
		/** @todo add target_size and landing_target subscriber in copter_visualization node, so to publish a marker of the target */
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void transform_cb(const geometry_msgs::TransformStamped &transform) {
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);

		send_landing_target(transform.header.stamp, tr);
	}

	void land_target_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		send_landing_target(req->header.stamp, tr);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LandingTargetPlugin, mavplugin::MavRosPlugin)
