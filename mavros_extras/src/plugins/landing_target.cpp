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

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

namespace mavplugin {
/**
 * @brief Landing Target plugin
 *
 * This plugin is intended to publish the location of a landing area captured from a downward facing camera
 * to the FCU and/or receive IRLock infrared tracking data.
 */
class LandingTargetPlugin : public MavRosPlugin,
	private TFListenerMixin<LandingTargetPlugin> {
public:
	LandingTargetPlugin() :
		sp_nh("~landing_target"),
		uas(nullptr),
		tf_rate(10.0),
		send_tf(true),
		listen_tf(false)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		sp_nh.param("send_tf", send_tf, true);
		sp_nh.param("listen_tf", listen_tf, false);
		sp_nh.param<std::string>("frame_id", frame_id, "landing_target");
		sp_nh.param<std::string>("child_frame_id", child_frame_id, "camera_center");
		sp_nh.param("tf_rate_limit", tf_rate, 50.0);
		sp_nh.param("target_size/x", target_size_x, 1.0);	// in meters
		sp_nh.param("target_size/y", target_size_y, 1.0);
		sp_nh.param<std::string>("mav_frame", mav_frame, "LOCAL_NED");

		land_target_pub = sp_nh.advertise<geometry_msgs::PoseStamped>("landing_target", 10);
		target_size_pub = sp_nh.advertise<geometry_msgs::Vector3>("target_size", 10);

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("landing_target", "Listen to landing_target transform " << frame_id
											<< " -> " << child_frame_id);
			tf_start("LandingTargetTF", &LandingTargetPlugin::send_landing_target);
		}

		land_target_sub = sp_nh.subscribe("target", 10, &LandingTargetPlugin::land_target_cb, this);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_LANDING_TARGET, &LandingTargetPlugin::handle_landing_target)
		};
	}

private:
	friend class TFListenerMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	bool send_tf;
	bool listen_tf;
	double tf_rate;
	ros::Time last_transform_stamp;
	tf::TransformBroadcaster tf_broadcaster;

	std::string frame_id;
	std::string child_frame_id;

	ros::Publisher land_target_pub;
	ros::Publisher target_size_pub;
	ros::Subscriber land_target_sub;

	double target_size_x, target_size_y;

	std::string mav_frame;

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
	void send_landing_target(const tf::Transform &transf, const ros::Time &stamp) {
		// origin position in ROS ENU frame
		tf::Vector3 pos = transf.getOrigin();

		float distance = sqrt(pos.x()*pos.x() + pos.y()*pos.y() + pos.z()*pos.z());
		float phi = atan(sqrt(pos.x()*pos.x() + pos.y()*pos.y()) / pos.z());	// = angle_x
		float theta = atan(pos.y()/pos.x());	// = angle_y

		float size_x_rad = target_size_x * phi;		// assuming this is the arc length of the circle in X-axis
		float size_y_rad = target_size_y * theta;	// assuming this is the arc length of the circle in Y-axis

		uint8_t frame = UAS::idx_frame(mav_frame);	// MAV_FRAME index based on given frame name
		if (frame == -1) {
			ROS_ERROR_NAMED("landing_target", "LT: invalid MAV_FRAME %s. Please check valid frame names!", mav_frame.c_str());
			return;
		}

		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "landing_target", "Target: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		landing_target( stamp.toNSec() / 1000,
				phi, -theta,	// this conversion must depend on the above below
				distance,	// TODO: add conversion to frames, on lib, according to MAV_FRAME enum
				size_x_rad,
				size_y_rad,
				0, 		// TODO: update number depending on received frame_id
				frame);		// by default, in LOCAL_NED			
	}

	void handle_landing_target(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_landing_target_t land_target;
		mavlink_msg_landing_target_decode(msg, &land_target);

		tf::Vector3 target;
		tf::Transform target_tf;
		tf::Quaternion q;

		float distance = land_target.distance;
		float phi = land_target.angle_x;
		float theta = land_target.angle_y;

		target.setX(distance * sin(phi) * sin(theta));
		target.setY(distance * sin(phi) * sin(theta));
		target.setZ(distance * cos(phi));

		target_tf.setOrigin(tf::Vector3(target.x(), -target.y(), -target.z())); // right now in NED but,
		q.setRPY (0, 0, 0);			// TODO : Set pose depending on MAV_FRAME enum
		target_tf.setRotation(q);

		ROS_DEBUG_THROTTLE_NAMED(10, "land_target", "Landing target: "
				"frame: %s angle offset:(X: %1.3frad, Y: %1.3frad) "
				"distance: %1.3fm position:(%1.3f, %1.3f, %1.3f)",
				mavros::UAS::str_frame(static_cast<enum MAV_FRAME>(land_target.frame)).c_str(),
				land_target.angle_x, land_target.angle_y, land_target.distance,
				target.x(), -target.y(), -target.z());

		std_msgs::Header header;
		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();

		tf::poseTFToMsg(target_tf, pose->pose);
		pose->header.frame_id = frame_id;
		pose->header.stamp = uas->synchronise_stamp(land_target.time_usec);

		land_target_pub.publish(pose);

		if (send_tf) 
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						target_tf,
						pose->header.stamp,
						frame_id, child_frame_id));

		auto tg_size = boost::make_shared<geometry_msgs::Vector3>();

		tg_size->x = land_target.size_x / phi;	// again, assuming this is the arc length of the circles in XY-plane
		tg_size->y = land_target.size_y / theta;
		tg_size->z = 0.0f;			// unless the target is not flat, z = 0.0

		target_size_pub.publish(tg_size);
		// TODO: add target_size and landing_target subscriber in visualization plugin, so to publish a marker of the target
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void land_target_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose, transform);
		send_landing_target(transform, req->header.stamp);
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LandingTargetPlugin, mavplugin::MavRosPlugin)
