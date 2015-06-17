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

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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

		land_target_pub = sp_nh.advertise<geometry_msgs::PoseStamped>("landing_target", 10);

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("landing_target", "Listen to landing_target transform " << frame_id
											<< " -> " << child_frame_id);
			tf_start("LandingTargetTF", &LandingTargetPlugin::send_landing_target);
		}

		land_target_sub = sp_nh.subscribe("target", 10, &LandingTargetPlugin::land_target_cb, this);
		local_position_sub = sp_nh.subscribe("~position/local", 10, &LandingTargetPlugin::local_pos_cb, this);
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

	tf::Transform transform;
	ros::Time lp_time;

	std::string frame_id;
	std::string child_frame_id;

	ros::Publisher land_target_pub;
	ros::Subscriber land_target_sub;
	ros::Subscriber local_position_sub;

	/* -*- low-level send -*- */

	void landing_target(uint8_t target_num,
			uint8_t frame,
			float angle_x,
			float angle_y,
			float distance) {
		mavlink_message_t msg;
		mavlink_msg_landing_target_pack_chan(UAS_PACK_CHAN(uas), &msg,
				target_num,
				frame,
				angle_x,
				angle_y,
				distance);
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
		float theta = atan(pos.y()/pos.x());	// = angle_x
		float phi = atan(sqrt(pos.x()*pos.x() + pos.y()*pos.y()) / pos.z());	// = angle_y	

		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "landing_target", "Target: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		landing_target( 0, 		// TODO: update number depending on received frame_id
				1, 		// in NED; should user choose or it is auto-defined?
				theta, -phi,	// which may mean this angles should be adapted to frame
				distance);	// TODO: add MAV_FRAME enum to uas
	}

	void handle_landing_target(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_landing_target_t land_target;
		mavlink_msg_landing_target_decode(msg, &land_target);

		ROS_DEBUG_THROTTLE_NAMED(10, "land_target", "Landing target: "
				"frame: %d angle offset:(X: %1.3frad, Y: %1.3frad) distance: %1.3fm",
				land_target.frame, land_target.angle_x, land_target.angle_y, land_target.distance);

		tf::Vector3 target;
		tf::Vector3 local = transform.getOrigin();
		tf::Quaternion q;

		float distance = land_target.distance;
		float theta = land_target.angle_x;
		float phi = land_target.angle_y;

		target.setX(distance * sin(phi) * sin(theta));
		target.setY(distance * sin(phi) * sin(theta));
		target.setZ(distance * cos(phi));

		tf::Transform target_tf;

		target_tf.setOrigin(tf::Vector3(target.x(), -target.y(), -target.z())); // right now in NED but,
		q.setRPY (0, 0, 0);			// TODO : Set pose depending on MAV_FRAME enum
		target_tf.setRotation(q);

		std_msgs::Header header;
		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();

		tf::poseTFToMsg(target_tf, pose->pose);
		pose->header.frame_id = frame_id;
		pose->header.stamp = lp_time;

		land_target_pub.publish(pose);

		if (send_tf) 
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						target_tf,
						pose->header.stamp,
						frame_id, child_frame_id));
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void land_target_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		tf::Transform tf;
		poseMsgToTF(req->pose, tf);
		send_landing_target(transform, req->header.stamp);
	}

	void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		tf::Transform tf;
		poseMsgToTF(req->pose, tf);
		transform = tf;			// to be used by handle_landing_target()
		lp_time = req->header.stamp;	// ''
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LandingTargetPlugin, mavplugin::MavRosPlugin)