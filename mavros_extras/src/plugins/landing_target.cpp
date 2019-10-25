/**
 * @brief Landing target plugin
 * @file landing_target.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015,2017,2019 Nuno Marques.
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
#include <mavros_msgs/LandingTarget.h>

namespace mavros {
namespace extra_plugins {
using mavlink::common::MAV_FRAME;
using mavlink::common::LANDING_TARGET_TYPE;

/**
 * @brief Landing Target plugin
 *
 * This plugin is intended to publish the location of a landing area captured from a downward facing camera
 * to the FCU and/or receive landing target tracking data coming from the FCU.
 */
class LandingTargetPlugin : public plugin::PluginBase,
	private plugin::TF2ListenerMixin<LandingTargetPlugin> {
public:
	LandingTargetPlugin() :
		nh("~landing_target"),
		tf_rate(10.0),
		send_tf(true),
		listen_tf(false),
		listen_lt(false),
		mav_frame("LOCAL_NED"),
		target_size_x(1.0),
		target_size_y(1.0),
		image_width(640),
		image_height(480),
		fov_x(2.0071286398),
		fov_y(2.0071286398),
		focal_length(2.8),
		land_target_type("VISION_FIDUCIAL")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// general params
		nh.param<std::string>("frame_id", frame_id, "landing_target_1");
		nh.param("listen_lt", listen_lt, false);	// subscribe to raw LadingTarget msg?
		nh.param<std::string>("mav_frame", mav_frame, "LOCAL_NED");
		frame = utils::mav_frame_from_str(mav_frame);	// MAV_FRAME index based on given frame name (If unknown, defaults to GENERIC)

		nh.param<std::string>("land_target_type", land_target_type, "VISION_FIDUCIAL");
		type = utils::landing_target_type_from_str(land_target_type);	// LANDING_TARGET_TYPE index based on given type name (If unknown, defaults to LIGHT_BEACON)

		// target size
		nh.param("target_size/x", target_size_x, 1.0);	// [meters]
		nh.param("target_size/y", target_size_y, 1.0);

		// image size
		nh.param<int>("image/width", image_width, 640);	// [pixels]
		nh.param<int>("image/height", image_height, 480);

		// camera field-of-view -> should be precised using the calibrated camera intrinsics
		nh.param("camera/fov_x", fov_x, 2.0071286398);	// default: 115 [degrees]
		nh.param("camera/fov_y", fov_y, 2.0071286398);
		// camera focal length
		nh.param("camera/focal_length", focal_length, 2.8);	//ex: OpenMV Cam M7: 2.8 [mm]

		// tf subsection
		nh.param("tf/send", send_tf, true);
		nh.param("tf/listen", listen_tf, false);
		nh.param<std::string>("tf/frame_id", tf_frame_id, frame_id);
		nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "camera_center");
		nh.param("tf/rate_limit", tf_rate, 50.0);

		land_target_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_in", 10);
		lt_marker_pub = nh.advertise<geometry_msgs::Vector3Stamped>("lt_marker", 10);

		if (listen_tf) {	// Listen to transform
			ROS_INFO_STREAM_NAMED("landing_target", "Listen to landing_target transform " << tf_frame_id
												      << " -> " << tf_child_frame_id);
			tf2_start("LandingTargetTF", &LandingTargetPlugin::transform_cb);
		}
		else if (listen_lt) {	// Subscribe to LandingTarget msg
			land_target_sub = nh.subscribe("raw", 10, &LandingTargetPlugin::landtarget_cb, this);
		}
		else {			// Subscribe to PoseStamped msg
			pose_sub = nh.subscribe("pose", 10, &LandingTargetPlugin::pose_cb, this);
		}
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&LandingTargetPlugin::handle_landing_target)
		};
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle nh;

	bool send_tf;
	bool listen_tf;
	double tf_rate;
	ros::Time last_transform_stamp;

	bool listen_lt;

	std::string frame_id;
	std::string tf_frame_id;
	std::string tf_child_frame_id;

	ros::Publisher land_target_pub;
	ros::Publisher lt_marker_pub;
	ros::Subscriber land_target_sub;
	ros::Subscriber pose_sub;

	double target_size_x, target_size_y;
	double fov_x, fov_y;
	double focal_length;
	int image_width, image_height;

	MAV_FRAME frame;
	std::string mav_frame;

	LANDING_TARGET_TYPE type;
	std::string land_target_type;

	/* -*- low-level send -*- */
	void landing_target(uint64_t time_usec,
				uint8_t target_num,
				uint8_t frame,
				Eigen::Vector2f angle,
				float distance,
				Eigen::Vector2f size,
				Eigen::Vector3d pos,
				Eigen::Quaterniond q,
				uint8_t type,
				uint8_t position_valid)
	{
		mavlink::common::msg::LANDING_TARGET lt {};

		lt.time_usec = time_usec;
		lt.target_num = target_num;
		lt.frame = frame;
		lt.distance = distance;
		lt.type = type;
		lt.position_valid = position_valid;
		lt.angle_x = angle.x();
		lt.angle_y = angle.y();
		lt.size_x = size.x();
		lt.size_y = size.y();
		lt.x = pos.x();
		lt.y = pos.y();
		lt.z = pos.z();

		ftf::quaternion_to_mavlink(q, lt.q);

		UAS_FCU(m_uas)->send_message_ignore_drop(lt);
	}

	/* -*- mid-level helpers -*- */
	/**
	 * @brief Displacement: (not to be mixed with angular displacement)
	 *
	 * WITH angle_rad = atan(y / x) * (π / 180)
	 * IF X & Y > 0: (1st quadrant)
	 *      θ_x = angle_rad
	 *      θ_y = - angle_rad
	 * IF X < 0 & Y > 0: (2nd quadrant)
	 *      θ_x = π - angle_rad
	 *      θ_y = angle_rad
	 * IF X < 0 & Y < 0: (3rd quadrant)
	 *      θ_x = π + angle_rad
	 *      θ_y = π - angle_rad
	 * IF X > 0 & Y < 0: (4th quadrant)
	 *      θ_x = - angle_rad
	 *      θ_y = π + angle_rad
	 */
	void inline cartesian_to_displacement(const Eigen::Vector3d &pos, Eigen::Vector2f &angle) {
		float angle_rad = atan(pos.y() / pos.x()) * (M_PI / 180.0);

		if (pos.x() > 0 && pos.y() > 0) {
			angle.x() = angle_rad;
			angle.y() = -angle_rad;
		}
		else if (pos.x() < 0 && pos.y() > 0) {
			angle.x() = M_PI - angle_rad;
			angle.y() = angle_rad;
		}
		else if (pos.x() < 0 && pos.y() < 0) {
			angle.x() = M_PI + angle_rad;
			angle.y() = M_PI - angle_rad;
		}
		else if (pos.x() > 0 && pos.y() < 0) {
			angle.x() = -angle_rad;
			angle.y() = M_PI + angle_rad;
		}
	}

	/**
	 * @brief Send landing target transform to FCU
	 */
	void send_landing_target(const ros::Time &stamp, const Eigen::Affine3d &tr) {
		/**
		 * @brief the position of the landing target WRT camera center - on the FCU,
		 * the position WRT to the origin local NED frame can be computed to allow
		 * the FCU to know where the landing target is in the local frame.
		 */
		auto pos = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));

		/** @brief the orientation of the landing target WRT camera frame */
		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

		Eigen::Vector2f angle;
		Eigen::Vector2f size_rad;
		Eigen::Vector2f fov;

		// the norm of the position vector is considered the distance to the landing target
		float distance = pos.norm();

		// if the landing target type is a vision type, compute the angular offsets
		if (land_target_type.find("VISION")) {
			/**
			 * @brief: the camera angular offsets can be computed by knowing the position
			 * of the target center relative to the camera center, the field-of-view of
			 * the camera and the image resolution being considered.
			 * The target size is computed by the angle of view formula (similar to angular diameter).
			 */
			angle.x() = (pos.x() - image_width / 2.0) * fov.x() / image_width;
			angle.y() = (pos.y() - image_height / 2.0) * fov.y() / image_height;
			/**
			 * @brief Angular diameter:
			 * δ = 2 * atan(d / (2 * D))
			 * where,	d = actual diameter; D = distance to the object (or focal length of a camera)
			 */
			size_rad = {2 * (M_PI / 180.0) * atan(target_size_x / (2 * focal_length)),
				    2 * (M_PI / 180.0) * atan(target_size_y / (2 * focal_length))};
		}
		// else, the same values are computed considering the displacement relative to X and Y axes of the camera frame reference
		else {
			cartesian_to_displacement(pos, angle);
			size_rad = {2 * (M_PI / 180.0) * atan(target_size_x / (2 * distance)),
				    2 * (M_PI / 180.0) * atan(target_size_y / (2 * distance))};
		}

		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "landing_target", "LT: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		auto rpy = ftf::quaternion_to_rpy(q);

		// the last char of frame_id is considered the number of the target
		uint8_t id = static_cast<uint8_t>(frame_id.back());

		ROS_DEBUG_THROTTLE_NAMED(10, "landing_target", "Tx landing target: "
					"ID: %d frame: %s angular offset: X:%1.3frad, Y:%1.3frad) "
					"distance: %1.3fm position: X:%1.3fm, Y:%1.3fm, Z:%1.3fm) "
					"orientation: roll:%1.4frad pitch:%1.4frad yaw:%1.4frad "
					"size: X:%1.3frad by Y:%1.3frad type: %s",
					id, utils::to_string(static_cast<MAV_FRAME>(frame)).c_str(),
					angle.x(), angle.y(), distance, pos.x(), pos.y(), pos.z(),
					rpy.x(), rpy.y(), rpy.z(), size_rad.x(), size_rad.y(),
					utils::to_string(static_cast<LANDING_TARGET_TYPE>(type)).c_str());

		landing_target(stamp.toNSec() / 1000,
					id,
					utils::enum_value(frame),	// by default, in LOCAL_NED
					angle,
					distance,
					size_rad,
					pos,
					q,
					utils::enum_value(type),
					1);	// position is valid from the first received msg
	}

	/**
	 * @brief Receive landing target from FCU.
	 */
	void handle_landing_target(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LANDING_TARGET &land_target) {
		/** @todo these transforms should be applied according to the MAV_FRAME */
		auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(land_target.x, land_target.y, land_target.z));
		auto orientation = ftf::transform_orientation_aircraft_baselink(
					ftf::transform_orientation_ned_enu(
						Eigen::Quaterniond(land_target.q[0], land_target.q[1], land_target.q[2], land_target.q[3])));

		auto rpy = ftf::quaternion_to_rpy(orientation);

		ROS_DEBUG_THROTTLE_NAMED(10, "landing_target", "Rx landing target: "
					"ID: %d frame: %s angular offset: X:%1.3frad, Y:%1.3frad) "
					"distance: %1.3fm position: X:%1.3fm, Y:%1.3fm, Z:%1.3fm) "
					"orientation: roll:%1.4frad pitch:%1.4frad yaw:%1.4frad "
					"size: X:%1.3frad by Y:%1.3frad type: %s",
					land_target.target_num, utils::to_string(static_cast<MAV_FRAME>(land_target.frame)).c_str(),
					land_target.angle_x, land_target.angle_y, land_target.distance,
					position.x(), position.y(), position.z(), rpy.x(), rpy.y(), rpy.z(), land_target.size_x, land_target.size_y,
					utils::to_string(static_cast<LANDING_TARGET_TYPE>(land_target.type)).c_str());

		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		pose->header = m_uas->synchronized_header(frame_id, land_target.time_usec);

		tf::pointEigenToMsg(position, pose->pose.position);
		tf::quaternionEigenToMsg(orientation, pose->pose.orientation);

		land_target_pub.publish(pose);

		if (send_tf) {
			geometry_msgs::TransformStamped transform;

			transform.header.stamp = pose->header.stamp;
			transform.header.frame_id = "landing_target_" + boost::lexical_cast<std::string>(land_target.target_num);
			transform.child_frame_id = tf_child_frame_id;

			transform.transform.rotation = pose->pose.orientation;
			tf::vectorEigenToMsg(position, transform.transform.translation);

			m_uas->tf2_broadcaster.sendTransform(transform);
		}

		auto tg_size_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
		Eigen::Vector3d target_size(target_size_x, target_size_y, 0.0);

		tf::vectorEigenToMsg(target_size, tg_size_msg->vector);

		lt_marker_pub.publish(tg_size_msg);
	}

	/* -*- callbacks -*- */
	/**
	 * @brief callback for TF2 listener
	 */
	void transform_cb(const geometry_msgs::TransformStamped &transform) {
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);

		send_landing_target(transform.header.stamp, tr);
	}

	/**
	 * @brief callback for PoseStamped msgs topic
	 */
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		send_landing_target(req->header.stamp, tr);
	}

	/**
	 * @brief callback for raw LandingTarget msgs topic - useful if one has the
	 * data processed in another node
	 */
	void landtarget_cb(const mavros_msgs::LandingTarget::ConstPtr &req) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		/** @todo these transforms should be applied according to the MAV_FRAME */
		auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto orientation = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

		landing_target(	req->header.stamp.toNSec() / 1000,
					req->target_num,
					req->frame,	// by default, in LOCAL_NED
					Eigen::Vector2f(req->angle[0], req->angle[1]),
					req->distance,
					Eigen::Vector2f(req->size[0], req->size[1]),
					position,
					orientation,
					req->type,
					1);		// position is valid from the first received msg
	}
};
}	// namespace extra_plugins
}	// namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::LandingTargetPlugin, mavros::plugin::PluginBase)
