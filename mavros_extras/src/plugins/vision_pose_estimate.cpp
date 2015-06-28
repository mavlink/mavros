/**
 * @brief VisionPoseEstimate plugin
 * @file vision_pose_estimate.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace mavplugin {
/**
 * @brief Vision pose estimate plugin
 *
 * Send pose estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class VisionPoseEstimatePlugin : public MavRosPlugin,
	private TFListenerMixin<VisionPoseEstimatePlugin> {
public:
	VisionPoseEstimatePlugin() :
		sp_nh("~vision_pose"),
		uas(nullptr),
		tf_rate(10.0)
	{ };

	void initialize(UAS &uas_)
	{
		bool pose_with_covariance;
		bool listen_tf;

		uas = &uas_;

		sp_nh.param("pose_with_covariance", pose_with_covariance, false);
		sp_nh.param("listen_tf", listen_tf, false);
		sp_nh.param<std::string>("frame_id", frame_id, "local_origin");
		sp_nh.param<std::string>("child_frame_id", child_frame_id, "vision");
		sp_nh.param("tf_rate_limit", tf_rate, 50.0);

		ROS_DEBUG_STREAM_NAMED("vision_pose", "Vision pose topic type: " <<
				((pose_with_covariance) ? "PoseWithCovarianceStamped" : "PoseStamped"));

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("vision_pose", "Listen to vision transform " << frame_id
											<< " -> " << child_frame_id);
			tf_start("VisionPoseTF", &VisionPoseEstimatePlugin::send_vision_transform);
		}
		else if (pose_with_covariance)
			vision_sub = sp_nh.subscribe("pose", 10, &VisionPoseEstimatePlugin::vision_cov_cb, this);
		else
			vision_sub = sp_nh.subscribe("pose", 10, &VisionPoseEstimatePlugin::vision_cb, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class TFListenerMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber vision_sub;

	std::string frame_id;
	std::string child_frame_id;

	double tf_rate;
	ros::Time last_transform_stamp;

	/* -*- low-level send -*- */

	void vision_position_estimate(uint64_t usec,
			float x, float y, float z,
			float roll, float pitch, float yaw) {
		mavlink_message_t msg;
		mavlink_msg_vision_position_estimate_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				x,
				y,
				z,
				roll,
				pitch,
				yaw);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send vision estimate transform to FCU position controller
	 */
	void send_vision_transform(const tf::Transform &transform, const ros::Time &stamp) {
		tf::Vector3 origin = transform.getOrigin();
		double roll, pitch, yaw;
		tf::Matrix3x3 orientation(transform.getBasis());
		orientation.getRPY(roll, pitch, yaw);

		/**
		 * @warning Issue #60.
		 * This now affects pose callbacks too.
		 */
		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "vision_pose", "Vision: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		auto position = UAS::transform_frame_enu_ned_xyz(origin.x(), origin.y(), origin.z());
		tf::Vector3 rpy = UAS::transform_frame_enu_ned_attitude_rpy(roll, pitch, yaw);
		
		vision_position_estimate(stamp.toNSec() / 1000,
				position.x(), position.y(), position.z(),
				rpy.x(), rpy.y(), rpy.z());
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void vision_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose.pose, transform);
		send_vision_transform(transform, req->header.stamp);
	}

	void vision_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose, transform);
		send_vision_transform(transform, req->header.stamp);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VisionPoseEstimatePlugin, mavplugin::MavRosPlugin)
