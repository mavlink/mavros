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
#include <eigen_conversions/eigen_msg.h>

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
	private TF2ListenerMixin<VisionPoseEstimatePlugin> {
public:
	VisionPoseEstimatePlugin() :
		sp_nh("~vision_pose"),
		uas(nullptr),
		tf_rate(10.0)
	{ };

	void initialize(UAS &uas_)
	{
		bool tf_listen;

		uas = &uas_;

		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "vision");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);

		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("vision_pose", "Listen to vision transform " << tf_frame_id
						<< " -> " << tf_child_frame_id);
			tf2_start("VisionPoseTF", &VisionPoseEstimatePlugin::transform_cb);
		}
		else {
			vision_sub = sp_nh.subscribe("pose", 10, &VisionPoseEstimatePlugin::vision_cb, this);
			vision_cov_sub = sp_nh.subscribe("pose_cov", 10, &VisionPoseEstimatePlugin::vision_cov_cb, this);
		}
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber vision_sub;
	ros::Subscriber vision_cov_sub;

	std::string tf_frame_id;
	std::string tf_child_frame_id;
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
	void send_vision_estimate(const ros::Time &stamp, const Eigen::Affine3d &tr) {
		/**
		 * @warning Issue #60.
		 * This now affects pose callbacks too.
		 */
		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "vision_pose", "Vision: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		auto position = UAS::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto rpy = UAS::quaternion_to_rpy(
				UAS::transform_orientation_enu_ned(Eigen::Quaterniond(tr.rotation())));

		vision_position_estimate(stamp.toNSec() / 1000,
				position.x(), position.y(), position.z(),
				rpy.x(), rpy.y(), rpy.z());
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void transform_cb(const geometry_msgs::TransformStamped &transform) {
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);

		send_vision_estimate(transform.header.stamp, tr);
	}

	void vision_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		send_vision_estimate(req->header.stamp, tr);
	}

	void vision_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &req) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose.pose, tr);

		send_vision_estimate(req->header.stamp, tr);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VisionPoseEstimatePlugin, mavplugin::MavRosPlugin)
