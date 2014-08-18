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
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
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
 * to FCU position controller.
 * 
 */
class VisionPoseEstimatePlugin : public MavRosPlugin,
	private TFListenerMixin<VisionPoseEstimatePlugin> {
public:
	VisionPoseEstimatePlugin() :
		uas(nullptr),
		tf_rate(10.0)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool pose_with_covariance;
		bool listen_tf;

		uas = &uas_;
		sp_nh = ros::NodeHandle(nh, "position");

		sp_nh.param("vision/pose_with_covariance", pose_with_covariance, false);
		sp_nh.param("vision/listen_tf", listen_tf, false);
		sp_nh.param<std::string>("vision/frame_id", frame_id, "local_origin");
		sp_nh.param<std::string>("vision/child_frame_id", child_frame_id, "vision");
		sp_nh.param("vision/tf_rate_limit", tf_rate, 50.0);

		ROS_DEBUG_STREAM_NAMED("position", "Vision pose topic type: " <<
				((pose_with_covariance)? "PoseWithCovarianceStamped" : "PoseStamped"));

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("position", "Listen to vision transform " << frame_id
					<< " -> " << child_frame_id);
			tf_start("VisionTF", &VisionPoseEstimatePlugin::send_vision_transform);
		}
		else if (pose_with_covariance)
			vision_sub = sp_nh.subscribe("vision", 10, &VisionPoseEstimatePlugin::vision_cov_cb, this);
		else
			vision_sub = sp_nh.subscribe("vision", 10, &VisionPoseEstimatePlugin::vision_cb, this);
	}

	const std::string get_name() const {
		return "VisionPoseEstimate";
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class TFListenerMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
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
	 * Send vision estimate transform to FCU position controller
	 */
	void send_vision_transform(const tf::Transform &transform, const ros::Time &stamp) {
		// origin and RPY in ENU frame
		tf::Vector3 position = transform.getOrigin();
		double roll, pitch, yaw;
		tf::Matrix3x3 orientation(transform.getBasis());
		orientation.getRPY(roll, pitch, yaw);

		/* Issue #60.
		 * Note: this now affects pose callbacks too, but i think its not big deal.
		 */
		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "position", "Vision: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		// TODO: check conversion. Issue #49.
		vision_position_estimate(stamp.toNSec() / 1000,
				position.y(), position.x(), -position.z(),
				roll, -pitch, -yaw); // ??? please check!
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

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VisionPoseEstimatePlugin, mavplugin::MavRosPlugin)
