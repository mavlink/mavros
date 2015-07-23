/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
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
#include <geometry_msgs/TransformStamped.h>

namespace mavplugin {
/**
 * @brief Local position plugin.
 * Publish local position to TF and PositionStamped
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

		// general params
		lp_nh.param<std::string>("frame_id", frame_id, "fcu");
		// tf subsection
		lp_nh.param("tf/send", tf_send, true);
		lp_nh.param<std::string>("tf/frame_id", tf_frame_id, "local_origin");
		lp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "fcu");

		local_position = lp_nh.advertise<geometry_msgs::PoseStamped>("local", 10);
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

	std::string frame_id;		//!< frame for Pose
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF
	bool tf_send;

	void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(msg, &pos_ned);

		auto position = UAS::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
		auto orientation = uas->get_attitude_orientation();

		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();

		pose->header = uas->synchronized_header(frame_id, pos_ned.time_boot_ms);

		tf::pointEigenToMsg(position, pose->pose.position);
		pose->pose.orientation = orientation;

		local_position.publish(pose);

		if (tf_send) {
			geometry_msgs::TransformStamped transform;

			transform.header.stamp = pose->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;

			transform.transform.rotation = orientation;
			tf::vectorEigenToMsg(position, transform.transform.translation);

			uas->tf2_broadcaster.sendTransform(transform);
		}
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LocalPositionPlugin, mavplugin::MavRosPlugin)


