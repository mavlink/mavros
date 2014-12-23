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
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

namespace mavplugin {

/**
 * @brief Local position plugin.
 * Publish local position to TF and PositionStamped,
 * send local position and visual position estimates to FCU.
 */
class LocalPositionPlugin : public MavRosPlugin {
public:
	LocalPositionPlugin() :
		uas(nullptr),
		send_tf(false)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		pos_nh = ros::NodeHandle(nh, "position");

		pos_nh.param("local/send_tf", send_tf, true);
		pos_nh.param<std::string>("local/frame_id", frame_id, "local_origin");
		pos_nh.param<std::string>("local/child_frame_id", child_frame_id, "fcu");

		local_position = pos_nh.advertise<geometry_msgs::PoseStamped>("local", 10);
	}

	std::string const get_name() const {
		return "LocalPosition";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_LOCAL_POSITION_NED, &LocalPositionPlugin::handle_local_position_ned)
		};
	}

private:
	UAS *uas;

	ros::NodeHandle pos_nh;
	ros::Publisher local_position;
	tf::TransformBroadcaster tf_broadcaster;

	std::string frame_id;		//!< origin for TF
	std::string child_frame_id;	//!< frame for TF and Pose
	bool send_tf;

	void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(msg, &pos_ned);

		ROS_DEBUG_THROTTLE_NAMED(10, "position", "Local position NED: boot_ms:%06d "
				"position:(%1.3f %1.3f %1.3f) speed:(%1.3f %1.3f %1.3f)",
				pos_ned.time_boot_ms,
				pos_ned.x, pos_ned.y, pos_ned.z,
				pos_ned.vx, pos_ned.vy, pos_ned.vz);

		/* TODO: check convertion to ENU
		 * I think XZY is not body-fixed, but orientation does.
		 * Perhaps this adds additional errorprone to us.
		 * Need more tests. Issue #49.
		 *
		 * orientation in ENU, body-fixed
		 */
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pos_ned.y, pos_ned.x, -pos_ned.z));
		transform.setRotation(uas->get_attitude_orientation());

		geometry_msgs::PoseStampedPtr pose = boost::make_shared<geometry_msgs::PoseStamped>();

		tf::poseTFToMsg(transform, pose->pose);
		pose->header.frame_id = frame_id;
		pose->header.stamp = ros::Time::now();

		if (send_tf)
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						transform,
						pose->header.stamp,
						frame_id, child_frame_id));

		local_position.publish(pose);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LocalPositionPlugin, mavplugin::MavRosPlugin)


