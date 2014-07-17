/**
 * @brief SetpointPosition plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
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

#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

namespace mavplugin {

/**
 * @brief Setpoint position plugin
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointPositionPlugin : public MavRosPlugin {
public:
	SetpointPositionPlugin()
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool listen_tf;

		uas = &uas_;
		pos_nh = ros::NodeHandle(nh, "position");

		pos_nh.param("setpoint/listen_tf", listen_tf, false);
		pos_nh.param<std::string>("setpoint/frame_id", frame_id, "local_origin");
		pos_nh.param<std::string>("setpoint/child_frame_id", child_frame_id, "setpoint");

		if (listen_tf) {
			ROS_INFO_STREAM_NAMED("position", "Listen to setpoint transform " << frame_id
					<< " -> " << child_frame_id);
			boost::thread t(boost::bind(&SetpointPositionPlugin::tf_listener, this));
			mavutils::set_thread_name(t, "SetpointTF");
			tf_thread.swap(t);
		}
		else {
			setpoint_sub = pos_nh.subscribe("local_setpoint", 10, &SetpointPositionPlugin::setpoint_cb, this);
		}
	}

	const std::string get_name() const {
		return "SetpointPosition";
	}

	const std::vector<uint8_t> get_supported_messages() const {
		return { /* Rx disabled */ };
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	}

private:
	UAS *uas;

	ros::NodeHandle pos_nh;
	ros::Subscriber setpoint_sub;

	std::string frame_id;
	std::string child_frame_id;

	boost::thread tf_thread;

	/* -*- low-level send -*- */

	void local_ned_position_setpoint_external(uint32_t time_boot_ms, uint8_t coordinate_frame,
			uint16_t type_mask,
			float x, float y, float z,
			float vx, float vy, float vz,
			float afx, float afy, float afz) {
		mavlink_message_t msg;
		mavlink_msg_local_ned_position_setpoint_external_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms, // why it not usec timestamp?
				UAS_PACK_TGT(uas),
				coordinate_frame,
				type_mask,
				x, y, z,
				vz, vy, vz,
				afx, afy, afz);
		uas->mav_link->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * Send transform to FCU position controller
	 *
	 * Note: send only XYZ,
	 * velocity and af vector could be in Twist message
	 * but it needs additional work.
	 */
	void send_setpoint_transform(const tf::Transform &transform, const ros::Time &stamp) {
		// ENU frame
		tf::Vector3 origin = transform.getOrigin();

		/* Documentation start from bit 1 instead 0,
		 * but implementation PX4 Firmware #1151 starts from 0
		 */
		uint16_t ignore_all_except_xyz = (4<<6)|(4<<3);

		// TODO: check conversion. Issue #49.
		local_ned_position_setpoint_external(stamp.toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_xyz,
				origin.y(), origin.x(), -origin.z(),
				0.0, 0.0, 0.0,
				0.0, 0.0, 0.0);
	}

	/* -*- callbacks -*- */

	void tf_listener(void) {
		tf::TransformListener listener(pos_nh);
		tf::StampedTransform transform;
		while (pos_nh.ok()) {
			// Wait up to 3s for transform
			listener.waitForTransform(frame_id, child_frame_id, ros::Time(0), ros::Duration(3.0));
			try{
				listener.lookupTransform(frame_id, child_frame_id, ros::Time(0), transform);
				send_setpoint_transform(static_cast<tf::Transform>(transform), transform.stamp_);
			}
			catch (tf::TransformException ex){
				ROS_ERROR_NAMED("position", "SetpointTF: %s", ex.what());
			}
		}
	}

	void setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose, transform);
		send_setpoint_transform(transform, req->header.stamp);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointPositionPlugin, mavplugin::MavRosPlugin)
