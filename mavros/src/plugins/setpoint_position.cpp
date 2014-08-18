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

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>

namespace mavplugin {

/**
 * @brief Setpoint position plugin
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointPositionPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointPositionPlugin>,
	private TFListenerMixin<SetpointPositionPlugin> {
public:
	SetpointPositionPlugin() :
		uas(nullptr),
		tf_rate(10.0)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool listen_tf;
		bool manual_def = false;
		bool srv_def = true;
		double x, y, z;

		uas = &uas_;
		sp_nh = ros::NodeHandle(nh, "setpoint");

		sp_nh.param("position/listen_tf", listen_tf, false);
		sp_nh.param<std::string>("position/frame_id", frame_id, "local_origin");
		sp_nh.param<std::string>("position/child_frame_id", child_frame_id, "setpoint");
		sp_nh.param("position/tf_rate_limit", tf_rate, 50.0);

		if (sp_nh.getParam("x", x) &&
				sp_nh.getParam("y", y) &&
				sp_nh.getParam("z", z)) {
			manual_def = true;
			ROS_DEBUG_NAMED("setpoint", "SP_pos: Manual set: local_pos(%f %f %f)",
					x, y, z);
		}
		
		else
			manual_def = false;

		if (manual_def) { // defined when running the node
			
			geometry_msgs::Pose pose;
			pose.position.x=x;
			pose.position.y=y;
			pose.position.z=z;
			
			tf::Transform transform;
			poseMsgToTF(pose, transform);
		
			send_setpoint_transform(transform, ros::Time::now());
		}
		
		else if (srv_def) // defined by mavsetp service
			pos_sub = sp_nh.subscribe("local/pos/set", 10, &SetpointPositionPlugin::position_cb, this);
		
		else {
		  
			if (listen_tf) {
				ROS_INFO_STREAM_NAMED("setpoint", "Listen to position setpoint transform " << frame_id
						<< " -> " << child_frame_id);
				tf_start("PositionSpTF", &SetpointPositionPlugin::send_setpoint_transform);
			}
			else
				pos_sub = sp_nh.subscribe("local_position", 10, &SetpointPositionPlugin::position_cb, this);
		}
	}

	const std::string get_name() const {
		return "SetpointPosition";
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	friend class TFListenerMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber pos_sub;

	std::string frame_id;
	std::string child_frame_id;

	double tf_rate;

	/* -*- mid-level helpers -*- */

	/**
	 * Send transform to FCU position controller
	 *
	 * Note: send only XYZ,
	 * 
	 */
	void send_setpoint_transform(const tf::Transform &transform, const ros::Time &stamp) {
		// ENU frame
		tf::Vector3 origin = transform.getOrigin();

		/* Documentation start from bit 1 instead 0,
		 * but implementation PX4 Firmware #1151 starts from 0
		 */
		uint16_t ignore_all_except_xyz = (7<<6)|(7<<3);

		// ENU->NED. Issue #49.
		set_position_target_local_ned(stamp.toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_xyz,
				origin.y(), origin.x(), -origin.z(),
				0.0, 0.0, 0.0,
				0.0, 0.0, 0.0);
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void position_cb(const geometry_msgs::PoseStamped::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose, transform);
		send_setpoint_transform(transform, req->header.stamp);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointPositionPlugin, mavplugin::MavRosPlugin)
