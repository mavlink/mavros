/**
 * @brief VisionSpeedEstimate plugin
 * @file vision_speed.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
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

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace mavplugin {

/**
 * @brief Vision speed estimate plugin
 *
 * Send speed estimation from various vision estimators
 * to FCU position controller.
 * 
 */
class VisionSpeedEstimatePlugin : public MavRosPlugin {
public:
	VisionSpeedEstimatePlugin() :
		uas(nullptr)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool listen_twist;
		uas = &uas_;
		sp_nh = ros::NodeHandle(nh, "vision_speed");

		sp_nh.param("listen_twist", listen_twist, false);

		if(listen_twist)
			vision_vel_sub = sp_nh.subscribe("speed_twist", 10, &VisionSpeedEstimatePlugin::vel_twist_cb, this);
		else
			vision_vel_sub = sp_nh.subscribe("speed_vector", 10, &VisionSpeedEstimatePlugin::vel_speed_cb, this);
	}

	const std::string get_name() const {
		return "VisionSpeedEstimate";
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber vision_vel_sub;

	/* -*- low-level send -*- */

	void vision_speed_estimate(uint64_t usec,
			float x, float y, float z) {
		mavlink_message_t msg;
		mavlink_msg_vision_speed_estimate_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				x, y, z);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * Send vision speed estimate to FCU velocity controller
	 */
	void send_vision_speed(float vx, float vy, float vz, const ros::Time &stamp) {

		// TODO: check conversion. Issue #49.
		vision_speed_estimate(stamp.toNSec() / 1000,
				vy, vx, -vz);
	}

	/* -*- callbacks -*- */

	void vel_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		send_vision_speed(req->twist.linear.x, req->twist.linear.y, req->twist.linear.z, req->header.stamp);
	}

	void vel_speed_cb(const geometry_msgs::Vector3Stamped::ConstPtr &req) {
		send_vision_speed(req->vector.x, req->vector.y, req->vector.z, req->header.stamp);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VisionSpeedEstimatePlugin, mavplugin::MavRosPlugin)
