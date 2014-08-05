/**
 * @brief SafetyArea plugin
 * @file safety_area.cpp
 * @author Nuno Marques
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

#include <geometry_msgs/PolygonStamped.h>

namespace mavplugin {

/**
 * @brief Safety allopwed area plugin
 *
 * Send safety area to FCU controller.
 */
class SafetyAreaPlugin : public MavRosPlugin {
public:
	SafetyAreaPlugin() :
		uas(nullptr)
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		bool manual_def = false;
		double p1x, p1y, p1z,
		       p2x, p2y, p2z;

		uas = &uas_;
		safety_nh = ros::NodeHandle(nh, "safety_area");

		if (safety_nh.getParam("p1/x", p1x) &&
				safety_nh.getParam("p1/y", p1y) &&
				safety_nh.getParam("p1/z", p1z)) {
			manual_def = true;
			ROS_DEBUG_NAMED("safetyarea", "SA: Manual set: P1(%f %f %f)",
					p1x, p1y, p1z);
		}

		if (manual_def &&
				safety_nh.getParam("p2/x", p2x) &&
				safety_nh.getParam("p2/y", p2y) &&
				safety_nh.getParam("p2/z", p2z)) {
			manual_def = true;
			ROS_DEBUG_NAMED("safetyarea", "SA: Manual set: P2(%f %f %f)",
					p2x, p2y, p2z);
		}
		else
			manual_def = false;

		if (manual_def)
			send_safety_set_allowed_area(
					p1x, p1y, p1z,
					p2x, p2y, p2z);

		safetyarea_sub = safety_nh.subscribe("set", 10, &SafetyAreaPlugin::safetyarea_cb, this);
	}

	const std::string get_name() const {
		return "SafetyArea";
	}

	const std::vector<uint8_t> get_supported_messages() const {
		return { /* Rx disabled */ };
		/**
		 * @todo Publish SAFETY_ALLOWED_AREA message
		 */
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	}

private:
	UAS *uas;

	ros::NodeHandle safety_nh;
	ros::Subscriber safetyarea_sub;

	/* -*- low-level send -*- */

	void safety_set_allowed_area(
			uint8_t coordinate_frame,
			float p1x, float p1y, float p1z,
			float p2x, float p2y, float p2z) {
		mavlink_message_t msg;
		mavlink_msg_safety_set_allowed_area_pack_chan(UAS_PACK_CHAN(uas), &msg,
				UAS_PACK_TGT(uas),
				coordinate_frame,
				p1x, p1y, p1z,
				p2x, p2y, p2z);
		uas->mav_link->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * Send a safety zone (volume), which is defined by two corners of a cube,
	 * to the FCU.
	 *
	 * @note ENU frame.
	 */
	void send_safety_set_allowed_area(float p1x, float p1y, float p1z,
			float p2x, float p2y, float p2z) {

		ROS_INFO_NAMED("safetyarea", "SA: Set safty area: P1(%f %f %f) P2(%f %f %f)",
				p1x, p1y, p1z,
				p2x, p2y, p2z);

		safety_set_allowed_area(
				MAV_FRAME_LOCAL_NED,
				p1y, p1x, -p1z,
				p2y, p2x, -p2z);
	}

	/* -*- callbacks -*- */

	void safetyarea_cb(const geometry_msgs::PolygonStamped::ConstPtr &req) {
		if (req->polygon.points.size() != 2) {
			ROS_ERROR_NAMED("safetyarea", "SA: Polygon should contain only two points");
			return;
		}

		send_safety_set_allowed_area(
				req->polygon.points[0].x,
				req->polygon.points[0].y,
				req->polygon.points[0].z,
				req->polygon.points[1].x,
				req->polygon.points[1].y,
				req->polygon.points[1].z);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SafetyAreaPlugin, mavplugin::MavRosPlugin)
