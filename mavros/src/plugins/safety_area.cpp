/**
 * @brief SafetyArea plugin
 * @file safety_area.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
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
		safety_nh("~safety_area"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		bool manual_def = false;
		double p1x, p1y, p1z,
			p2x, p2y, p2z;

		uas = &uas_;

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

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
		
		/** @todo Publish SAFETY_ALLOWED_AREA message */
	}

private:
	ros::NodeHandle safety_nh;
	UAS *uas;

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
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send a safety zone (volume), which is defined by two corners of a cube,
	 * to the FCU.
	 *
	 * @note ENU frame.
	 */
	void send_safety_set_allowed_area(float p1x, float p1y, float p1z,
			float p2x, float p2y, float p2z) {
		ROS_INFO_NAMED("safetyarea", "SA: Set safty area: P1(%f %f %f) P2(%f %f %f)",
				p1x, p1y, p1z,
				p2x, p2y, p2z);

		auto p1 = UAS::transform_frame_enu_ned(Eigen::Vector3d(p1x, p1y, p1z));
		auto p2 = UAS::transform_frame_enu_ned(Eigen::Vector3d(p2x, p2y, p2z));

		safety_set_allowed_area(
				MAV_FRAME_LOCAL_NED, // TODO: use enum from lib
				p1.x(), p1.y(), p1.z(),
				p2.x(), p2.y(), p2.z());
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
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SafetyAreaPlugin, mavplugin::MavRosPlugin)
