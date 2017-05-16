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

#include <geometry_msgs/PolygonStamped.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Safety allopwed area plugin
 *
 * Send safety area to FCU controller.
 */
class SafetyAreaPlugin : public plugin::PluginBase {
public:
	SafetyAreaPlugin() : PluginBase(),
		safety_nh("~safety_area")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		safety_nh.param<std::string>("frame_id", frame_id, "safety_area");

		bool manual_def = false;
		double p1x, p1y, p1z,
			p2x, p2y, p2z;

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
					Eigen::Vector3d(p1x, p1y, p1z),
					Eigen::Vector3d(p2x, p2y, p2z));

		safetyarea_sub = safety_nh.subscribe("set", 10, &SafetyAreaPlugin::safetyarea_cb, this);
		safetyarea_pub = safety_nh.advertise<geometry_msgs::PolygonStamped>("get",10);
	}

	Subscriptions get_subscriptions()
	{
		return {
		       make_handler(&SafetyAreaPlugin::handle_safety_allowed_area)
		};
	}

private:
	ros::NodeHandle safety_nh;

	std::string frame_id;

	ros::Subscriber safetyarea_sub;
	ros::Publisher safetyarea_pub;

	/* -*- rx handlers -*- */
	void handle_safety_allowed_area(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SAFETY_ALLOWED_AREA &saa)
	{
		auto saa_msg = boost::make_shared<geometry_msgs::PolygonStamped>();

		Eigen::Vector3d p1(saa.p1x, saa.p1y, saa.p1z);
		Eigen::Vector3d p2(saa.p2x, saa.p2y, saa.p2z);

		p1 = ftf::transform_frame_ned_enu(p1);
		p2 = ftf::transform_frame_ned_enu(p2);

		saa_msg->header.stamp = ros::Time::now();
		saa_msg->header.frame_id = frame_id;
		//saa_msg->header.frame_id = utils::to_string(saa.frame);	@TODO: after #311 merged this will work

		saa_msg->polygon.points.resize(2);
		saa_msg->polygon.points[0].x = p1.x();
		saa_msg->polygon.points[0].y = p1.y();
		saa_msg->polygon.points[0].z = p1.z();
		saa_msg->polygon.points[1].x = p2.x();
		saa_msg->polygon.points[1].y = p2.y();
		saa_msg->polygon.points[1].z = p2.z();

		safetyarea_pub.publish(saa_msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send a safety zone (volume), which is defined by two corners of a cube,
	 * to the FCU.
	 *
	 * @note ENU frame.
	 */
	void send_safety_set_allowed_area(Eigen::Vector3d p1, Eigen::Vector3d p2)
	{
		ROS_INFO_STREAM_NAMED("safetyarea", "SA: Set safety area: P1 " << p1 << " P2 " << p2);

		p1 = ftf::transform_frame_enu_ned(p1);
		p2 = ftf::transform_frame_enu_ned(p2);

		mavlink::common::msg::SAFETY_SET_ALLOWED_AREA s;
		m_uas->msg_set_target(s);

		// TODO: use enum from lib
		s.frame = utils::enum_value(mavlink::common::MAV_FRAME::LOCAL_NED);

		// [[[cog:
		// for p in range(1, 3):
		//     for v in ('x', 'y', 'z'):
		//         cog.outl("s.p%d%s = p%d.%s();" % (p, v, p, v))
		// ]]]
		s.p1x = p1.x();
		s.p1y = p1.y();
		s.p1z = p1.z();
		s.p2x = p2.x();
		s.p2y = p2.y();
		s.p2z = p2.z();
		// [[[end]]] (checksum: c996a362f338fcc6b714c8be583c3be0)

		UAS_FCU(m_uas)->send_message_ignore_drop(s);
	}

	/* -*- callbacks -*- */

	void safetyarea_cb(const geometry_msgs::PolygonStamped::ConstPtr &req)
	{
		if (req->polygon.points.size() != 2) {
			ROS_ERROR_NAMED("safetyarea", "SA: Polygon should contain only two points");
			return;
		}

		// eigen_conversions do not have convertor for Point32
		// [[[cog:
		// for p in range(2):
		//     cog.outl("Eigen::Vector3d p%d(%s);" % (p + 1, ', '.join([
		//         'req->polygon.points[%d].%s' % (p, v) for v in ('x', 'y', 'z')
		//         ])))
		// ]]]
		Eigen::Vector3d p1(req->polygon.points[0].x, req->polygon.points[0].y, req->polygon.points[0].z);
		Eigen::Vector3d p2(req->polygon.points[1].x, req->polygon.points[1].y, req->polygon.points[1].z);
		// [[[end]]] (checksum: c3681d584e02f7d91d6b3b48f87b1771)

		send_safety_set_allowed_area(p1, p2);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SafetyAreaPlugin, mavros::plugin::PluginBase)
