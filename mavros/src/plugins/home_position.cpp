/**
 * @brief HomePosition plugin
 * @file home_position.cpp
 * @author Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Thomas Stastny, Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/HomePosition.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief home position plugin.
 *
 * Publishes home position.
 */
class HomePositionPlugin : public plugin::PluginBase {
public:
	HomePositionPlugin() :
		hp_nh("~home_position")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		home_position_pub = hp_nh.advertise<mavros_msgs::HomePosition>("get",10);
		home_position_sub = hp_nh.subscribe("set", 10, &HomePositionPlugin::home_position_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&HomePositionPlugin::handle_home_position),
		};
	}

private:
	ros::NodeHandle hp_nh;

	ros::Publisher home_position_pub;
	ros::Subscriber home_position_sub;

	void handle_home_position(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HOME_POSITION &home_position) {
		std_msgs::Header header;
		header.stamp = ros::Time::now();//TODO: request add time_boot_ms to msg definition
		header.frame_id = "home_position";

		auto hp = boost::make_shared<mavros_msgs::HomePosition>();

		hp->header = header;
		hp->geo.latitude = home_position.latitude / 1E7;
		hp->geo.longitude = home_position.longitude / 1E7;
		hp->geo.altitude = home_position.altitude / 1E3;	//TODO: add height above ellipsoid #693

		Eigen::Affine3d tr;

		tr.translate(ftf::transform_frame_ned_enu(
						Eigen::Vector3d(
							home_position.x,
							home_position.y,
							home_position.z)));

		tr.rotate(ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							Eigen::Quaterniond(
								home_position.q[0],
								home_position.q[1],
								home_position.q[2],
								home_position.q[3]))));

		tf::poseEigenToMsg(tr, hp->pose);

		auto hp_approach_enu = ftf::transform_frame_ned_enu(
					Eigen::Vector3d(
						home_position.approach_x,
						home_position.approach_y,
						home_position.approach_z));

		tf::vectorEigenToMsg(hp_approach_enu, hp->approach);

		home_position_pub.publish(hp);
	}

	void home_position_cb(const mavros_msgs::HomePosition::ConstPtr &req)
	{
		mavlink::common::msg::SET_HOME_POSITION hp {};

		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		hp.target_system = m_uas->get_tgt_system();

		auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));

		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(
						Eigen::Quaterniond(tr.rotation())));
		ftf::quaternion_to_mavlink(q, hp.q);

		auto approach = ftf::transform_frame_enu_ned(
					Eigen::Vector3d(
						req->approach.x,
						req->approach.y,
						req->approach.z));

		// [[[cog:
		// for f in "xyz":
		//     cog.outl("hp.%s = position.%s();" % (f, f))
		// for f in "xyz":
		//     cog.outl("hp.approach_%s = approach.%s();" % (f, f))
		// ]]]
		hp.x = position.x();
		hp.y = position.y();
		hp.z = position.z();
		hp.approach_x = approach.x();
		hp.approach_y = approach.y();
		hp.approach_z = approach.z();
		// [[[end]]] (checksum: 9b2576713d76ec4e48650956a985d3fe)

		UAS_FCU(m_uas)->send_message_ignore_drop(hp);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HomePositionPlugin, mavros::plugin::PluginBase)
