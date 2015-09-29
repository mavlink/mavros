/**
 * @brief MAVROS GCS proxy
 * @file gcs_bridge.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <mavros/utils.h>
#include <mavros/mavlink_diag.h>
#include <mavconn/interface.h>

using namespace mavros;
using namespace mavconn;

ros::Publisher mavlink_pub;
ros::Subscriber mavlink_sub;
MAVConnInterface::Ptr gcs_link;


void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
	auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

	rmsg->header.stamp = ros::Time::now();
	mavros_msgs::mavlink::convert(*mmsg, *rmsg);
	mavlink_pub.publish(rmsg);
};

void mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg) {
	mavlink_message_t mmsg;

	if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
		gcs_link->send_message(&mmsg, rmsg->sysid, rmsg->compid);
	else
		ROS_ERROR("Packet drop: illegal payload64 size");
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gcs_bridge");
	ros::NodeHandle priv_nh("~");
	ros::NodeHandle mavlink_nh("mavlink");
	diagnostic_updater::Updater updater;
	mavros::MavlinkDiag gcs_link_diag("GCS bridge");

	std::string gcs_url;
	priv_nh.param<std::string>("gcs_url", gcs_url, "udp://@");

	try {
		gcs_link = MAVConnInterface::open_url(gcs_url);
		gcs_link_diag.set_mavconn(gcs_link);
		gcs_link_diag.set_connection_status(true);
	}
	catch (mavconn::DeviceError &ex) {
		ROS_FATAL("GCS: %s", ex.what());
		return 1;
	}

	mavlink_pub = mavlink_nh.advertise<mavros_msgs::Mavlink>("to", 10);
	gcs_link->message_received.connect(mavlink_pub_cb);

	// prefer UDPROS, but allow TCPROS too
	mavlink_sub = mavlink_nh.subscribe("from", 10, mavlink_sub_cb,
		ros::TransportHints()
			.unreliable().maxDatagramSize(1024)
			.reliable());

	// setup updater
	updater.setHardwareID(gcs_url);
	updater.add(gcs_link_diag);

	// updater spinner
	auto diag_timer = priv_nh.createTimer(ros::Duration(0.5),
			[&](const ros::TimerEvent &evt) {
				updater.update();
			});
	diag_timer.start();

	ros::spin();
	return 0;
}

