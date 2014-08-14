/**
 * @brief MAVROS UDP proxy
 * @file mavros_udp.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
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

#include <ros/ros.h>
#include <ros/console.h>

#include <mavros/utils.h>
#include <mavros/mavconn_interface.h>

#include <mavros/Mavlink.h>

using namespace mavros;
using namespace mavconn;

ros::Publisher mavlink_pub;
ros::Subscriber mavlink_sub;
boost::shared_ptr<MAVConnInterface> gcs_link;

void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
	MavlinkPtr rmsg = boost::make_shared<Mavlink>();

	rmsg->header.stamp = ros::Time::now();
	mavutils::copy_mavlink_to_ros(mmsg, rmsg);
	mavlink_pub.publish(rmsg);
};

void mavlink_sub_cb(const Mavlink::ConstPtr &rmsg) {
	mavlink_message_t mmsg;

	if (mavutils::copy_ros_to_mavlink(rmsg, mmsg))
		gcs_link->send_message(&mmsg, rmsg->sysid, rmsg->compid);
	else
		ROS_ERROR("Packet drop: illegal payload64 size");
};

int main(int argc, char *argv[])
{
	//ros::init(argc, argv, "gcs_bridge", ros::init_options::AnonymousName);
	ros::init(argc, argv, "gcs_bridge");
	ros::NodeHandle priv_nh("~");
	ros::NodeHandle mavlink_nh("/mavlink");

	std::string gcs_url;
	priv_nh.param<std::string>("gcs_url", gcs_url, "udp://@");

	gcs_link = MAVConnInterface::open_url(gcs_url);

	mavlink_pub = mavlink_nh.advertise<Mavlink>("to", 10);
	gcs_link->message_received.connect(mavlink_pub_cb);

	mavlink_sub = mavlink_nh.subscribe("from", 10, mavlink_sub_cb,
			ros::TransportHints()
				.unreliable()
				.maxDatagramSize(1024));

	ros::spin();
	return 0;
}

