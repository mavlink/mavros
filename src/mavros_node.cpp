/**
 * @file mavconn_interface.h
 * @author Vladimit Ermkov <voon341@gmail.com>
 */
/*
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

#include <mavros/mavconn_interface.h>
#include "mavconn_serial.h"
#include "mavconn_udp.h"

#include <mavros/Mavlink.h>

using namespace mavros;
using namespace mavconn;


class MavRos
{
public:
	MavRos(ros::NodeHandle &nh_) :
		node_handle(nh_),
		mavlink_node_handle("/mavlink") // for compatible reasons
	{
		std::string serial_port;
		int serial_baud;
		std::string bind_host;
		int bind_port;
		std::string gcs_host;
		int gcs_port;
		int system_id;
		int component_id;

		node_handle.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
		node_handle.param<int>("serial_baudrate", serial_baud, 57600);
		node_handle.param<std::string>("bind_host", bind_host, "0.0.0.0");
		node_handle.param<int>("bind_port", bind_port, 14555);
		node_handle.param<std::string>("gcs_host", gcs_host, "");
		node_handle.param<int>("gcs_port", gcs_port, 14550);
		node_handle.param<int>("system_id", system_id, 1);
		node_handle.param<int>("component_id", component_id, MAV_COMP_ID_UDP_BRIDGE);

		serial_link.reset(new mavconn::MAVConnSerial(system_id, component_id, serial_port, serial_baud));
		udp_link.reset(new mavconn::MAVConnUDP(system_id, component_id, bind_host, bind_port, gcs_host, gcs_port));

		mavlink_pub = mavlink_node_handle.advertise<Mavlink>("from", 1000);
		serial_link->message_received.connect(boost::bind(&MAVConnUDP::send_message, udp_link.get(), _1, _2, _3));
		serial_link->message_received.connect(boost::bind(&MavRos::mavlink_pub_cb, this, _1, _2, _3));

		mavlink_sub = mavlink_node_handle.subscribe("to", 1000, &MavRos::mavlink_sub_cb, this);
		udp_link->message_received.connect(boost::bind(&MAVConnSerial::send_message, serial_link.get(), _1, _2, _3));
	};

	~MavRos() {};

	void spin() {
		ros::spin();
	};

private:
	ros::NodeHandle node_handle;
	ros::NodeHandle mavlink_node_handle;
	std::auto_ptr<MAVConnSerial> serial_link;
	std::auto_ptr<MAVConnUDP> udp_link;
	//std::list<std::auto_ptr<MAVPlugin> > plugins;

	ros::Publisher mavlink_pub;
	ros::Subscriber mavlink_sub;

	void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
		Mavlink rmsg;

		rmsg.header.stamp = ros::Time::now();
		rmsg.len = mmsg->len;
		rmsg.seq = mmsg->seq;
		rmsg.sysid = mmsg->sysid;
		rmsg.compid = mmsg->compid;
		rmsg.msgid = mmsg->msgid;
		for (size_t i = 0; i < (mmsg->len + 7) / 8; i++)
			rmsg.payload64.push_back(mmsg->payload64[i]);

		mavlink_pub.publish(rmsg);
	};

	void mavlink_sub_cb(const Mavlink &rmsg) {
		mavlink_message_t mmsg;

		mmsg.msgid = rmsg.msgid;
		mmsg.len = rmsg.len;
		copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64); // TODO: add paranoic checks

		serial_link->send_message(&mmsg); // use dafault sys/comp ids
	};
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mavros");
	ros::NodeHandle nh("~");

	MavRos mavros(nh);
	mavros.spin();

	return 0;
}

