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

#include <diagnostic_updater/diagnostic_updater.h>
#include <mavros/mavconn_interface.h>
#include "mavconn_serial.h"
#include "mavconn_udp.h"

#include <mavros/Mavlink.h>

using namespace mavros;
using namespace mavconn;


class MavlinkDiag : public diagnostic_updater::DiagnosticTask
{
public:
	MavlinkDiag(std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		last_drop_count(0)
	{};

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		if (boost::shared_ptr<mavconn::MAVConnInterface> link = weak_link.lock()) {
			mavlink_status_t mav_status = link->get_status();

			stat.addf("Received packets:", "%u", mav_status.packet_rx_success_count);
			stat.addf("Dropped packets:", "%u", mav_status.packet_rx_drop_count);
			stat.addf("Buffer overruns:", "%u", mav_status.buffer_overrun);
			stat.addf("Parse errors:", "%u", mav_status.parse_error);
			stat.addf("Rx sequence number:", "%u", mav_status.current_rx_seq);
			stat.addf("Tx sequence number:", "%u", mav_status.current_tx_seq);

			if (mav_status.packet_rx_drop_count > last_drop_count)
				stat.summaryf(1, "%d packeges dropped since last report",
						mav_status.packet_rx_drop_count - last_drop_count);
			else
				stat.summary(0, "connected"); // TODO add HEARTBEAT check

			last_drop_count = mav_status.packet_rx_drop_count;
		} else {
			stat.summary(2, "not connected");
		}
	};

	void set_mavconn(const boost::shared_ptr<mavconn::MAVConnInterface> &link)
	{
		weak_link = link;
	};

private:
	boost::weak_ptr<mavconn::MAVConnInterface> weak_link;
	unsigned int last_drop_count;
};


class MavRos
{
public:
	MavRos(ros::NodeHandle &nh_) :
		node_handle(nh_),
		mavlink_node_handle("/mavlink"), // for compatible reasons
		serial_link_diag("FCU connection"),
		udp_link_diag("UDP bridge")
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
		node_handle.param<int>("serial_baud", serial_baud, 57600);
		node_handle.param<std::string>("bind_host", bind_host, "0.0.0.0");
		node_handle.param<int>("bind_port", bind_port, 14555);
		node_handle.param<std::string>("gcs_host", gcs_host, "");
		node_handle.param<int>("gcs_port", gcs_port, 14550);
		node_handle.param<int>("system_id", system_id, 1);
		node_handle.param<int>("component_id", component_id, MAV_COMP_ID_UDP_BRIDGE);

		diag_updater.setHardwareID("Mavlink");
		diag_updater.add(serial_link_diag);
		diag_updater.add(udp_link_diag);

		serial_link.reset(new mavconn::MAVConnSerial(system_id, component_id, serial_port, serial_baud));
		udp_link.reset(new mavconn::MAVConnUDP(system_id, component_id, bind_host, bind_port, gcs_host, gcs_port));

		mavlink_pub = mavlink_node_handle.advertise<Mavlink>("from", 1000);
		serial_link->message_received.connect(boost::bind(&MAVConnUDP::send_message, udp_link.get(), _1, _2, _3));
		serial_link->message_received.connect(boost::bind(&MavRos::mavlink_pub_cb, this, _1, _2, _3));
		serial_link_diag.set_mavconn(serial_link);

		mavlink_sub = mavlink_node_handle.subscribe("to", 1000, &MavRos::mavlink_sub_cb, this);
		udp_link->message_received.connect(boost::bind(&MAVConnSerial::send_message, serial_link.get(), _1, _2, _3));
		udp_link_diag.set_mavconn(udp_link);
	};

	~MavRos() {};

	void spin() {
		ros::Rate loop_rate(1000);
		while (node_handle.ok()) {
			ros::spinOnce();
			diag_updater.update();

			loop_rate.sleep();
		}
	};

private:
	ros::NodeHandle node_handle;
	ros::NodeHandle mavlink_node_handle;
	boost::shared_ptr<MAVConnSerial> serial_link;
	boost::shared_ptr<MAVConnUDP> udp_link;
	//std::list<std::auto_ptr<MAVPlugin> > plugins;

	ros::Publisher mavlink_pub;
	ros::Subscriber mavlink_sub;

	diagnostic_updater::Updater diag_updater;
	MavlinkDiag serial_link_diag;
	MavlinkDiag udp_link_diag;

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

