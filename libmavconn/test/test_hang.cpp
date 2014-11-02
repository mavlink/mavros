/**
 * Test mavconn library
 *
 * This test created for issue #72.
 * It is hand test, no build rules.
 *
 * Compile command:
 *     g++ -I /opt/ros/indigo/include -std=c++11 test/test_hang.cpp -o /tmp/hang \
 *     -L/opt/ros/indigo/lib -lroscpp -lrosconsole -lboost_system -lmavconn \
 *     -DMAVLINK_DIALECT=ardupilotmega
 */

#include <ros/ros.h>

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/udp.h>
//#include <mavconn/tcp.h>

using namespace mavconn;

static void send_heartbeat(MAVConnInterface *ip) {
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack_chan(ip->get_system_id(), ip->get_component_id(), ip->get_channel(), &msg,
			MAV_TYPE_ONBOARD_CONTROLLER,
			MAV_AUTOPILOT_INVALID,
			MAV_MODE_MANUAL_ARMED,
			0,
			MAV_STATE_ACTIVE);
	ip->send_message(&msg);
}

void recv_message(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	ROS_INFO("RECV MSG %d %d", msg->msgid, msg->seq);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "mavconn_test", ros::init_options::AnonymousName);

	std::unique_ptr<MAVConnInterface> server;
	std::unique_ptr<MAVConnInterface> client;

	// create echo server
	server.reset(new MAVConnUDP(42, 200, "0.0.0.0", 45000));
	server->message_received.connect(boost::bind(&MAVConnInterface::send_message, server.get(), _1, _2, _3));

	// create client
	client.reset(new MAVConnUDP(44, 200, "0.0.0.0", 45001, "localhost", 45000));
	client->message_received.connect(boost::bind(recv_message, _1, _2, _3));

	while(ros::ok()) {
		send_heartbeat(client.get());
	}
	return 0;
}
