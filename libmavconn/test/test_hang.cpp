/**
 * Test mavconn library
 *
 * This test created for issue #72.
 * It is hand test, no build rules.
 *
 * Compile command:
 *     g++ -I/opt/ros/kinetic/include -I$HOME/ros/install/include -Iinclude \
 *     -std=c++11 -ggdb test/test_hang.cpp -o /tmp/hang \
 *     -L/opt/ros/kinetic/lib -L$HOME/ros/devel/lib -lroscpp -lrosconsole -lboost_system -lmavconn -lrt \
 *     -DMAVLINK_DIALECT=ardupilotmega
 */

#include <ros/ros.h>

#include <mavconn/interface.h>
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
	server->message_received += [&](const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) { server->send_message(msg, sysid, compid); };
	//server->message_received.connect([&](const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) { server->send_message(msg, sysid, compid); });

	// create client
	client.reset(new MAVConnUDP(44, 200, "0.0.0.0", 45001, "localhost", 45000));
	client->message_received += recv_message;
	//client->message_received.connect(recv_message);

	while (ros::ok()) {
		send_heartbeat(client.get());
	}
	return 0;
}
