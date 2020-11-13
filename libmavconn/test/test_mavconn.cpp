/**
 * Test mavconn library
 */

// Gmock broken on Ubuntu (thrusty),
//  its gmock 1.6 require gtest 1.7, while repository only provides 1.6
//  this error exist one year without any updates.
//  https://bugs.launchpad.net/ubuntu/+source/google-mock/+bug/1201279
//
//  I think on Debian that was fixed while ago. But i can't use it in ros buildfarm.
//#include <gmock/gmock.h>
#include <gtest/gtest.h>

//#include <ros/ros.h>

#include <chrono>
#include <condition_variable>

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/udp.h>
#include <mavconn/tcp.h>

using namespace mavconn;
using mavlink::mavlink_message_t;
using mavlink::msgid_t;

namespace mavlink {
	namespace common {
		using namespace mavlink::minimal;
		namespace msg {
			using namespace mavlink::minimal::msg;
			}
	}
}

static void send_heartbeat(MAVConnInterface *ip) {
	using mavlink::common::MAV_TYPE;
	using mavlink::common::MAV_AUTOPILOT;
	using mavlink::common::MAV_MODE;
	using mavlink::common::MAV_STATE;

	mavlink::common::msg::HEARTBEAT hb = {};
	hb.type = int(MAV_TYPE::ONBOARD_CONTROLLER);
	hb.autopilot = int(MAV_AUTOPILOT::INVALID);
	hb.base_mode = int(MAV_MODE::MANUAL_ARMED);
	hb.custom_mode = 0;
	hb.system_status = int(MAV_STATE::ACTIVE);

	ip->send_message(hb);
}

class UDP : public ::testing::Test {
public:
	std::mutex mutex;
	std::condition_variable cond;

	msgid_t message_id;

	void recv_message(const mavlink_message_t *message, const Framing framing) {
		//printf("Got message %u, len: %u, framing: %d\n", message->msgid, message->len, int(framing));
		message_id = message->msgid;
		cond.notify_one();
	}

	bool wait_one() {
		std::unique_lock<std::mutex> lock(mutex);
		return cond.wait_for(lock, std::chrono::seconds(2)) == std::cv_status::no_timeout;
	}
};

#if 0
// XXX(vooon): temparary disable that check - it don't work on Travis (with ICI)
TEST_F(UDP, bind_error)
{
	MAVConnInterface::Ptr conns[2];

	conns[0] = std::make_shared<MAVConnUDP>(42, 200, "localhost", 45000);
	ASSERT_THROW(conns[1] = std::make_shared<MAVConnUDP>(42, 200, "localhost", 45000), DeviceError);
}
#endif

TEST_F(UDP, send_message)
{
	MAVConnInterface::Ptr echo, client;

	message_id = std::numeric_limits<msgid_t>::max();
	auto msgid = mavlink::common::msg::HEARTBEAT::MSG_ID;

	// create echo server
	echo = std::make_shared<MAVConnUDP>(42, 200, "0.0.0.0", 45002);
	echo->message_received_cb = [&](const mavlink_message_t * msg, const Framing framing) {
		echo->send_message(msg);
	};

	// create client
	client = std::make_shared<MAVConnUDP>(44, 200, "0.0.0.0", 45003, "localhost", 45002);
	client->message_received_cb = std::bind(&UDP::recv_message, this, std::placeholders::_1, std::placeholders::_2);

	// wait echo
	send_heartbeat(client.get());
	send_heartbeat(client.get());
	EXPECT_EQ(wait_one(), true);
	EXPECT_EQ(message_id, msgid);
}

class TCP : public UDP {};

TEST_F(TCP, bind_error)
{
	MAVConnInterface::Ptr conns[2];

	conns[0] = std::make_shared<MAVConnTCPServer>(42, 200, "localhost", 57600);
	ASSERT_THROW(conns[1] = std::make_shared<MAVConnTCPServer>(42, 200, "localhost", 57600), DeviceError);
}

TEST_F(TCP, connect_error)
{
	MAVConnInterface::Ptr client;
	ASSERT_THROW(client = std::make_shared<MAVConnTCPClient>(42, 200, "localhost", 57666), DeviceError);
}

TEST_F(TCP, send_message)
{
	MAVConnInterface::Ptr echo_server, client;

	message_id = std::numeric_limits<msgid_t>::max();
	auto msgid = mavlink::common::msg::HEARTBEAT::MSG_ID;

	// create echo server
	echo_server = std::make_shared<MAVConnTCPServer>(42, 200, "0.0.0.0", 57602);
	echo_server->message_received_cb = [&](const mavlink_message_t * msg, const Framing framing) {
		echo_server->send_message(msg);
	};

	// create client
	client = std::make_shared<MAVConnTCPClient>(44, 200, "localhost", 57602);
	client->message_received_cb = std::bind(&TCP::recv_message, this, std::placeholders::_1, std::placeholders::_2);

	// wait echo
	send_heartbeat(client.get());
	send_heartbeat(client.get());
	EXPECT_EQ(wait_one(), true);
	EXPECT_EQ(message_id, msgid);
}

TEST_F(TCP, client_reconnect)
{
	MAVConnInterface::Ptr echo_server;
	MAVConnInterface::Ptr client1, client2;

	// create echo server
	echo_server = std::make_shared<MAVConnTCPServer>(42, 200, "0.0.0.0", 57604);
	echo_server->message_received_cb = [&](const mavlink_message_t * msg, const Framing framing) {
		echo_server->send_message(msg);
	};

	EXPECT_NO_THROW({
			client1 = std::make_shared<MAVConnTCPClient>(44, 200, "localhost", 57604);
		});

	EXPECT_NO_THROW({
			client2 = std::make_shared<MAVConnTCPClient>(45, 200, "localhost", 57604);
		});

	EXPECT_NO_THROW({
			client1 = std::make_shared<MAVConnTCPClient>(46, 200, "localhost", 57604);
		});
}

TEST(SERIAL, open_error)
{
	MAVConnInterface::Ptr serial;
	ASSERT_THROW(serial = std::make_shared<MAVConnSerial>(42, 200, "/some/magic/not/exist/path", 57600), DeviceError);
}

#if 0
TEST(URL, open_url_serial)
{
	MAVConnInterface::Ptr serial;
	MAVConnSerial *serial_p;

	/* not best way to test tty access,
	 * but it does not require any preparation
	 * Disabled because it breaks terminal.
	 */
	EXPECT_NO_THROW({
			serial = MAVConnInterface::open_url("/dev/tty:115200");
			serial_p = dynamic_cast<MAVConnSerial*>(serial.get());
			EXPECT_NE(serial_p, nullptr);
		});

	EXPECT_NO_THROW({
			serial = MAVConnInterface::open_url("serial:///dev/tty:115200?ids=2,240");
			serial_p = dynamic_cast<MAVConnSerial*>(serial.get());
			EXPECT_NE(serial_p, nullptr);
		});
}
#endif

TEST(URL, open_url_udp)
{
	MAVConnInterface::Ptr udp;
	MAVConnUDP *udp_p;

	EXPECT_NO_THROW({
			udp = MAVConnInterface::open_url("udp://localhost:45004@localhost:45005/?ids=2,241");
			udp_p = dynamic_cast<MAVConnUDP*>(udp.get());
			EXPECT_NE(udp_p, nullptr);
		});

	EXPECT_NO_THROW({
			udp = MAVConnInterface::open_url("udp://@localhost:45005");
			udp_p = dynamic_cast<MAVConnUDP*>(udp.get());
			EXPECT_NE(udp_p, nullptr);
		});

	EXPECT_NO_THROW({
			udp = MAVConnInterface::open_url("udp://localhost:45006@");
			udp_p = dynamic_cast<MAVConnUDP*>(udp.get());
			EXPECT_NE(udp_p, nullptr);
		});

	EXPECT_THROW({
			udp = MAVConnInterface::open_url("udp://localhost:45008");
		}, DeviceError);
}

TEST(URL, open_url_tcp)
{
	MAVConnInterface::Ptr tcp_server, tcp_client;

	MAVConnTCPServer *tcp_server_p;
	MAVConnTCPClient *tcp_client_p;

	EXPECT_NO_THROW({
			tcp_server = MAVConnInterface::open_url("tcp-l://localhost:57606");
			tcp_server_p = dynamic_cast<MAVConnTCPServer*>(tcp_server.get());
			EXPECT_NE(tcp_server_p, nullptr);
		});

	EXPECT_NO_THROW({
			tcp_client = MAVConnInterface::open_url("tcp://localhost:57606");
			tcp_client_p = dynamic_cast<MAVConnTCPClient*>(tcp_client.get());
			EXPECT_NE(tcp_client_p, nullptr);
		});
}

int main(int argc, char **argv){
	//ros::init(argc, argv, "mavconn_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
