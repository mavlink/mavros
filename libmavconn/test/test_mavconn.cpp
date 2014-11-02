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

#include <ros/ros.h>

#include <chrono>
#include <condition_variable>

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/udp.h>
#include <mavconn/tcp.h>

using namespace mavconn;

TEST(MAVConn, allocate_check)
{
	std::unique_ptr<MAVConnInterface> conns[3];

	conns[0].reset(new MAVConnUDP(42, 200, "localhost", 45000));
	conns[1].reset(new MAVConnUDP(42, 201, "localhost", 45001));
	conns[2].reset(new MAVConnUDP(42, 202, "localhost", 45002));

	conns[1].reset(); // delete before allocation to ensure index
	conns[1].reset(new MAVConnUDP(42, 203, "localhost", 45003));
	ASSERT_EQ(conns[1]->get_channel(), 1);
}

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

class UDP : public ::testing::Test {
public:
	std::mutex mutex;
	std::condition_variable cond;

	int8_t message_id;

	void recv_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid) {
		//ROS_DEBUG("Got message %d, len: %d", message->msgid, message->len);
		message_id = message->msgid;
		cond.notify_one();
	}

	bool wait_one() {
		std::unique_lock<std::mutex> lock(mutex);
		return cond.wait_for(lock, std::chrono::seconds(2)) == std::cv_status::no_timeout;
	}
};

TEST_F(UDP, bind_error)
{
	std::unique_ptr<MAVConnInterface> conns[2];

	conns[0].reset(new MAVConnUDP(42, 200, "localhost", 45000));
	ASSERT_THROW(conns[1].reset(new MAVConnUDP(42, 200, "localhost", 45000)), DeviceError);
}

TEST_F(UDP, send_message)
{
	// Issue #72
	std::unique_ptr<MAVConnInterface> echo;
	std::unique_ptr<MAVConnInterface> client;

	message_id = 255;

	// create echo server
	echo.reset(new MAVConnUDP(42, 200, "0.0.0.0", 45000));
	echo->message_received.connect(boost::bind(&MAVConnInterface::send_message, echo.get(), _1, _2, _3));

	// create client
	client.reset(new MAVConnUDP(44, 200, "0.0.0.0", 45001, "localhost", 45000));
	client->message_received.connect(boost::bind(&UDP::recv_message, this, _1, _2, _3));

	// wait echo
	send_heartbeat(client.get());
	send_heartbeat(client.get());
	EXPECT_EQ(wait_one(), true);
	EXPECT_EQ(message_id, MAVLINK_MSG_ID_HEARTBEAT);
}

class TCP : public UDP {
};

TEST_F(TCP, bind_error)
{
	std::unique_ptr<MAVConnInterface> conns[2];

	conns[0].reset(new MAVConnTCPServer(42, 200, "localhost", 57600));
	ASSERT_THROW(conns[1].reset(new MAVConnTCPServer(42, 200, "localhost", 57600)), DeviceError);
}

TEST_F(TCP, connect_error)
{
	std::unique_ptr<MAVConnInterface> client;
	ASSERT_THROW(client.reset(new MAVConnTCPClient(42, 200, "localhost", 57666)), DeviceError);
}

TEST_F(TCP, send_message)
{
	// Issue #72
	std::unique_ptr<MAVConnInterface> echo_server;
	std::unique_ptr<MAVConnInterface> client;

	message_id = 255;

	// create echo server
	echo_server.reset(new MAVConnTCPServer(42, 200, "0.0.0.0", 57600));
	echo_server->message_received.connect(boost::bind(&MAVConnInterface::send_message, echo_server.get(), _1, _2, _3));

	// create client
	client.reset(new MAVConnTCPClient(44, 200, "localhost", 57600));
	client->message_received.connect(boost::bind(&TCP::recv_message, this, _1, _2, _3));

	// wait echo
	send_heartbeat(client.get());
	send_heartbeat(client.get());
	EXPECT_EQ(wait_one(), true);
	ASSERT_EQ(message_id, MAVLINK_MSG_ID_HEARTBEAT);
}

TEST_F(TCP, client_reconnect)
{
	std::unique_ptr<MAVConnInterface> echo_server;
	std::unique_ptr<MAVConnInterface> client1, client2;

	// create echo server
	echo_server.reset(new MAVConnTCPServer(42, 200, "0.0.0.0", 57600));
	echo_server->message_received.connect(boost::bind(&MAVConnInterface::send_message, echo_server.get(), _1, _2, _3));

	EXPECT_NO_THROW({
		client1.reset(new MAVConnTCPClient(44, 200, "localhost", 57600));
	});

	EXPECT_NO_THROW({
		client2.reset(new MAVConnTCPClient(45, 200, "localhost", 57600));
	});

	EXPECT_NO_THROW({
		client1.reset(new MAVConnTCPClient(46, 200, "localhost", 57600));
	});
}

TEST(SERIAL, open_error)
{
	std::unique_ptr<MAVConnInterface> serial;
	ASSERT_THROW(serial.reset(new MAVConnSerial(42, 200, "/some/magic/not/exist/path", 57600)), DeviceError);
}

#if 0
TEST(URL, open_url_serial)
{
	boost::shared_ptr<MAVConnInterface> serial;
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
	boost::shared_ptr<MAVConnInterface> udp;
	MAVConnUDP *udp_p;

	EXPECT_NO_THROW({
		udp = MAVConnInterface::open_url("udp://localhost:45000@localhost:45005/?ids=2,241");
		udp_p = dynamic_cast<MAVConnUDP*>(udp.get());
		EXPECT_NE(udp_p, nullptr);
	});

	EXPECT_NO_THROW({
		udp = MAVConnInterface::open_url("udp://@localhost:45005");
		udp_p = dynamic_cast<MAVConnUDP*>(udp.get());
		EXPECT_NE(udp_p, nullptr);
	});

	EXPECT_NO_THROW({
		udp = MAVConnInterface::open_url("udp://localhost:45000@");
		udp_p = dynamic_cast<MAVConnUDP*>(udp.get());
		EXPECT_NE(udp_p, nullptr);
	});

	EXPECT_THROW({
		udp = MAVConnInterface::open_url("udp://localhost:45000");
	}, DeviceError);
}

TEST(URL, open_url_tcp)
{
	boost::shared_ptr<MAVConnInterface>
		tcp_server,
		tcp_client;

	MAVConnTCPServer *tcp_server_p;
	MAVConnTCPClient *tcp_client_p;

	EXPECT_NO_THROW({
		tcp_server = MAVConnInterface::open_url("tcp-l://localhost:57600");
		tcp_server_p = dynamic_cast<MAVConnTCPServer*>(tcp_server.get());
		EXPECT_NE(tcp_server_p, nullptr);
	});

	EXPECT_NO_THROW({
		tcp_client = MAVConnInterface::open_url("tcp://localhost:57600");
		tcp_client_p = dynamic_cast<MAVConnTCPClient*>(tcp_client.get());
		EXPECT_NE(tcp_client_p, nullptr);
	});
}

int main(int argc, char **argv){
	//ros::init(argc, argv, "mavconn_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
