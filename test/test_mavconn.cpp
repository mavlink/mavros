/**
 * Test mavconn library
 */

#include <gtest/gtest.h>

#include <mavros/mavconn_interface.h>
#include <mavros/mavconn_serial.h>
#include <mavros/mavconn_udp.h>

using namespace mavconn;

TEST(MAVConn, allocate_check)
{
	std::auto_ptr<MAVConnInterface> conns[3];

	conns[0].reset(new MAVConnUDP(42, 200, "localhost", 45000));
	conns[1].reset(new MAVConnUDP(42, 201, "localhost", 45001));
	conns[2].reset(new MAVConnUDP(42, 202, "localhost", 45002));

	conns[1].reset(); // delete before allocation to ensure index
	conns[1].reset(new MAVConnUDP(42, 203, "localhost", 45003));
	EXPECT_EQ(conns[1]->get_channel(), 1);
}

#if 0
/* This test can not be started on ROS buildfarm, need new test */

boost::recursive_timed_mutex mutex;
int sig_recv_cnt = 0;

void sig_recv(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	boost::recursive_timed_mutex::scoped_lock lock(mutex);

	sig_recv_cnt++;
	std::cout << "[RECV] Sys-Id: " << (int)sysid << " Comp-Id: " << (int)compid <<
		" Message-Id: " << (int)message->msgid << " Length: " << (int)message->len << std::endl;
}

TEST(MAVConn, test_connection)
{
	MAVConnInterface *conn;

	conn = new MAVConnSerial(42, 200, "/dev/ttyACM0", 115200);

	conn->message_received.connect(sig_recv);

	//while (true);
	sleep(10);

	delete conn;
	EXPECT_GT(sig_recv_cnt, 0);
}
#endif

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
