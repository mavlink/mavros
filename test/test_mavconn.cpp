/**
 * Test mavconn library
 */

#include <gtest/gtest.h>

#include <mavros/mavconn_interface.h>
#include "mavconn_serial.h"

using namespace mavconn;

#if 0
TEST(MAVConn, allocate_check)
{
	MAVConnInterface *conns[3];

	conns[0] = new MAVConnSerial(42, 200, "/dev/ttyUSB0", 115200);
	conns[1] = new MAVConnSerial(42, 201, "/dev/ttyS1", 115200);
	conns[3] = new MAVConnSerial(42, 202, "/dev/ttyS3", 115200);

	delete conns[1];
	conns[1] = new MAVConnSerial(42, 203, "/dev/ttyS4", 115200);

	delete conns[0];
	delete conns[1];
	delete conns[2];
}
#endif

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


int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
