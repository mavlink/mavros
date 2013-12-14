/**
 * test MAVLink UDP-Serial proxy
 */

#include "mavconn_serial.h"
#include "mavconn_udp.h"

#include <stdlib.h>

using namespace mavconn;

std::auto_ptr<MAVConnSerial> serial_link;
std::auto_ptr<MAVConnUDP> udp_link;

void udp_recv_message_cb(const mavlink_message_t *message, uint8_t sys_id, uint8_t comp_id)
{
	std::cout << "[UDP] Sys-Id: " << (int)sys_id << " Comp-Id: " << (int)comp_id <<
		" Message-Id: " << (int)message->msgid << " Length: " << (int)message->len << std::endl;
}

void serial_recv_message_cb(const mavlink_message_t *message, uint8_t sys_id, uint8_t comp_id)
{
	std::cout << "[SER] Sys-Id: " << (int)sys_id << " Comp-Id: " << (int)comp_id <<
		" Message-Id: " << (int)message->msgid << " Length: " << (int)message->len << std::endl;
}

int main(int argc, char *argv[])
{
	std::string serial_port;
	unsigned serial_baud = 115200;
	std::string bind_host = "localhost";
	unsigned short bind_port = 14555;
	std::string remote_host = "";
	unsigned short remote_port = 14550;

	if (argc < 2) {
		std::cout << "Usage: mavudpproxy <serial port> [baudrate] [bind host] [bind port] [remote host] [remote port]" << std::endl;
		return 1;
	}

	if (argc >= 2)
		serial_port = argv[1];
	if (argc >= 3)
		serial_baud = atoi(argv[2]);
	if (argc >= 4)
		bind_host = argv[3];
	if (argc >= 5)
		bind_port = atoi(argv[4]);
	if (argc >= 6)
		remote_host = argv[5];
	if (argc >= 7)
		remote_port = atoi(argv[6]);

	std::cout << "Listening serial port " << serial_port << " at " << serial_baud << " bps" << std::endl;
	std::cout << "Listening UDP " << bind_host << ":" << bind_port << std::endl;
	if (remote_host != "")
		std::cout << "Remote UDP " << remote_host << ":" << remote_port << std::endl;

	serial_link.reset(new MAVConnSerial(1, 240, serial_port, serial_baud));
	udp_link.reset(new MAVConnUDP(1, 240, bind_host, bind_port, remote_host, remote_port));

	serial_link->message_received.connect(serial_recv_message_cb);
	serial_link->message_received.connect(boost::bind(&MAVConnUDP::send_message, udp_link.get(), _1, _2, _3));

	udp_link->message_received.connect(udp_recv_message_cb);
	udp_link->message_received.connect(boost::bind(&MAVConnSerial::send_message, serial_link.get(), _1, _2, _3));

	while (true);
	return 0;
}
