/**
 * @file mavconn_serial.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013 Vladimir Ermakov.
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

#include <mavros/mavconn_serial.h>
#include <ros/console.h>
#include <ros/assert.h>

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>

namespace mavconn {

static void set_speed(struct termios &ts, unsigned baudrate) {
	speed_t mode = B57600;

	switch (baudrate) {
	case 0:		mode = B0;	break;
	// aicient
	case 600:	mode = B600;	break;
	case 1200:	mode = B1200;	break;
	case 1800:	mode = B1800;	break;
	case 2400:	mode = B2400;	break;
	case 4800:	mode = B4800;	break;
	// useful
	case 9600:	mode = B9600;	break;
	case 19200:	mode = B19200;	break;
	case 38400:	mode = B38400;	break;
	case 57600:	mode = B57600;	break;
	case 115200:	mode = B115200;	break;
	// USB
	case 230400:	mode = B230400;	break;
	case 460800:	mode = B460800;	break;
	case 500000:	mode = B500000;	break;
	case 576000:	mode = B576000;	break;
	case 921600:	mode = B921600;	break;
	default:
		if (baudrate > 921600) {
			mode = B921600;
			ROS_WARN_NAMED("mavconn", "Unknown baudrate %u, "
					"seems it is USB device, trying 921600", baudrate);
		}
		else
			ROS_ERROR_NAMED("mavconn", "Unknown baudrate %u, "
					"trying default 57600", baudrate);
		break;
	}

	::cfsetispeed(&ts, mode);
	::cfsetospeed(&ts, mode);
}

static void configure_8N1(struct termios &ts)
{
	// configure 8N1, not flow control
	ts.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
	ts.c_cflag |= CLOCAL | CREAD | CS8;

	ts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	ts.c_oflag &= ~OPOST;

	// setup blocking timeout (not used)
	ts.c_cc[VMIN] = 1;
	ts.c_cc[VTIME] = 0;
}

MAVConnSerial::MAVConnSerial(uint8_t system_id, uint8_t component_id,
		std::string device, unsigned baudrate) :
	MAVConnInterface(system_id, component_id),
	fd(-1)
{
	ROS_INFO_STREAM_NAMED("mavconn", "serial: device: " << device << " @ " << baudrate << " bps");

	fd = ::open(device.c_str(), O_RDWR|O_NOCTTY);
	if (fd < 0)
		throw DeviceError("serial: open", errno);

	struct termios opts;

	if (tcgetattr(fd, &opts) < 0)
		throw DeviceError("serial: tcgetattr", errno);

	configure_8N1(opts);
	set_speed(opts, baudrate);

	if (tcsetattr(fd, TCSANOW, &opts) < 0)
		throw DeviceError("serial: tcsetattr", errno);

	if (fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK) == -1)
		throw DeviceError("serial: fcntl", errno);

	// run io for async io
	io.set<MAVConnSerial, &MAVConnSerial::event_cb>(this);
	io.start(fd, ev::READ);
	start_default_loop();
}

MAVConnSerial::~MAVConnSerial() {
	close();
}

void MAVConnSerial::close() {
	if (fd < 0)
		return;

	io.stop();
	::close(fd); fd = -1;

	/* emit */ port_closed();
}

void MAVConnSerial::send_bytes(const uint8_t *bytes, size_t length)
{
	MsgBuffer *buf = new MsgBuffer(bytes, length);
	{
		boost::recursive_mutex::scoped_lock lock(mutex);
		tx_q.push_back(buf);
		io.set(ev::READ | ev::WRITE);
	}
}

void MAVConnSerial::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
	ROS_ASSERT(message != nullptr);
	MsgBuffer *buf = nullptr;

	/* if sysid/compid pair not match we need explicit finalize
	 * else just copy to buffer */
	if (message->sysid != sysid || message->compid != compid) {
		mavlink_message_t msg = *message;

#if MAVLINK_CRC_EXTRA
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len, mavlink_crcs[msg.msgid]);
#else
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len);
#endif

		buf = new MsgBuffer(&msg);
	}
	else
		buf = new MsgBuffer(message);

	ROS_DEBUG_NAMED("mavconn", "serial::send_message: Message-ID: %d [%zu bytes]", message->msgid, buf->nbytes());

	{
		boost::recursive_mutex::scoped_lock lock(mutex);
		tx_q.push_back(buf);
		io.set(ev::READ | ev::WRITE);
	}
}

void MAVConnSerial::event_cb(ev::io &watcher, int revents)
{
	if (ev::ERROR & revents) {
		ROS_ERROR_NAMED("mavconn", "event_cb::revents: 0x%08x", revents);
		close();
		return;
	}

	if (ev::READ & revents)
		read_cb(watcher);

	if (ev::WRITE & revents)
		write_cb(watcher);
}

void MAVConnSerial::read_cb(ev::io &watcher)
{
	mavlink_message_t message;
	mavlink_status_t status;
	uint8_t rx_buf[MsgBuffer::MAX_SIZE];

	ssize_t nread = ::read(watcher.fd, rx_buf, sizeof(rx_buf));
	if (nread < 1) {
		ROS_ERROR_NAMED("mavconn", "serial::read_cb: %s", strerror(errno));
		close();
		return;
	}

	for (ssize_t i = 0; i < nread; i++) {
		if (mavlink_parse_char(channel, rx_buf[i], &message, &status)) {
			ROS_DEBUG_NAMED("mavconn", "serial::read_cb: recv Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d",
					message.msgid, message.len, message.sysid, message.compid);

			/* emit */ message_received(&message, message.sysid, message.compid);
		}
	}
}

void MAVConnSerial::write_cb(ev::io &watcher)
{
	boost::recursive_mutex::scoped_lock lock(mutex);

	if (tx_q.empty()) {
		io.set(ev::READ);
		return;
	}

	MsgBuffer *buf = tx_q.front();
	ssize_t written = ::write(watcher.fd, buf->dpos(), buf->nbytes());
	if (written < 0) {
		ROS_ERROR_NAMED("mavconn", "serial::write_cb: %s", strerror(errno));
		close();
		return;
	}

	buf->pos += written;
	if (buf->nbytes() == 0) {
		tx_q.pop_front();
		delete buf;
	}

	if (tx_q.empty())
		io.set(ev::READ);
	else
		io.set(ev::READ | ev::WRITE);
}

}; // namespace mavconn
