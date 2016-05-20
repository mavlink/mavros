/**
 * @brief MAVConn class interface
 * @file interface.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <set>
#include <cassert>
#include <console_bridge/console.h>

#include <mavconn/interface.h>
#include <mavconn/msgbuffer.h>
#include <mavconn/serial.h>
#include <mavconn/udp.h>
#include <mavconn/tcp.h>

namespace mavconn {

#define PFX	"mavconn: "

#if MAVLINK_CRC_EXTRA
const uint8_t MAVConnInterface::mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;
#endif
std::set<int> MAVConnInterface::allocated_channels;
std::recursive_mutex MAVConnInterface::channel_mutex;


MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id) :
	sys_id(system_id),
	comp_id(component_id),
	tx_total_bytes(0),
	rx_total_bytes(0),
	last_tx_total_bytes(0),
	last_rx_total_bytes(0),
	last_iostat(steady_clock::now())
{
	channel = new_channel();
	assert(channel >= 0);
}

int MAVConnInterface::new_channel() {
	std::lock_guard<std::recursive_mutex> lock(channel_mutex);
	int chan = 0;

	for (chan = 0; chan < MAVLINK_COMM_NUM_BUFFERS; chan++) {
		if (allocated_channels.count(chan) == 0) {
			logDebug(PFX "Allocate new channel: %d", chan);
			allocated_channels.insert(chan);
			return chan;
		}
	}

	logError(PFX "channel overrun");
	return -1;
}

void MAVConnInterface::delete_channel(int chan) {
	std::lock_guard<std::recursive_mutex> lock(channel_mutex);
	logDebug(PFX "Freeing channel: %d", chan);
	allocated_channels.erase(allocated_channels.find(chan));
}

int MAVConnInterface::channes_available() {
	std::lock_guard<std::recursive_mutex> lock(channel_mutex);
	return MAVLINK_COMM_NUM_BUFFERS - allocated_channels.size();
}

MsgBuffer *MAVConnInterface::new_msgbuffer(const mavlink_message_t *message,
		uint8_t sysid, uint8_t compid)
{
	/* if sysid/compid pair not match we need explicit finalize
	 * else just copy to buffer */
	if (message->sysid != sysid || message->compid != compid) {
		mavlink_message_t msg = *message;

#ifdef MAVLINK2_COMPAT
		// for mavlink 1.0 len == min_len
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len, message->len,
				mavlink_crcs[msg.msgid]);
#else
# if MAVLINK_CRC_EXTRA
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len,
				mavlink_crcs[msg.msgid]);
# else
		mavlink_finalize_message_chan(&msg, sysid, compid, channel, message->len);
# endif
#endif

		return new MsgBuffer(&msg);
	}
	else
		return new MsgBuffer(message);
}

mavlink_status_t MAVConnInterface::get_status()
{
	return *mavlink_get_channel_status(channel);
}

MAVConnInterface::IOStat MAVConnInterface::get_iostat()
{
	std::lock_guard<std::recursive_mutex> lock(iostat_mutex);
	IOStat stat;

	stat.tx_total_bytes = tx_total_bytes;
	stat.rx_total_bytes = rx_total_bytes;

	auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
	auto d_rx = stat.rx_total_bytes - last_rx_total_bytes;
	last_tx_total_bytes = stat.tx_total_bytes;
	last_rx_total_bytes = stat.rx_total_bytes;

	auto now = steady_clock::now();
	auto dt = now - last_iostat;
	last_iostat = now;

	float dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

	stat.tx_speed = d_tx / dt_s;
	stat.rx_speed = d_rx / dt_s;

	return stat;
}

void MAVConnInterface::iostat_tx_add(size_t bytes)
{
	tx_total_bytes += bytes;
}

void MAVConnInterface::iostat_rx_add(size_t bytes)
{
	rx_total_bytes += bytes;
}

/**
 * Parse host:port pairs
 */
static void url_parse_host(std::string host,
		std::string &host_out, int &port_out,
		const std::string def_host, const int def_port)
{
	std::string port;

	auto sep_it = std::find(host.begin(), host.end(), ':');
	if (sep_it == host.end()) {
		// host
		if (!host.empty()) {
			host_out = host;
			port_out = def_port;
		}
		else {
			host_out = def_host;
			port_out = def_port;
		}
		return;
	}

	if (sep_it == host.begin()) {
		// :port
		host_out = def_host;
	}
	else {
		// host:port
		host_out.assign(host.begin(), sep_it);
	}

	port.assign(sep_it + 1, host.end());
	port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
static void url_parse_query(std::string query, uint8_t &sysid, uint8_t &compid)
{
	const std::string ids_end("ids=");
	std::string sys, comp;

	if (query.empty())
		return;

	auto ids_it = std::search(query.begin(), query.end(),
			ids_end.begin(), ids_end.end());
	if (ids_it == query.end()) {
		logWarn(PFX "URL: unknown query arguments");
		return;
	}

	std::advance(ids_it, ids_end.length());
	auto comma_it = std::find(ids_it, query.end(), ',');
	if (comma_it == query.end()) {
		logError(PFX "URL: no comma in ids= query");
		return;
	}

	sys.assign(ids_it, comma_it);
	comp.assign(comma_it + 1, query.end());

	sysid = std::stoi(sys);
	compid = std::stoi(comp);

	logDebug(PFX "URL: found system/component id = [%u, %u]", sysid, compid);
}

static MAVConnInterface::Ptr url_parse_serial(
		std::string path, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string file_path;
	int baudrate;

	// /dev/ttyACM0:57600
	url_parse_host(path, file_path, baudrate, "/dev/ttyACM0", 57600);
	url_parse_query(query, system_id, component_id);

	return boost::make_shared<MAVConnSerial>(system_id, component_id,
			file_path, baudrate);
}

static MAVConnInterface::Ptr url_parse_udp(
		std::string hosts, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string bind_pair, remote_pair;
	std::string bind_host, remote_host;
	int bind_port, remote_port;

	auto sep_it = std::find(hosts.begin(), hosts.end(), '@');
	if (sep_it == hosts.end()) {
		logError(PFX "UDP URL should contain @!");
		throw DeviceError("url", "UDP separator not found");
	}

	bind_pair.assign(hosts.begin(), sep_it);
	remote_pair.assign(sep_it + 1, hosts.end());

	// udp://0.0.0.0:14555@:14550
	url_parse_host(bind_pair, bind_host, bind_port, "0.0.0.0", 14555);
	url_parse_host(remote_pair, remote_host, remote_port, "", 14550);
	url_parse_query(query, system_id, component_id);

	return boost::make_shared<MAVConnUDP>(system_id, component_id,
			bind_host, bind_port,
			remote_host, remote_port);
}

static MAVConnInterface::Ptr url_parse_tcp_client(
		std::string host, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string server_host;
	int server_port;

	// tcp://localhost:5760
	url_parse_host(host, server_host, server_port, "localhost", 5760);
	url_parse_query(query, system_id, component_id);

	return boost::make_shared<MAVConnTCPClient>(system_id, component_id,
			server_host, server_port);
}

static MAVConnInterface::Ptr url_parse_tcp_server(
		std::string host, std::string query,
		uint8_t system_id, uint8_t component_id)
{
	std::string bind_host;
	int bind_port;

	// tcp-l://0.0.0.0:5760
	url_parse_host(host, bind_host, bind_port, "0.0.0.0", 5760);
	url_parse_query(query, system_id, component_id);

	return boost::make_shared<MAVConnTCPServer>(system_id, component_id,
			bind_host, bind_port);
}

MAVConnInterface::Ptr MAVConnInterface::open_url(std::string url,
		uint8_t system_id, uint8_t component_id) {

	/* Based on code found here:
	 * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
	 */

	const std::string proto_end("://");
	std::string proto;
	std::string host;
	std::string path;
	std::string query;

	auto proto_it = std::search(
			url.begin(), url.end(),
			proto_end.begin(), proto_end.end());
	if (proto_it == url.end()) {
		// looks like file path
		logDebug(PFX "URL: %s: looks like file path", url.c_str());
		return url_parse_serial(url, "", system_id, component_id);
	}

	// copy protocol
	proto.reserve(std::distance(url.begin(), proto_it));
	std::transform(url.begin(), proto_it,
			std::back_inserter(proto),
			std::ref(tolower));

	// copy host
	std::advance(proto_it, proto_end.length());
	auto path_it = std::find(proto_it, url.end(), '/');
	std::transform(proto_it, path_it,
			std::back_inserter(host),
			std::ref(tolower));

	// copy path, and query if exists
	auto query_it = std::find(path_it, url.end(), '?');
	path.assign(path_it, query_it);
	if (query_it != url.end())
		++query_it;
	query.assign(query_it, url.end());

	logDebug(PFX "URL: %s: proto: %s, host: %s, path: %s, query: %s",
			url.c_str(), proto.c_str(), host.c_str(),
			path.c_str(), query.c_str());

	if (proto == "udp")
		return url_parse_udp(host, query, system_id, component_id);
	else if (proto == "tcp")
		return url_parse_tcp_client(host, query, system_id, component_id);
	else if (proto == "tcp-l")
		return url_parse_tcp_server(host, query, system_id, component_id);
	else if (proto == "serial")
		return url_parse_serial(path, query, system_id, component_id);
	else
		throw DeviceError("url", "Unknown URL type");
}

}; // namespace mavconn
