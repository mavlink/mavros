/**
 * @file mavconn_interface.cpp
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

#include <set>
#include <ev++.h>
#include <mavros/mavconn_interface.h>
#include <mavros/utils.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/ros.h>

namespace mavconn {

#if MAVLINK_CRC_EXTRA
const uint8_t MAVConnInterface::mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;
#endif
std::set<int> MAVConnInterface::allocated_channels;

static ev::default_loop default_loop;
static boost::thread default_loop_thd;

static void loop_spinner() {
	while (ros::ok()) {
		ROS_DEBUG_NAMED("mavconn", "EV: starting default loop");
		default_loop.run(0);
		ROS_DEBUG_NAMED("mavconn", "EV: default loop stopped");
	}
}

void MAVConnInterface::start_default_loop() {
	if (default_loop_thd.joinable())
		return;

	boost::thread t(loop_spinner);
	mavutils::set_thread_name(t, "ev_default_loop");
	default_loop_thd.swap(t);
}

MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id) :
	sys_id(system_id),
	comp_id(component_id)
{
	channel = new_channel();
	ROS_ASSERT_MSG(channel >= 0, "channel allocation failure");
}

int MAVConnInterface::new_channel() {
	int chan = 0;

	for (chan = 0; chan < MAVLINK_COMM_NUM_BUFFERS; chan++) {
		if (allocated_channels.count(chan) == 0) {
			ROS_DEBUG_NAMED("mavconn", "Allocate new channel: %d", chan);
			allocated_channels.insert(chan);
			return chan;
		}
	}

	ROS_ERROR_NAMED("mavconn", "channel overrun");
	return -1;
}

void MAVConnInterface::delete_channel(int chan) {
	ROS_DEBUG_NAMED("mavconn", "Freeing channel: %d", chan);
	allocated_channels.erase(allocated_channels.find(chan));
}

boost::shared_ptr<MAVConnInterface> MAVConnInterface::open_url(std::string url,
		uint8_t system_id, uint8_t component_id) {
	// TODO
	return boost::shared_ptr<MAVConnInterface>();
}

}; // namespace mavconn
