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

#include <mavros/mavconn_interface.h>
#include <ros/console.h>
#include <ros/assert.h>

using namespace mavconn;

#if MAVLINK_CRC_EXTRA
const uint8_t MAVConnInterface::mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;
#endif
std::set<int> MAVConnInterface::allocated_channels;

MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id) :
	sys_id(system_id),
	comp_id(component_id)
{
	channel = new_channel();
	ROS_ASSERT_MSG(channel >= 0, "channel allocation failure");
}

int MAVConnInterface::new_channel()
{
	int chan = 0;

	for (chan = 0; chan <= MAVLINK_COMM_NUM_BUFFERS; chan++) {
		if (allocated_channels.count(chan) == 0) {
			ROS_DEBUG_NAMED("mavconn", "Allocate new channel: %d", chan);
			allocated_channels.insert(chan);
			return chan;
		}
	}

	if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
		ROS_ERROR_NAMED("mavconn", "channel overrun");
		return -1;
	}
}

void MAVConnInterface::delete_channel(int chan)
{
	ROS_DEBUG_NAMED("mavconn", "Freeing channel: %d", chan);
	allocated_channels.erase(allocated_channels.find(chan));
};

