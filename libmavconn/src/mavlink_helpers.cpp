/**
 * @brief MAVLink helpers
 * @file mavlink_helpers.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * This file defines replace for some helper function to prevent problem #269.
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <console_bridge/console.h>
#include <mavconn/interface.h>

using mavconn::MAVConnInterface;

// Use:
// mavlink::common::MESSAGE_ENTRIES
//
// And use empy to generate list

void MAVConnInterface::init_msg_entry(void)
{
	mavconn::lock_guard lock(init_mutex);

	// it is initialized?
	if (message_entries.size())
		return;

	logInform("mavconn: Initialize message_entries map");

	auto load = [&](const char *dialect, const mavlink::mavlink_msg_entry_t &e) {
		auto it = message_entries.find(e.msgid);
		if (it != message_entries.end()) {
			if (memcmp(&e, it->second, sizeof(e)) != 0) {
				logWarn("mavconn: init: message from %s, MSG-ID %d ignored! Table has different entry.", dialect, e.msgid);
			}
			else {
				logDebug("mavconn: init: message from %s, MSG-ID %d in table.", dialect, e.msgid);
			}
		}
		else {
			logDebug("mavconn: init: add message entry for %s, MSG-ID %d", dialect, e.msgid);
			message_entries[e.msgid] = &e;
		}
	};

	// XXX work for empy
	for (auto &e : mavlink::common::MESSAGE_ENTRIES) {
		load("common", e);
	}

	for (auto &e : mavlink::ardupilotmega::MESSAGE_ENTRIES) {
		load("ardupilotmega", e);
	}
}

/**
 * Internal function to give access to message information such as additional crc byte.
 */
const mavlink::mavlink_msg_entry_t* mavlink::mavlink_get_msg_entry(uint32_t msgid)
{
	auto it = MAVConnInterface::message_entries.find(msgid);
	if (it != MAVConnInterface::message_entries.end())
		return it->second;
	else
		return nullptr;
}
