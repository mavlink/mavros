/**
 * @@brief MAVLink helpers
 * @@file mavlink_helpers.cpp
 * @@author Vladimir Ermakov <vooon341@@gmail.com>
 *
 * This file defines replace for some helper function to prevent problem #269.
 *
 * @@addtogroup mavconn
 * @@{
 */
/*
 * libmavconn
 * Copyright 2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
@#
@# EmPy template of dialect helpers source file
@#

#include <mavconn/console_bridge_compat.h>
#include <mavconn/interface.h>

// AUTOMATIC GENERATED FILE!
// from src/mavlink_helpers.cpp.em

using mavconn::MAVConnInterface;

void MAVConnInterface::init_msg_entry()
{
	CONSOLE_BRIDGE_logDebug("mavconn: Initialize message_entries map");

	auto load = [&](const char *dialect, const mavlink::mavlink_msg_entry_t & e) {
		auto it = message_entries.find(e.msgid);
		if (it != message_entries.end()) {
			if (memcmp(&e, it->second, sizeof(e)) != 0) {
				CONSOLE_BRIDGE_logDebug("mavconn: init: message from %s, MSG-ID %d ignored! Table has different entry.", dialect, e.msgid);
			}
			else {
				CONSOLE_BRIDGE_logDebug("mavconn: init: message from %s, MSG-ID %d in table.", dialect, e.msgid);
			}
		}
		else {
			CONSOLE_BRIDGE_logDebug("mavconn: init: add message entry for %s, MSG-ID %d", dialect, e.msgid);
			message_entries[e.msgid] = &e;
		}
	};

	@[for dialect in MAVLINK_V20_DIALECTS]for (auto &e : mavlink::@dialect::MESSAGE_ENTRIES) @(' ' * (20 - len(dialect))) load("@dialect", e);
	@[end for]
}

std::vector<std::string> MAVConnInterface::get_known_dialects()
{
	return {
		@[for dialect in MAVLINK_V20_DIALECTS]"@dialect",
		@[end for]
	};
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
