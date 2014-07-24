/**
 * @brief Mixin for setpoint plugins
 * @file setpoint_mixin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
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

#pragma once

#include <mavros/mavros_plugin.h>

namespace mavplugin {

/**
 * @brief This mixin adds local_ned_position_setpoint_extarnal()
 *
 * @note derived class should provide UAS pointer in uas member.
 */
template <class Derived>
class LocalNEDPositionSetpointExternalMixin {
public:
	void local_ned_position_setpoint_external(uint32_t time_boot_ms, uint8_t coordinate_frame,
			uint16_t type_mask,
			float x, float y, float z,
			float vx, float vy, float vz,
			float afx, float afy, float afz) {
		UAS *_uas = static_cast<Derived *>(this)->uas;
		mavlink_message_t msg;
		mavlink_msg_local_ned_position_setpoint_external_pack_chan(UAS_PACK_CHAN(_uas), &msg,
				time_boot_ms, // why it not usec timestamp?
				UAS_PACK_TGT(_uas),
				coordinate_frame,
				type_mask,
				x, y, z,
				vz, vy, vz,
				afx, afy, afz);
		_uas->mav_link->send_message(&msg);
	}
};

}; // namespace mavplugin
