/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief PX4 custom mode constants
 * @file px4_custom_mode.h
 * PX4 custom flight modes
 *
 * Modifyed copy px4_custom_mode.h from PX4/Firmware
 *
 * @author Anton Babushkin <anton@px4.io>
 */

#pragma once

//#include <stdint.h>

namespace px4 {
/**
 * @brief PX4 custom mode
 *
 * This union decodes uint32_t HEARTBEAT.custom_mode
 * and uint32_t SET_MODE.custom_mode.
 */
union custom_mode {
	enum MAIN_MODE : uint8_t {
		MAIN_MODE_MANUAL = 1,
		MAIN_MODE_ALTCTL,
		MAIN_MODE_POSCTL,
		MAIN_MODE_AUTO,
		MAIN_MODE_ACRO,
		MAIN_MODE_OFFBOARD,
		MAIN_MODE_STABILIZED,
		MAIN_MODE_RATTITUDE
	};

	enum SUB_MODE_AUTO : uint8_t {
		SUB_MODE_AUTO_READY = 1,
		SUB_MODE_AUTO_TAKEOFF,
		SUB_MODE_AUTO_LOITER,
		SUB_MODE_AUTO_MISSION,
		SUB_MODE_AUTO_RTL,
		SUB_MODE_AUTO_LAND,
		SUB_MODE_AUTO_RTGS,
		SUB_MODE_AUTO_FOLLOW_TARGET,
		SUB_MODE_AUTO_PRECLAND
	};

	struct {
		uint16_t reserved;
		uint8_t main_mode;
		uint8_t sub_mode;
	};
	uint32_t data;
	float data_float;

	custom_mode() : data(0)
	{ }

	explicit custom_mode(uint32_t val) : data(val)
	{ }

	constexpr custom_mode(uint8_t mm, uint8_t sm) :
		reserved(0),
		main_mode(mm),
		sub_mode(sm)
	{ }
};

/**
 * @brief helper function to define any mode as uint32_t constant
 *
 * @param mm main mode
 * @param sm sub mode (currently used only in auto mode)
 * @return uint32_t representation
 */
constexpr uint32_t define_mode(enum custom_mode::MAIN_MODE mm, uint8_t sm = 0) {
	return custom_mode(mm, sm).data;
}

/**
 * @brief helper function to define auto mode as uint32_t constant
 *
 * Same as @a define_mode(custom_mode::MAIN_MODE_AUTO, sm)
 *
 * @param sm auto sub mode
 * @return uint32_t representation
 */
constexpr uint32_t define_mode_auto(enum custom_mode::SUB_MODE_AUTO sm) {
	return define_mode(custom_mode::MAIN_MODE_AUTO, sm);
}
};	// namespace px4
