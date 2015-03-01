/**
 * @brief PX4 custom mode constants
 * @file px4_custom_mode.h
 *
 * @addtogroup nodelib
 * @{
 */
/* Modifyed copy px4_custom_mode.h from PX4/Firmware
 * No license given in header, but i think is BSD 3-clause.
 *
 *  Created on: 09.08.2013
 *      Author: ton
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
	};

	enum SUB_MODE_AUTO : uint8_t {
		SUB_MODE_AUTO_READY = 1,
		SUB_MODE_AUTO_TAKEOFF,
		SUB_MODE_AUTO_LOITER,
		SUB_MODE_AUTO_MISSION,
		SUB_MODE_AUTO_RTL,
		SUB_MODE_AUTO_LAND,
		SUB_MODE_AUTO_RTGS
	};

	struct {
		uint16_t reserved;
		uint8_t main_mode;
		uint8_t sub_mode;
	};
	uint32_t data;
	float data_float;

	custom_mode() : data(0)
	{ };

	explicit custom_mode(uint32_t val) : data(val)
	{ };

	constexpr custom_mode(uint8_t mm, uint8_t sm) :
		reserved(0),
		main_mode(mm),
		sub_mode(sm)
	{ };
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
