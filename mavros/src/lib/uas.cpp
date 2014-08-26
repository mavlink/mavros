/**
 * @brief MAVROS UAS manager
 * @file uas.cpp
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

#include <mavros/mavros_uas.h>
#include <mavros/utils.h>
#include <mavros/px4_custom_mode.h>

using namespace mavros;

UAS::UAS() :
	type(MAV_TYPE_GENERIC),
	autopilot(MAV_AUTOPILOT_GENERIC),
	target_system(1),
	target_component(1),
	connected(false)
{
}

void UAS::stop(void)
{
}

/* -*- mode stringify functions -*- */

static std::string str_base_mode(int base_mode) {
	std::ostringstream mode;
	mode << "MODE(0x" << std::hex << std::uppercase << base_mode << ")";
	return mode.str();
}

static std::string str_custom_mode(int custom_mode) {
	std::ostringstream mode;
	mode << "CMODE(" << custom_mode << ")";
	return mode.str();
}

//! APM:Plane custom mode -> string
static const std::map<uint32_t, std::string> arduplane_cmode_map = {
	{ 0, "MANUAL" },
	{ 1, "CIRCLE" },
	{ 2, "STABILIZE" },
	{ 3, "TRAINING" },
	{ 4, "ACRO" },
	{ 5, "FBWA" },
	{ 6, "FBWB" },
	{ 7, "CRUISE" },
	{ 8, "AUTOTUNE" },
	{ 10, "AUTO" },
	{ 11, "RTL" },
	{ 12, "LOITER" },
	{ 14, "LAND" },
	{ 15, "GUIDED" },
	{ 16, "INITIALISING" }
};

//! APM:Copter custom mode -> string
static const std::map<uint32_t, std::string> arducopter_cmode_map = {
	{ 0, "STABILIZE" },
	{ 1, "ACRO" },
	{ 2, "ALT_HOLD" },
	{ 3, "AUTO" },
	{ 4, "GUIDED" },
	{ 5, "LOITER" },
	{ 6, "RTL" },
	{ 7, "CIRCLE" },
	{ 8, "POSITION" },
	{ 9, "LAND" },
	{ 10, "OF_LOITER" },
	{ 11, "APPROACH" }
};

//! PX4 custom mode -> string
static const std::map<uint32_t, std::string> px4_cmode_map = {
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_MANUAL),           "MANUAL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_ACRO),             "ACRO" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_ALTCTL),           "ALTCTL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_POSCTL),           "POSCTL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_OFFBOARD),         "OFFBOARD" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_MISSION), "AUTO.MISSION" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_LOITER),  "AUTO.LOITER" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_RTL),     "AUTO.RTL" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_LAND),    "AUTO.LAND" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_RTGS),    "AUTO.RTGS" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_READY),   "AUTO.READY" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_TAKEOFF), "AUTO.TAKEOFF" }
};

static inline std::string str_mode_arduplane(int custom_mode) {
	auto it = arduplane_cmode_map.find(custom_mode);
	if (it != arduplane_cmode_map.end())
		return it->second;
	else
		return str_custom_mode(custom_mode);
}

static inline std::string str_mode_arducopter(int custom_mode) {
	auto it = arducopter_cmode_map.find(custom_mode);
	if (it != arducopter_cmode_map.end())
		return it->second;
	else
		return str_custom_mode(custom_mode);
}

static inline std::string str_mode_px4(int custom_mode_int) {
	px4::custom_mode custom_mode(custom_mode_int);

	// clear fields
	custom_mode.reserved = 0;
	if (custom_mode.main_mode != px4::custom_mode::MAIN_MODE_AUTO) {
		ROS_WARN_COND(custom_mode.sub_mode != 0, "PX4: Unknown sub-mode");
		custom_mode.sub_mode = 0;
	}

	auto it = px4_cmode_map.find(custom_mode.data);
	if (it != px4_cmode_map.end())
		return it->second;
	else
		return str_custom_mode(custom_mode_int);
}

std::string UAS::str_mode_v10(int base_mode, int custom_mode) {
	if (!(base_mode && MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))
		return str_base_mode(base_mode);

	auto type = get_type();
	auto ap = get_autopilot();
	if (MAV_AUTOPILOT_ARDUPILOTMEGA == ap) {
		if (type == MAV_TYPE_QUADROTOR ||
				type == MAV_TYPE_HEXAROTOR ||
				type == MAV_TYPE_OCTOROTOR ||
				type == MAV_TYPE_TRICOPTER ||
				type == MAV_TYPE_COAXIAL)
			return str_mode_arducopter(custom_mode);
		else if (type == MAV_TYPE_FIXED_WING)
			return str_mode_arduplane(custom_mode);
		else
			/* TODO: APM:Rover */
			return str_custom_mode(custom_mode);
	}
	else if (MAV_AUTOPILOT_PX4 == ap)
		return str_mode_px4(custom_mode);
	else
		/* TODO: other autopilot */
		return str_custom_mode(custom_mode);
}

