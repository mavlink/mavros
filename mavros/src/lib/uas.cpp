/**
 * @brief MAVROS UAS manager
 * @file uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
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

#include <stdexcept>
#include <mavros/mavros_uas.h>
#include <mavros/utils.h>
#include <mavros/px4_custom_mode.h>

using namespace mavros;

UAS::UAS() :
	type(MAV_TYPE_GENERIC),
	autopilot(MAV_AUTOPILOT_GENERIC),
	target_system(1),
	target_component(1),
	connected(false),
	angular_velocity(),
	linear_acceleration(),
	orientation(),
	gps_latitude(0),
	gps_longitude(0),
	gps_altitude(0),
	gps_eph(0),
	gps_epv(0),
	fix_status(0)
{
}

void UAS::stop(void)
{
}

/* -*- time syncronise functions -*- */

static inline ros::Time ros_time_from_ns(uint64_t &stamp_ns) {
	return ros::Time(
			stamp_ns / 1000000000UL,	// t_sec
			stamp_ns % 1000000000UL);	// t_nsec
}

ros::Time UAS::synchronise_stamp(uint32_t time_boot_ms) {
	// copy offset from atomic var
	uint64_t offset_ns = time_offset;

	if (offset_ns > 0) {
		uint64_t stamp_ns = static_cast<uint64_t>(time_boot_ms) * 1000000UL + offset_ns;
		return ros_time_from_ns(stamp_ns);
	}
	else
		return ros::Time::now();
}

ros::Time UAS::synchronise_stamp(uint64_t time_usec) {
	uint64_t offset_ns = time_offset;

	if (offset_ns > 0) {
		uint64_t stamp_ns = time_usec * 1000UL + offset_ns;
		return ros_time_from_ns(stamp_ns);
	}
	else
		return ros::Time::now();
}

/* -*- mode stringify functions -*- */

typedef std::map<uint32_t, std::string> cmode_map;

/** APM:Plane custom mode -> string
 *
 * ArduPlane/defines.h
 */
static const cmode_map arduplane_cmode_map = {
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
	{ 14, "LAND" },		// not in list
	{ 15, "GUIDED" },
	{ 16, "INITIALISING" }
};

/** APM:Copter custom mode -> string
 *
 * ArduCopter/defines.h
 */
static const cmode_map arducopter_cmode_map = {
	{ 0, "STABILIZE" },
	{ 1, "ACRO" },
	{ 2, "ALT_HOLD" },
	{ 3, "AUTO" },
	{ 4, "GUIDED" },
	{ 5, "LOITER" },
	{ 6, "RTL" },
	{ 7, "CIRCLE" },
	{ 8, "POSITION" },	// not in list
	{ 9, "LAND" },
	{ 10, "OF_LOITER" },
	{ 11, "DRIFT" },	// renamed, prev name: APPROACH
	{ 13, "SPORT" },
	{ 14, "FLIP" },
	{ 15, "AUTOTUNE" },
	{ 16, "POSHOLD" }
};

/** APM:Rover custom mode -> string
 *
 * APMrover2/defines.h
 */
static const cmode_map apmrover2_cmode_map = {
	{ 0, "MANUAL" },
	{ 2, "LEARNING" },
	{ 3, "STEERING" },
	{ 4, "HOLD" },
	{ 10, "AUTO" },
	{ 11, "RTL" },
	{ 15, "GUIDED" },
	{ 16, "INITIALISING" }
};

//! PX4 custom mode -> string
static const cmode_map px4_cmode_map = {
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

static inline std::string str_base_mode(int base_mode) {
	std::ostringstream mode;
	mode << "MODE(0x" << std::hex << std::uppercase << base_mode << ")";
	return mode.str();
}

static std::string str_custom_mode(uint32_t custom_mode) {
	std::ostringstream mode;
	mode << "CMODE(" << custom_mode << ")";
	return mode.str();
}

static std::string str_mode_cmap(const cmode_map &cmap, uint32_t custom_mode) {
	auto it = cmap.find(custom_mode);
	if (it != cmap.end())
		return it->second;
	else
		return str_custom_mode(custom_mode);
}

static inline std::string str_mode_px4(uint32_t custom_mode_int) {
	px4::custom_mode custom_mode(custom_mode_int);

	// clear fields
	custom_mode.reserved = 0;
	if (custom_mode.main_mode != px4::custom_mode::MAIN_MODE_AUTO) {
		ROS_WARN_COND_NAMED(custom_mode.sub_mode != 0, "uas", "PX4: Unknown sub-mode");
		custom_mode.sub_mode = 0;
	}

	return str_mode_cmap(px4_cmode_map, custom_mode.data);
}

static inline bool is_apm_copter(enum MAV_TYPE &type) {
	return type == MAV_TYPE_QUADROTOR ||
		type == MAV_TYPE_HEXAROTOR ||
		type == MAV_TYPE_OCTOROTOR ||
		type == MAV_TYPE_TRICOPTER ||
		type == MAV_TYPE_COAXIAL;
}

std::string UAS::str_mode_v10(uint8_t base_mode, uint32_t custom_mode) {
	if (!(base_mode && MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))
		return str_base_mode(base_mode);

	auto type = get_type();
	auto ap = get_autopilot();
	if (MAV_AUTOPILOT_ARDUPILOTMEGA == ap) {
		if (is_apm_copter(type))
			return str_mode_cmap(arducopter_cmode_map, custom_mode);
		else if (type == MAV_TYPE_FIXED_WING)
			return str_mode_cmap(arduplane_cmode_map, custom_mode);
		else if (type == MAV_TYPE_GROUND_ROVER)
			return str_mode_cmap(apmrover2_cmode_map, custom_mode);
		else {
			ROS_WARN_THROTTLE_NAMED(30, "uas", "MODE: Unknown APM based FCU! Type: %d", type);
			return str_custom_mode(custom_mode);
		}
	}
	else if (MAV_AUTOPILOT_PX4 == ap)
		return str_mode_px4(custom_mode);
	else
		/* TODO: other autopilot */
		return str_custom_mode(custom_mode);
}

static bool cmode_find_cmap(const cmode_map &cmap, std::string &cmode_str, uint32_t &cmode) {
	// 1. try find by name
	for (auto &mode : cmap) {
		if (mode.second == cmode_str) {
			cmode = mode.first;
			return true;
		}
	}

	// 2. try convert integer
	//! @todo parse CMODE(dec)
	try {
		cmode = std::stoi(cmode_str, 0, 0);
		return true;
	}
	catch (std::invalid_argument &ex) {
		// failed
	}

	// Debugging output.
	std::ostringstream os;
	for (auto &mode : cmap)
		os << " " << mode.second;

	ROS_ERROR_STREAM_NAMED("uas", "MODE: Unknown mode: " << cmode_str);
	ROS_DEBUG_STREAM_NAMED("uas", "MODE: Known modes are:" << os.str());

	return false;
}

bool UAS::cmode_from_str(std::string cmode_str, uint32_t &custom_mode) {
	// upper case
	std::transform(cmode_str.begin(), cmode_str.end(), cmode_str.begin(), std::ref(toupper));

	auto type = get_type();
	auto ap = get_autopilot();
	if (MAV_AUTOPILOT_ARDUPILOTMEGA == ap) {
		if (is_apm_copter(type))
			return cmode_find_cmap(arducopter_cmode_map, cmode_str, custom_mode);
		else if (type == MAV_TYPE_FIXED_WING)
			return cmode_find_cmap(arduplane_cmode_map, cmode_str, custom_mode);
		else if (type == MAV_TYPE_GROUND_ROVER)
			return cmode_find_cmap(apmrover2_cmode_map, cmode_str, custom_mode);
	}
	else if (MAV_AUTOPILOT_PX4 == ap)
		return cmode_find_cmap(px4_cmode_map, cmode_str, custom_mode);

	ROS_ERROR_NAMED("uas", "MODE: Unsupported FCU");
	return false;
}
