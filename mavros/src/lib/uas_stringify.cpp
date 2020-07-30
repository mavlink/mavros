/**
 * @brief MAVROS UAS manager
 * @file uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <array>
#include <unordered_map>
#include <stdexcept>
#include <mavros/mavros_uas.h>
#include <mavros/px4_custom_mode.h>

using namespace mavros;
using mavros::utils::enum_value;


/* -*- mode stringify functions -*- */

typedef std::unordered_map<uint32_t, const std::string> cmode_map;

/** APM:Plane custom mode -> string
 *
 * ArduPlane/defines.h
 */
static const cmode_map arduplane_cmode_map{{
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
	{ 16, "INITIALISING" },
	{ 17, "QSTABILIZE" },	// QuadPlane
	{ 18, "QHOVER" },
	{ 19, "QLOITER" },
	{ 20, "QLAND" },
	{ 21, "QRTL" }
}};

/** APM:Copter custom mode -> string
 *
 * ArduCopter/defines.h
 */
static const cmode_map arducopter_cmode_map{{
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
	{ 16, "POSHOLD" },
	{ 17, "BRAKE" },
	{ 18, "THROW" },
	{ 19, "AVOID_ADSB" },
	{ 20, "GUIDED_NOGPS" }
}};

/** APM:Rover custom mode -> string
 *
 * APMrover2/defines.h
 */
static const cmode_map apmrover2_cmode_map{{
	{ 0, "MANUAL" },
	{ 1, "ACRO" },
	{ 3, "STEERING" },
	{ 4, "HOLD" },
	{ 5, "LOITER" },
	{ 6, "FOLLOW" },
	{ 7, "SIMPLE" },
	{ 10, "AUTO" },
	{ 11, "RTL" },
	{ 12, "SMART_RTL" },
	{ 15, "GUIDED" },
	{ 16, "INITIALISING" }
}};

/** ArduSub custom mode -> string
 *
 * @note Modes marked n/a is not implemented (defines.h comments)
 *
 * ArduSub/defines.h
 */
static const cmode_map ardusub_cmode_map{{
	{ 0, "STABILIZE" },
	{ 1, "ACRO" },
	{ 2, "ALT_HOLD" },
	{ 3, "AUTO" },		// n/a
	{ 4, "GUIDED" },	// n/a
	{ 5, "VELHOLD" },
	{ 6, "RTL" },		// n/a
	{ 7, "CIRCLE" },	// n/a
	{ 9, "SURFACE" },
	{ 10, "OF_LOITER" },	// deprecated
	{ 11, "DRIFT" },	// n/a
	{ 13, "TRANSECT" },
	{ 14, "FLIP" },		// n/a
	{ 15, "AUTOTUNE" },	// n/a
	{ 16, "POSHOLD" },
	{ 17, "BRAKE" },	// n/a
	{ 18, "THROW" },
	{ 19, "MANUAL" }
}};

//! PX4 custom mode -> string
static const cmode_map px4_cmode_map{{
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_MANUAL),           "MANUAL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_ACRO),             "ACRO" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_ALTCTL),           "ALTCTL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_POSCTL),           "POSCTL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_OFFBOARD),         "OFFBOARD" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_STABILIZED),       "STABILIZED" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_RATTITUDE),        "RATTITUDE" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_MISSION), "AUTO.MISSION" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_LOITER),  "AUTO.LOITER" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_RTL),     "AUTO.RTL" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_LAND),    "AUTO.LAND" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_RTGS),    "AUTO.RTGS" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_READY),   "AUTO.READY" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_TAKEOFF), "AUTO.TAKEOFF" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_FOLLOW_TARGET), "AUTO.FOLLOW_TARGET" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_PRECLAND), "AUTO.PRECLAND" },
}};

static inline std::string str_base_mode(int base_mode) {
	return utils::format("MODE(0x%2X)", base_mode);
}

static std::string str_custom_mode(uint32_t custom_mode) {
	return utils::format("CMODE(%u)", custom_mode);
}

static std::string str_mode_cmap(const cmode_map &cmap, uint32_t custom_mode)
{
	auto it = cmap.find(custom_mode);
	if (it != cmap.end())
		return it->second;
	else
		return str_custom_mode(custom_mode);
}

static inline std::string str_mode_px4(uint32_t custom_mode_int)
{
	px4::custom_mode custom_mode(custom_mode_int);

	// clear fields
	custom_mode.reserved = 0;
	if (custom_mode.main_mode != px4::custom_mode::MAIN_MODE_AUTO) {
		ROS_WARN_COND_NAMED(custom_mode.sub_mode != 0, "uas", "PX4: Unknown sub-mode %d.%d",
			custom_mode.main_mode, custom_mode.sub_mode);
		custom_mode.sub_mode = 0;
	}

	return str_mode_cmap(px4_cmode_map, custom_mode.data);
}

static inline bool is_apm_copter(UAS::MAV_TYPE type)
{
	return type == UAS::MAV_TYPE::QUADROTOR ||
	       type == UAS::MAV_TYPE::HEXAROTOR ||
	       type == UAS::MAV_TYPE::OCTOROTOR ||
	       type == UAS::MAV_TYPE::TRICOPTER ||
	       type == UAS::MAV_TYPE::COAXIAL;
}

std::string UAS::str_mode_v10(uint8_t base_mode, uint32_t custom_mode)
{
	if (!(base_mode & enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED)))
		return str_base_mode(base_mode);

	auto type = get_type();
	auto ap = get_autopilot();
	if (MAV_AUTOPILOT::ARDUPILOTMEGA == ap) {
		if (is_apm_copter(type))
			return str_mode_cmap(arducopter_cmode_map, custom_mode);
		else if (type == MAV_TYPE::FIXED_WING)
			return str_mode_cmap(arduplane_cmode_map, custom_mode);
		else if (type == MAV_TYPE::GROUND_ROVER)
			return str_mode_cmap(apmrover2_cmode_map, custom_mode);
		else if (type == MAV_TYPE::SURFACE_BOAT)
			return str_mode_cmap(apmrover2_cmode_map, custom_mode);		// NOTE: #1051 for now (19.06.2018) boat is same as rover
		else if (type == MAV_TYPE::SUBMARINE)
			return str_mode_cmap(ardusub_cmode_map, custom_mode);
		else {
			ROS_WARN_THROTTLE_NAMED(30, "uas", "MODE: Unknown APM based FCU! Type: %d", enum_value(type));
			return str_custom_mode(custom_mode);
		}
	}
	else if (MAV_AUTOPILOT::PX4 == ap)
		return str_mode_px4(custom_mode);
	else
		/* TODO: other autopilot */
		return str_custom_mode(custom_mode);
}

/* XXX TODO
 * Add a fallback CMODE(dec) decoder for unknown FCU's
 */

static bool cmode_find_cmap(const cmode_map &cmap, std::string &cmode_str, uint32_t &cmode)
{
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
	ROS_INFO_STREAM_NAMED("uas", "MODE: Known modes are:" << os.str());

	return false;
}

bool UAS::cmode_from_str(std::string cmode_str, uint32_t &custom_mode)
{
	// upper case
	std::transform(cmode_str.begin(), cmode_str.end(), cmode_str.begin(), std::ref(toupper));

	auto type = get_type();
	auto ap = get_autopilot();
	if (MAV_AUTOPILOT::ARDUPILOTMEGA == ap) {
		if (is_apm_copter(type))
			return cmode_find_cmap(arducopter_cmode_map, cmode_str, custom_mode);
		else if (type == MAV_TYPE::FIXED_WING)
			return cmode_find_cmap(arduplane_cmode_map, cmode_str, custom_mode);
		else if (type == MAV_TYPE::GROUND_ROVER)
			return cmode_find_cmap(apmrover2_cmode_map, cmode_str, custom_mode);
		else if (type == MAV_TYPE::SURFACE_BOAT)
			return cmode_find_cmap(apmrover2_cmode_map, cmode_str, custom_mode);
		else if (type == MAV_TYPE::SUBMARINE)
			return cmode_find_cmap(ardusub_cmode_map, cmode_str, custom_mode);
	}
	else if (MAV_AUTOPILOT::PX4 == ap)
		return cmode_find_cmap(px4_cmode_map, cmode_str, custom_mode);

	ROS_ERROR_NAMED("uas", "MODE: Unsupported FCU");
	return false;
}
