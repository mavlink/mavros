/*
 * Copyright 2014,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS UAS manager
 * @file uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <array>
#include <unordered_map>
#include <stdexcept>
#include <string>
#include <algorithm>

#include "mavros/mavros_uas.hpp"
#include "mavros/px4_custom_mode.hpp"

using namespace mavros;             // NOLINT
using namespace mavros::uas;        // NOLINT
using mavros::utils::enum_value;


/* -*- mode stringify functions -*- */

typedef std::unordered_map<uint32_t, const std::string> cmode_map;

/** APM:Plane custom mode -> string
 *
 * ArduPlane/defines.h
 */
static const cmode_map arduplane_cmode_map{{
  {0, "MANUAL"},
  {1, "CIRCLE"},
  {2, "STABILIZE"},
  {3, "TRAINING"},
  {4, "ACRO"},
  {5, "FBWA"},
  {6, "FBWB"},
  {7, "CRUISE"},
  {8, "AUTOTUNE"},
  {10, "AUTO"},
  {11, "RTL"},
  {12, "LOITER"},
  {14, "AVOID_ADSB"},           // renamed, prev name: LAND
  {15, "GUIDED"},
  {16, "INITIALISING"},
  {17, "QSTABILIZE"},           // QuadPlane
  {18, "QHOVER"},
  {19, "QLOITER"},
  {20, "QLAND"},
  {21, "QRTL"}
}};

/** APM:Copter custom mode -> string
 *
 * ArduCopter/defines.h
 */
static const cmode_map arducopter_cmode_map{{
  {0, "STABILIZE"},
  {1, "ACRO"},
  {2, "ALT_HOLD"},
  {3, "AUTO"},
  {4, "GUIDED"},
  {5, "LOITER"},
  {6, "RTL"},
  {7, "CIRCLE"},
  {8, "POSITION"},              // not in list
  {9, "LAND"},
  {10, "OF_LOITER"},
  {11, "DRIFT"},                // renamed, prev name: APPROACH
  {13, "SPORT"},
  {14, "FLIP"},
  {15, "AUTOTUNE"},
  {16, "POSHOLD"},
  {17, "BRAKE"},
  {18, "THROW"},
  {19, "AVOID_ADSB"},
  {20, "GUIDED_NOGPS"}
}};

/** APM:Rover custom mode -> string
 *
 * APMrover2/defines.h
 */
static const cmode_map apmrover2_cmode_map{{
  {0, "MANUAL"},
  {1, "ACRO"},
  {3, "STEERING"},
  {4, "HOLD"},
  {5, "LOITER"},
  {6, "FOLLOW"},
  {7, "SIMPLE"},
  {10, "AUTO"},
  {11, "RTL"},
  {12, "SMART_RTL"},
  {15, "GUIDED"},
  {16, "INITIALISING"}
}};

/** ArduSub custom mode -> string
 *
 * @note Modes marked n/a is not implemented (defines.h comments)
 *
 * ArduSub/defines.h
 */
static const cmode_map ardusub_cmode_map{{
  {0, "STABILIZE"},
  {1, "ACRO"},
  {2, "ALT_HOLD"},
  {3, "AUTO"},                  // n/a
  {4, "GUIDED"},                // n/a
  {5, "VELHOLD"},
  {6, "RTL"},                   // n/a
  {7, "CIRCLE"},                // n/a
  {9, "SURFACE"},
  {10, "OF_LOITER"},            // deprecated
  {11, "DRIFT"},                // n/a
  {13, "TRANSECT"},
  {14, "FLIP"},                 // n/a
  {15, "AUTOTUNE"},             // n/a
  {16, "POSHOLD"},
  {17, "BRAKE"},                // n/a
  {18, "THROW"},
  {19, "MANUAL"}
}};

//! PX4 custom mode <-> string, for modes that can be set by name
static const cmode_map px4_cmode_map{{
  {px4::define_mode(px4::custom_mode::MAIN_MODE_MANUAL), "MANUAL"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_ACRO), "ACRO"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_ALTCTL), "ALTCTL"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_POSCTL), "POSCTL"},
  {px4::define_mode(
      px4::custom_mode::MAIN_MODE_POSCTL,
      px4::custom_mode::SUB_MODE_POSCTL_SLOW), "POSCTL.SLOW"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_OFFBOARD), "OFFBOARD"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_STABILIZED), "STABILIZED"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_RATTITUDE), "RATTITUDE"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_ALTITUDE_CRUISE), "ALTITUDE_CRUISE"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_MANUAL_PARKING), "MANUAL_PARKING"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_MISSION), "AUTO.MISSION"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_LOITER), "AUTO.LOITER"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_RTL), "AUTO.RTL"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_LAND), "AUTO.LAND"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_RTGS), "AUTO.RTGS"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_READY), "AUTO.READY"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_TAKEOFF), "AUTO.TAKEOFF"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_FOLLOW_TARGET), "AUTO.FOLLOW_TARGET"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_PRECLAND), "AUTO.PRECLAND"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_EXTERNAL1), "AUTO.EXTERNAL1"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_EXTERNAL2), "AUTO.EXTERNAL2"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_EXTERNAL3), "AUTO.EXTERNAL3"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_EXTERNAL4), "AUTO.EXTERNAL4"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_EXTERNAL5), "AUTO.EXTERNAL5"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_EXTERNAL6), "AUTO.EXTERNAL6"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_EXTERNAL7), "AUTO.EXTERNAL7"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_EXTERNAL8), "AUTO.EXTERNAL8"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_GUIDED_COURSE), "AUTO.GUIDED_COURSE"},
}};

/**
 * PX4 custom mode -> string, for modes that PX4 reports but does not accept
 * in a set mode command: it rejects Termination, VTOL Takeoff and Descend,
 * and switches to plain Position mode for Orbit (use MAV_CMD_DO_ORBIT).
 */
static const cmode_map px4_cmode_map_report_only{{
  {px4::define_mode(
      px4::custom_mode::MAIN_MODE_POSCTL,
      px4::custom_mode::SUB_MODE_POSCTL_ORBIT), "POSCTL.ORBIT"},
  {px4::define_mode(px4::custom_mode::MAIN_MODE_TERMINATION), "TERMINATION"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_VTOL_TAKEOFF), "AUTO.VTOL_TAKEOFF"},
  {px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_DESCEND), "AUTO.DESCEND"},
}};

// Flix modes
static const cmode_map flix_cmode_map{{
  {0, "RAW"},
  {1, "ACRO"},
  {2, "STAB"},
  {3, "AUTO"},
}};

static inline std::string str_base_mode(int base_mode)
{
  return utils::format("MODE(0x%2X)", base_mode);
}

static std::string str_custom_mode(uint32_t custom_mode)
{
  return utils::format("CMODE(%u)", custom_mode);
}

static std::string str_mode_cmap(const cmode_map & cmap, uint32_t custom_mode)
{
  auto it = cmap.find(custom_mode);
  if (it != cmap.end()) {
    return it->second;
  } else {
    return str_custom_mode(custom_mode);
  }
}

static bool find_px4_mode(uint32_t custom_mode, std::string & name)
{
  for (const auto * cmap : {&px4_cmode_map, &px4_cmode_map_report_only}) {
    auto it = cmap->find(custom_mode);
    if (it != cmap->end()) {
      name = it->second;
      return true;
    }
  }
  return false;
}

static std::string str_mode_px4(
  uint32_t custom_mode_int, const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  px4::custom_mode custom_mode(custom_mode_int);

  // clear fields
  custom_mode.mode.reserved = 0;

  std::string name;
  if (find_px4_mode(custom_mode.data, name)) {
    return name;
  }

  if (custom_mode.mode.sub_mode != 0) {
    // Unknown sub mode: fall back to the main mode name, if it has one.
    RCLCPP_WARN_THROTTLE(
      logger, clock, 30000, "PX4: Unknown sub-mode %d.%d",
      custom_mode.mode.main_mode, custom_mode.mode.sub_mode);

    const px4::custom_mode main_mode_only(custom_mode.mode.main_mode, 0);
    if (find_px4_mode(main_mode_only.data, name)) {
      return name;
    }
  }

  return str_custom_mode(custom_mode.data);
}

static inline bool is_apm_copter(uas::MAV_TYPE type)
{
  return type == uas::MAV_TYPE::QUADROTOR ||
         type == uas::MAV_TYPE::HEXAROTOR ||
         type == uas::MAV_TYPE::OCTOROTOR ||
         type == uas::MAV_TYPE::TRICOPTER ||
         type == uas::MAV_TYPE::COAXIAL;
}

std::string UAS::str_mode_v10(uint8_t base_mode, uint32_t custom_mode)
{
  auto type = get_type();
  auto ap = get_autopilot();
  if (MAV_AUTOPILOT::ARDUPILOTMEGA == ap) {
    if (is_apm_copter(type)) {
      return str_mode_cmap(arducopter_cmode_map, custom_mode);
    } else if (type == MAV_TYPE::FIXED_WING) {
      return str_mode_cmap(arduplane_cmode_map, custom_mode);
    } else if (type == MAV_TYPE::GROUND_ROVER) {
      return str_mode_cmap(apmrover2_cmode_map, custom_mode);
    } else if (type == MAV_TYPE::SURFACE_BOAT) {
      // NOTE(vooon): #1051 for now (19.06.2018) boat is same as rover
      return str_mode_cmap(apmrover2_cmode_map, custom_mode);
    } else if (type == MAV_TYPE::SUBMARINE) {
      return str_mode_cmap(ardusub_cmode_map, custom_mode);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 30000, "MODE: Unknown APM based FCU! Type: %d", enum_value(type));
      return str_custom_mode(custom_mode);
    }
  } else if (MAV_AUTOPILOT::PX4 == ap) {
    return str_mode_px4(custom_mode, get_logger(), *get_clock());
  } else if (MAV_AUTOPILOT::FLIX == ap) {
    return str_mode_cmap(flix_cmode_map, custom_mode);
  } else if (!(base_mode & enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED))) {
    return str_base_mode(base_mode);
  } else {
    /* TODO(vooon): other autopilot */
    return str_custom_mode(custom_mode);
  }
}

/* XXX TODO
 * Add a fallback CMODE(dec) decoder for unknown FCU's
 */

static bool cmode_find_cmap(const cmode_map & cmap, std::string & cmode_str, uint32_t & cmode)
{
  // 1. try find by name
  for (auto & mode : cmap) {
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
  } catch (std::invalid_argument & ex) {
    // failed
  }

  // Debugging output.
  std::ostringstream os;
  for (auto & mode : cmap) {
    os << " " << mode.second;
  }

  auto lg = rclcpp::get_logger("uas");
  RCLCPP_ERROR_STREAM(lg, "MODE: Unknown mode: " << cmode_str);
  RCLCPP_INFO_STREAM(lg, "MODE: Known modes are:" << os.str());

  return false;
}

bool UAS::cmode_from_str(std::string cmode_str, uint32_t & custom_mode)
{
  // upper case
  std::transform(cmode_str.begin(), cmode_str.end(), cmode_str.begin(), std::ref(toupper));

  auto type = get_type();
  auto ap = get_autopilot();
  if (MAV_AUTOPILOT::ARDUPILOTMEGA == ap) {
    if (is_apm_copter(type)) {
      return cmode_find_cmap(arducopter_cmode_map, cmode_str, custom_mode);
    } else if (type == MAV_TYPE::FIXED_WING) {
      return cmode_find_cmap(arduplane_cmode_map, cmode_str, custom_mode);
    } else if (type == MAV_TYPE::GROUND_ROVER) {
      return cmode_find_cmap(apmrover2_cmode_map, cmode_str, custom_mode);
    } else if (type == MAV_TYPE::SURFACE_BOAT) {
      return cmode_find_cmap(apmrover2_cmode_map, cmode_str, custom_mode);
    } else if (type == MAV_TYPE::SUBMARINE) {
      return cmode_find_cmap(ardusub_cmode_map, cmode_str, custom_mode);
    }
  } else if (MAV_AUTOPILOT::PX4 == ap) {
    return cmode_find_cmap(px4_cmode_map, cmode_str, custom_mode);
  }

  RCLCPP_ERROR(get_logger(), "MODE: Unsupported FCU");
  return false;
}
