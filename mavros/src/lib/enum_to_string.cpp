/*
 * Copyright 2016,2021 Valdimir Ermakov
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief enum stringify helpers
 * @file enum_to_string.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */

#include <array>
#include <string>
#include <unordered_map>

#include "mavros/utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mavros
{
namespace utils
{
using mavlink::minimal::MAV_AUTOPILOT;
using mavlink::minimal::MAV_TYPE;
using mavlink::minimal::MAV_STATE;
using mavlink::minimal::MAV_COMPONENT;
using mavlink::common::MAV_ESTIMATOR_TYPE;
using mavlink::common::ADSB_ALTITUDE_TYPE;
using mavlink::common::ADSB_EMITTER_TYPE;
using mavlink::common::GPS_FIX_TYPE;
using mavlink::common::MAV_MISSION_RESULT;
using mavlink::common::MAV_FRAME;
using mavlink::common::MAV_DISTANCE_SENSOR;
using mavlink::common::LANDING_TARGET_TYPE;

static auto logger = rclcpp::get_logger("uas.enum");

// [[[cog:
// import pymavlink.dialects.v20.common as common
//
// # NOTE(vooon): Foxy couldn't exclude that file from cpplint
// #              So in order to pass lint test i have to remove
// #              description comments.
// EMIT_DESCRIPTION = False
//
// def get_enum(ename):
//     enum = sorted(common.enums[ename].items())
//     enum.pop() # remove ENUM_END
//     return enum
//
//
// def split_by(delim, s):
//     for c in delim:
//         if c in s:
//             return s.split(c, 1)[0].strip()
//
//     return s.strip()
//
//
// def make_whitespace(l, v):
//     d = l - len(v)
//     return ' ' * d if d > 2 else '  '
//
//
// def ename_array_name(ename, suffix=None):
//     l = ename.rsplit('::', 1)
//     return (l[1] if len(l) > 1 else l[0]).lower() + (suffix or '_strings')
//
//
// def array_outl(name, enum, suffix=None):
//     array = ename_array_name(name, suffix)
//     cog.outl(f"""
// //! {name} values
// static const std::array<const std::string, {len(enum)}> {array}{{{{""")
//
//
// def to_string_outl(ename, funcname='to_string', suffix=None):
//     array = ename_array_name(ename, suffix)
//     cog.outl(f"""
// std::string {funcname}({ename} e)
// {{
//   size_t idx = enum_value(e);
//   if (idx >= {array}.size()) {{
//     return std::to_string(idx);
//   }}
//
//   return {array}[idx];
// }}""")
//
//
// def enum_value_is_description_outl(ename, suffix=None,
//     split_by_delim='-,/.', funcname='to_string'):
//     enum = get_enum(ename)
//
//     array_outl(ename, enum, suffix)
//     for k, e in enum:
//         value = split_by(split_by_delim, e.description)
//         sp = make_whitespace(30, value)
//         if EMIT_DESCRIPTION and e.description:
//             cog.outl(f"""/* {k:>2} */ "{value}",{sp}// {e.description}""")
//         else:
//             cog.outl(f"""/* {k:>2} */ "{value}",""")
//
//     cog.outl("}};")
//     cog.outl()
//     to_string_outl(ename, funcname, suffix)
//
//
// def enum_value_is_name_outl(ename, suffix=None, funcname='to_string'):
//     enum = get_enum(ename)
//
//     array_outl(ename, enum, suffix)
//     for k, e in enum:
//         name_short =  e.name[len(ename) + 1:]
//         sp = make_whitespace(30, name_short)
//         if EMIT_DESCRIPTION and e.description:
//             cog.outl(f"""/* {k:>2} */ "{name_short}",{sp}// {e.description}""")
//         else:
//             cog.outl(f"""/* {k:>2} */ "{name_short}",""")
//
//     cog.outl("}};")
//     to_string_outl(ename, funcname, suffix)
// ]]]
// [[[end]]] (checksum: d41d8cd98f00b204e9800998ecf8427e)

// [[[cog:
// ename = 'MAV_AUTOPILOT'
// enum_value_is_description_outl(ename)
// ]]]

//! MAV_AUTOPILOT values
static const std::array<const std::string, 21> mav_autopilot_strings{{
/*  0 */ "Generic autopilot",
/*  1 */ "Reserved for future use",
/*  2 */ "SLUGS autopilot",
/*  3 */ "ArduPilot",
/*  4 */ "OpenPilot",
/*  5 */ "Generic autopilot only supporting simple waypoints",
/*  6 */ "Generic autopilot supporting waypoints and other simple navigation commands",
/*  7 */ "Generic autopilot supporting the full mission command set",
/*  8 */ "No valid autopilot",
/*  9 */ "PPZ UAV",
/* 10 */ "UAV Dev Board",
/* 11 */ "FlexiPilot",
/* 12 */ "PX4 Autopilot",
/* 13 */ "SMACCMPilot",
/* 14 */ "AutoQuad",
/* 15 */ "Armazila",
/* 16 */ "Aerob",
/* 17 */ "ASLUAV autopilot",
/* 18 */ "SmartAP Autopilot",
/* 19 */ "AirRails",
/* 20 */ "Fusion Reflex",
}};


std::string to_string(MAV_AUTOPILOT e)
{
  size_t idx = enum_value(e);
  if (idx >= mav_autopilot_strings.size()) {
    return std::to_string(idx);
  }

  return mav_autopilot_strings[idx];
}
// [[[end]]] (checksum: 68013f2988194a55231693f1d7fa9726)

// [[[cog:
// ename = 'MAV_TYPE'
// enum_value_is_description_outl(ename)
// ]]]

//! MAV_TYPE values
static const std::array<const std::string, 43> mav_type_strings{{
/*  0 */ "Generic micro air vehicle",
/*  1 */ "Fixed wing aircraft",
/*  2 */ "Quadrotor",
/*  3 */ "Coaxial helicopter",
/*  4 */ "Normal helicopter with tail rotor",
/*  5 */ "Ground installation",
/*  6 */ "Operator control unit",
/*  7 */ "Airship",
/*  8 */ "Free balloon",
/*  9 */ "Rocket",
/* 10 */ "Ground rover",
/* 11 */ "Surface vessel",
/* 12 */ "Submarine",
/* 13 */ "Hexarotor",
/* 14 */ "Octorotor",
/* 15 */ "Tricopter",
/* 16 */ "Flapping wing",
/* 17 */ "Kite",
/* 18 */ "Onboard companion controller",
/* 19 */ "Two",
/* 20 */ "Quad",
/* 21 */ "Tiltrotor VTOL",
/* 22 */ "VTOL reserved 2",
/* 23 */ "VTOL reserved 3",
/* 24 */ "VTOL reserved 4",
/* 25 */ "VTOL reserved 5",
/* 26 */ "Gimbal",
/* 27 */ "ADSB system",
/* 28 */ "Steerable",
/* 29 */ "Dodecarotor",
/* 30 */ "Camera",
/* 31 */ "Charging station",
/* 32 */ "FLARM collision avoidance system",
/* 33 */ "Servo",
/* 34 */ "Open Drone ID. See https:",
/* 35 */ "Decarotor",
/* 36 */ "Battery",
/* 37 */ "Parachute",
/* 38 */ "Log",
/* 39 */ "OSD",
/* 40 */ "IMU",
/* 41 */ "GPS",
/* 42 */ "Winch",
}};


std::string to_string(MAV_TYPE e)
{
  size_t idx = enum_value(e);
  if (idx >= mav_type_strings.size()) {
    return std::to_string(idx);
  }

  return mav_type_strings[idx];
}
// [[[end]]] (checksum: 5bcb1f5fb05dbb71467360a932e5b182)

// [[[cog:
// ename = 'MAV_TYPE'
// enum_value_is_name_outl(ename, funcname='enum_to_name', suffix='_names')
// ]]]

//! MAV_TYPE values
static const std::array<const std::string, 43> mav_type_names{{
/*  0 */ "GENERIC",
/*  1 */ "FIXED_WING",
/*  2 */ "QUADROTOR",
/*  3 */ "COAXIAL",
/*  4 */ "HELICOPTER",
/*  5 */ "ANTENNA_TRACKER",
/*  6 */ "GCS",
/*  7 */ "AIRSHIP",
/*  8 */ "FREE_BALLOON",
/*  9 */ "ROCKET",
/* 10 */ "GROUND_ROVER",
/* 11 */ "SURFACE_BOAT",
/* 12 */ "SUBMARINE",
/* 13 */ "HEXAROTOR",
/* 14 */ "OCTOROTOR",
/* 15 */ "TRICOPTER",
/* 16 */ "FLAPPING_WING",
/* 17 */ "KITE",
/* 18 */ "ONBOARD_CONTROLLER",
/* 19 */ "VTOL_DUOROTOR",
/* 20 */ "VTOL_QUADROTOR",
/* 21 */ "VTOL_TILTROTOR",
/* 22 */ "VTOL_RESERVED2",
/* 23 */ "VTOL_RESERVED3",
/* 24 */ "VTOL_RESERVED4",
/* 25 */ "VTOL_RESERVED5",
/* 26 */ "GIMBAL",
/* 27 */ "ADSB",
/* 28 */ "PARAFOIL",
/* 29 */ "DODECAROTOR",
/* 30 */ "CAMERA",
/* 31 */ "CHARGING_STATION",
/* 32 */ "FLARM",
/* 33 */ "SERVO",
/* 34 */ "ODID",
/* 35 */ "DECAROTOR",
/* 36 */ "BATTERY",
/* 37 */ "PARACHUTE",
/* 38 */ "LOG",
/* 39 */ "OSD",
/* 40 */ "IMU",
/* 41 */ "GPS",
/* 42 */ "WINCH",
}};

std::string enum_to_name(MAV_TYPE e)
{
  size_t idx = enum_value(e);
  if (idx >= mav_type_names.size()) {
    return std::to_string(idx);
  }

  return mav_type_names[idx];
}
// [[[end]]] (checksum: dfc98695f046912ae73ec2bd4508e9c1)

// [[[cog:
// ename = 'MAV_STATE'
// enum_value_is_name_outl(ename)
// ]]]

//! MAV_STATE values
static const std::array<const std::string, 9> mav_state_strings{{
/*  0 */ "UNINIT",
/*  1 */ "BOOT",
/*  2 */ "CALIBRATING",
/*  3 */ "STANDBY",
/*  4 */ "ACTIVE",
/*  5 */ "CRITICAL",
/*  6 */ "EMERGENCY",
/*  7 */ "POWEROFF",
/*  8 */ "FLIGHT_TERMINATION",
}};

std::string to_string(MAV_STATE e)
{
  size_t idx = enum_value(e);
  if (idx >= mav_state_strings.size()) {
    return std::to_string(idx);
  }

  return mav_state_strings[idx];
}
// [[[end]]] (checksum: e953b14d18e31abb45db4fe72ebb749f)

// [[[cog:
// ename = "timesync_mode"
// ent = [ "NONE", "MAVLINK", "ONBOARD", "PASSTHROUGH", ]
//
// array_outl(ename, ent)
// for k, e in enumerate(ent):
//     cog.outl(f"""/* {k:>2} */ "{e}",""")
//
// cog.outl("}};")
// cog.outl()
// to_string_outl(ename)
// ]]]

//! timesync_mode values
static const std::array<const std::string, 4> timesync_mode_strings{{
/*  0 */ "NONE",
/*  1 */ "MAVLINK",
/*  2 */ "ONBOARD",
/*  3 */ "PASSTHROUGH",
}};


std::string to_string(timesync_mode e)
{
  size_t idx = enum_value(e);
  if (idx >= timesync_mode_strings.size()) {
    return std::to_string(idx);
  }

  return timesync_mode_strings[idx];
}
// [[[end]]] (checksum: 7a286bcf12006fdeff1bd9fca8ce4176)

timesync_mode timesync_mode_from_str(const std::string & mode)
{
  for (size_t idx = 0; idx < timesync_mode_strings.size(); idx++) {
    if (timesync_mode_strings[idx] == mode) {
      std::underlying_type<timesync_mode>::type rv = idx;
      return static_cast<timesync_mode>(rv);
    }
  }

  RCLCPP_ERROR_STREAM(logger, "TM: Unknown mode: " << mode);
  return timesync_mode::NONE;
}

// [[[cog:
// ename = 'ADSB_ALTITUDE_TYPE'
// enum_value_is_name_outl(ename)
// ]]]

//! ADSB_ALTITUDE_TYPE values
static const std::array<const std::string, 2> adsb_altitude_type_strings{{
/*  0 */ "PRESSURE_QNH",
/*  1 */ "GEOMETRIC",
}};

std::string to_string(ADSB_ALTITUDE_TYPE e)
{
  size_t idx = enum_value(e);
  if (idx >= adsb_altitude_type_strings.size()) {
    return std::to_string(idx);
  }

  return adsb_altitude_type_strings[idx];
}
// [[[end]]] (checksum: 2e8d87a6e603b105ded642f34978fd55)

// [[[cog:
// ename = 'ADSB_EMITTER_TYPE'
// enum_value_is_name_outl(ename)
// ]]]

//! ADSB_EMITTER_TYPE values
static const std::array<const std::string, 20> adsb_emitter_type_strings{{
/*  0 */ "NO_INFO",
/*  1 */ "LIGHT",
/*  2 */ "SMALL",
/*  3 */ "LARGE",
/*  4 */ "HIGH_VORTEX_LARGE",
/*  5 */ "HEAVY",
/*  6 */ "HIGHLY_MANUV",
/*  7 */ "ROTOCRAFT",
/*  8 */ "UNASSIGNED",
/*  9 */ "GLIDER",
/* 10 */ "LIGHTER_AIR",
/* 11 */ "PARACHUTE",
/* 12 */ "ULTRA_LIGHT",
/* 13 */ "UNASSIGNED2",
/* 14 */ "UAV",
/* 15 */ "SPACE",
/* 16 */ "UNASSGINED3",
/* 17 */ "EMERGENCY_SURFACE",
/* 18 */ "SERVICE_SURFACE",
/* 19 */ "POINT_OBSTACLE",
}};

std::string to_string(ADSB_EMITTER_TYPE e)
{
  size_t idx = enum_value(e);
  if (idx >= adsb_emitter_type_strings.size()) {
    return std::to_string(idx);
  }

  return adsb_emitter_type_strings[idx];
}
// [[[end]]] (checksum: 342e71a579408cf35a4dbf8b42bd099a)

// [[[cog:
// ename = 'MAV_ESTIMATOR_TYPE'
// enum_value_is_name_outl(ename)
// ]]]

//! MAV_ESTIMATOR_TYPE values
static const std::array<const std::string, 9> mav_estimator_type_strings{{
/*  0 */ "UNKNOWN",
/*  1 */ "NAIVE",
/*  2 */ "VISION",
/*  3 */ "VIO",
/*  4 */ "GPS",
/*  5 */ "GPS_INS",
/*  6 */ "MOCAP",
/*  7 */ "LIDAR",
/*  8 */ "AUTOPILOT",
}};

std::string to_string(MAV_ESTIMATOR_TYPE e)
{
  size_t idx = enum_value(e);
  if (idx >= mav_estimator_type_strings.size()) {
    return std::to_string(idx);
  }

  return mav_estimator_type_strings[idx];
}
// [[[end]]] (checksum: 451e5fe0a2c760a5c9d4efed0f97553d)

// [[[cog:
// ename = 'GPS_FIX_TYPE'
// enum_value_is_name_outl(ename)
// ]]]

//! GPS_FIX_TYPE values
static const std::array<const std::string, 9> gps_fix_type_strings{{
/*  0 */ "NO_GPS",
/*  1 */ "NO_FIX",
/*  2 */ "2D_FIX",
/*  3 */ "3D_FIX",
/*  4 */ "DGPS",
/*  5 */ "RTK_FLOAT",
/*  6 */ "RTK_FIXED",
/*  7 */ "STATIC",
/*  8 */ "PPP",
}};

std::string to_string(GPS_FIX_TYPE e)
{
  size_t idx = enum_value(e);
  if (idx >= gps_fix_type_strings.size()) {
    return std::to_string(idx);
  }

  return gps_fix_type_strings[idx];
}
// [[[end]]] (checksum: 260b7022824b9717be95db4e4e28a8d5)

// [[[cog:
// ename = 'MAV_MISSION_RESULT'
// enum_value_is_description_outl(ename, split_by_delim='')
// ]]]

//! MAV_MISSION_RESULT values
static const std::array<const std::string, 16> mav_mission_result_strings{{
/*  0 */ "mission accepted OK",
/*  1 */ "Generic error / not accepting mission commands at all right now.",
/*  2 */ "Coordinate frame is not supported.",
/*  3 */ "Command is not supported.",
/*  4 */ "Mission items exceed storage space.",
/*  5 */ "One of the parameters has an invalid value.",
/*  6 */ "param1 has an invalid value.",
/*  7 */ "param2 has an invalid value.",
/*  8 */ "param3 has an invalid value.",
/*  9 */ "param4 has an invalid value.",
/* 10 */ "x / param5 has an invalid value.",
/* 11 */ "y / param6 has an invalid value.",
/* 12 */ "z / param7 has an invalid value.",
/* 13 */ "Mission item received out of sequence",
/* 14 */ "Not accepting any mission commands from this communication partner.",
/* 15 */ "Current mission operation cancelled (e.g. mission upload, mission download).",
}};


std::string to_string(MAV_MISSION_RESULT e)
{
  size_t idx = enum_value(e);
  if (idx >= mav_mission_result_strings.size()) {
    return std::to_string(idx);
  }

  return mav_mission_result_strings[idx];
}
// [[[end]]] (checksum: ddddde7cdf04ebd988f019d5b93eadbe)

// [[[cog:
// ename = 'MAV_FRAME'
// enum_value_is_name_outl(ename)
// ]]]

//! MAV_FRAME values
static const std::array<const std::string, 22> mav_frame_strings{{
/*  0 */ "GLOBAL",
/*  1 */ "LOCAL_NED",
/*  2 */ "MISSION",
/*  3 */ "GLOBAL_RELATIVE_ALT",
/*  4 */ "LOCAL_ENU",
/*  5 */ "GLOBAL_INT",
/*  6 */ "GLOBAL_RELATIVE_ALT_INT",
/*  7 */ "LOCAL_OFFSET_NED",
/*  8 */ "BODY_NED",
/*  9 */ "BODY_OFFSET_NED",
/* 10 */ "GLOBAL_TERRAIN_ALT",
/* 11 */ "GLOBAL_TERRAIN_ALT_INT",
/* 12 */ "BODY_FRD",
/* 13 */ "RESERVED_13",
/* 14 */ "RESERVED_14",
/* 15 */ "RESERVED_15",
/* 16 */ "RESERVED_16",
/* 17 */ "RESERVED_17",
/* 18 */ "RESERVED_18",
/* 19 */ "RESERVED_19",
/* 20 */ "LOCAL_FRD",
/* 21 */ "LOCAL_FLU",
}};

std::string to_string(MAV_FRAME e)
{
  size_t idx = enum_value(e);
  if (idx >= mav_frame_strings.size()) {
    return std::to_string(idx);
  }

  return mav_frame_strings[idx];
}
// [[[end]]] (checksum: 9e2018e38b2c586263f10adba00d2ca6)

// [[[cog:
// ename = 'MAV_COMPONENT'
// suffix = 'MAV_COMP_ID'
// enum = get_enum(ename)
//
// cog.outl(
//     f"static const std::unordered_map<typename std::underlying_type<{ename}>::type,\n"
//     f"  const std::string> {suffix.lower()}_strings{{{{")
// for k, e in enum:
//     name_short =  e.name[len(suffix) + 1:]
//     entry = f"""{{{k}, "{name_short}"}},"""
//     if EMIT_DESCRIPTION and e.description:
//         cog.outl(f"""  {entry:<39} // {e.description}""")
//     else:
//         cog.outl(f"""  {entry}""")
//
// cog.outl("}};")
// ]]]
static const std::unordered_map<typename std::underlying_type<MAV_COMPONENT>::type,
  const std::string> mav_comp_id_strings{{
  {0, "ALL"},
  {1, "AUTOPILOT1"},
  {25, "USER1"},
  {26, "USER2"},
  {27, "USER3"},
  {28, "USER4"},
  {29, "USER5"},
  {30, "USER6"},
  {31, "USER7"},
  {32, "USER8"},
  {33, "USER9"},
  {34, "USER10"},
  {35, "USER11"},
  {36, "USER12"},
  {37, "USER13"},
  {38, "USER14"},
  {39, "USER15"},
  {40, "USER16"},
  {41, "USER17"},
  {42, "USER18"},
  {43, "USER19"},
  {44, "USER20"},
  {45, "USER21"},
  {46, "USER22"},
  {47, "USER23"},
  {48, "USER24"},
  {49, "USER25"},
  {50, "USER26"},
  {51, "USER27"},
  {52, "USER28"},
  {53, "USER29"},
  {54, "USER30"},
  {55, "USER31"},
  {56, "USER32"},
  {57, "USER33"},
  {58, "USER34"},
  {59, "USER35"},
  {60, "USER36"},
  {61, "USER37"},
  {62, "USER38"},
  {63, "USER39"},
  {64, "USER40"},
  {65, "USER41"},
  {66, "USER42"},
  {67, "USER43"},
  {68, "TELEMETRY_RADIO"},
  {69, "USER45"},
  {70, "USER46"},
  {71, "USER47"},
  {72, "USER48"},
  {73, "USER49"},
  {74, "USER50"},
  {75, "USER51"},
  {76, "USER52"},
  {77, "USER53"},
  {78, "USER54"},
  {79, "USER55"},
  {80, "USER56"},
  {81, "USER57"},
  {82, "USER58"},
  {83, "USER59"},
  {84, "USER60"},
  {85, "USER61"},
  {86, "USER62"},
  {87, "USER63"},
  {88, "USER64"},
  {89, "USER65"},
  {90, "USER66"},
  {91, "USER67"},
  {92, "USER68"},
  {93, "USER69"},
  {94, "USER70"},
  {95, "USER71"},
  {96, "USER72"},
  {97, "USER73"},
  {98, "USER74"},
  {99, "USER75"},
  {100, "CAMERA"},
  {101, "CAMERA2"},
  {102, "CAMERA3"},
  {103, "CAMERA4"},
  {104, "CAMERA5"},
  {105, "CAMERA6"},
  {140, "SERVO1"},
  {141, "SERVO2"},
  {142, "SERVO3"},
  {143, "SERVO4"},
  {144, "SERVO5"},
  {145, "SERVO6"},
  {146, "SERVO7"},
  {147, "SERVO8"},
  {148, "SERVO9"},
  {149, "SERVO10"},
  {150, "SERVO11"},
  {151, "SERVO12"},
  {152, "SERVO13"},
  {153, "SERVO14"},
  {154, "GIMBAL"},
  {155, "LOG"},
  {156, "ADSB"},
  {157, "OSD"},
  {158, "PERIPHERAL"},
  {159, "QX1_GIMBAL"},
  {160, "FLARM"},
  {161, "PARACHUTE"},
  {171, "GIMBAL2"},
  {172, "GIMBAL3"},
  {173, "GIMBAL4"},
  {174, "GIMBAL5"},
  {175, "GIMBAL6"},
  {180, "BATTERY"},
  {181, "BATTERY2"},
  {189, "MAVCAN"},
  {190, "MISSIONPLANNER"},
  {191, "ONBOARD_COMPUTER"},
  {192, "ONBOARD_COMPUTER2"},
  {193, "ONBOARD_COMPUTER3"},
  {194, "ONBOARD_COMPUTER4"},
  {195, "PATHPLANNER"},
  {196, "OBSTACLE_AVOIDANCE"},
  {197, "VISUAL_INERTIAL_ODOMETRY"},
  {198, "PAIRING_MANAGER"},
  {200, "IMU"},
  {201, "IMU_2"},
  {202, "IMU_3"},
  {220, "GPS"},
  {221, "GPS2"},
  {236, "ODID_TXRX_1"},
  {237, "ODID_TXRX_2"},
  {238, "ODID_TXRX_3"},
  {240, "UDP_BRIDGE"},
  {241, "UART_BRIDGE"},
  {242, "TUNNEL_NODE"},
  {250, "SYSTEM_CONTROL"},
}};
// [[[end]]] (checksum: 3a66ba989a8794aff36d5dbadae852d8)

std::string to_string(MAV_COMPONENT e)
{
  size_t idx = enum_value(e);
  auto it = mav_comp_id_strings.find(idx);

  if (it == mav_comp_id_strings.end()) {
    return std::to_string(idx);
  }

  return it->second;
}

MAV_FRAME mav_frame_from_str(const std::string & mav_frame)
{
  for (size_t idx = 0; idx < mav_frame_strings.size(); idx++) {
    if (mav_frame_strings[idx] == mav_frame) {
      std::underlying_type<MAV_FRAME>::type rv = idx;
      return static_cast<MAV_FRAME>(rv);
    }
  }

  RCLCPP_ERROR_STREAM(logger, "FRAME: Unknown MAV_FRAME: " << mav_frame);
  return MAV_FRAME::LOCAL_NED;
}

MAV_TYPE mav_type_from_str(const std::string & mav_type)
{
  for (size_t idx = 0; idx < mav_type_names.size(); idx++) {
    if (mav_type_names[idx] == mav_type) {
      std::underlying_type<MAV_TYPE>::type rv = idx;
      return static_cast<MAV_TYPE>(rv);
    }
  }
  RCLCPP_ERROR_STREAM(logger, "TYPE: Unknown MAV_TYPE: " << mav_type);
  return MAV_TYPE::GENERIC;
}

// [[[cog:
// ename = 'MAV_DISTANCE_SENSOR'
// enum_value_is_name_outl(ename)
// ]]]

//! MAV_DISTANCE_SENSOR values
static const std::array<const std::string, 5> mav_distance_sensor_strings{{
/*  0 */ "LASER",
/*  1 */ "ULTRASOUND",
/*  2 */ "INFRARED",
/*  3 */ "RADAR",
/*  4 */ "UNKNOWN",
}};

std::string to_string(MAV_DISTANCE_SENSOR e)
{
  size_t idx = enum_value(e);
  if (idx >= mav_distance_sensor_strings.size()) {
    return std::to_string(idx);
  }

  return mav_distance_sensor_strings[idx];
}
// [[[end]]] (checksum: dda871f638e51a30d2ecd3b0d063c0de)

// [[[cog:
// ename = 'LANDING_TARGET_TYPE'
// enum_value_is_name_outl(ename)
// ]]]

//! LANDING_TARGET_TYPE values
static const std::array<const std::string, 4> landing_target_type_strings{{
/*  0 */ "LIGHT_BEACON",
/*  1 */ "RADIO_BEACON",
/*  2 */ "VISION_FIDUCIAL",
/*  3 */ "VISION_OTHER",
}};

std::string to_string(LANDING_TARGET_TYPE e)
{
  size_t idx = enum_value(e);
  if (idx >= landing_target_type_strings.size()) {
    return std::to_string(idx);
  }

  return landing_target_type_strings[idx];
}
// [[[end]]] (checksum: f582577481c6b17014ed9925665f7634)

LANDING_TARGET_TYPE landing_target_type_from_str(const std::string & landing_target_type)
{
  for (size_t idx = 0; idx < landing_target_type_strings.size(); idx++) {
    if (landing_target_type_strings[idx] == landing_target_type) {
      std::underlying_type<LANDING_TARGET_TYPE>::type rv = idx;
      return static_cast<LANDING_TARGET_TYPE>(rv);
    }
  }

  RCLCPP_ERROR_STREAM(
    logger,
    "TYPE: Unknown LANDING_TARGET_TYPE: " << landing_target_type <<
      ". Defaulting to LIGHT_BEACON");
  return LANDING_TARGET_TYPE::LIGHT_BEACON;
}

}       // namespace utils
}       // namespace mavros
