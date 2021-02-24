/**
 * @brief enum stringify helpers
 * @file enum_to_string.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2016 Valdimir Ermakov
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <array>
#include <unordered_map>
#include <mavros/utils.h>
#include <ros/console.h>

namespace mavros {
namespace utils {
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

// [[[cog:
// import pymavlink.dialects.v20.common as common
//
// def get_enum(ename):
//     enum = sorted(common.enums[ename].items())
//     enum.pop() # remove ENUM_END
//     return enum
//
// def split_by(delim, s):
//     for c in delim:
//         if c in s:
//             return s.split(c, 1)[0].strip()
//
//     return s.strip()
//
// def make_whitespace(l, v):
//     d = l - len(v)
//     return ' ' * d if d > 0 else ' '
//
// def ename_array_name(ename, suffix=None):
//     l = ename.rsplit('::', 1)
//     return (l[1] if len(l) > 1 else l[0]).lower() + (suffix or '_strings')
//
// def array_outl(name, enum, suffix=None):
//     array = ename_array_name(name, suffix)
//     cog.outl(f"""\
// //! {name} values
// static const std::array<const std::string, {len(enum)}> {array}{{{{""")
//
// def to_string_outl(ename, funcname='to_string', suffix=None):
//     array = ename_array_name(ename, suffix)
//     cog.outl(f"""\
// std::string {funcname}({ename} e)
// {{
// 	size_t idx = enum_value(e);
// 	if (idx >= {array}.size())
// 		return std::to_string(idx);
//
// 	return {array}[idx];
// }}""")
//
// def enum_name_is_value_outl(ename, suffix=None, funcname='to_string'):
//     enum = get_enum(ename)
//
//     array_outl(ename, enum, suffix)
//     for k, e in enum:
//         name_short =  e.name[len(ename) + 1:]
//         sp = make_whitespace(30, name_short)
//         if e.description:
//             cog.outl(f"""/* {k:>2} */ "{name_short}",{sp}// {e.description}""")
//         else:
//             cog.outl(f"""/* {k:>2} */ "{name_short}",""")
//
//     cog.outl("}};")
//     cog.outl()
//     to_string_outl(ename, funcname, suffix)
//
// ename = 'MAV_AUTOPILOT'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = split_by('-,/.', e.description)
//     sp = make_whitespace(30, value)
//     cog.outl(f"""/* {k:>2} */ "{value}",{sp}// {e.description}""")
//
// cog.outl("}};")
// cog.outl()
// to_string_outl(ename)
// ]]]
//! MAV_AUTOPILOT values
static const std::array<const std::string, 20> mav_autopilot_strings{{
/*  0 */ "Generic autopilot",             // Generic autopilot, full support for everything
/*  1 */ "Reserved for future use",       // Reserved for future use.
/*  2 */ "SLUGS autopilot",               // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
/*  3 */ "ArduPilot",                     // ArduPilot - Plane/Copter/Rover/Sub/Tracker, http://ardupilot.org
/*  4 */ "OpenPilot",                     // OpenPilot, http://openpilot.org
/*  5 */ "Generic autopilot only supporting simple waypoints", // Generic autopilot only supporting simple waypoints
/*  6 */ "Generic autopilot supporting waypoints and other simple navigation commands", // Generic autopilot supporting waypoints and other simple navigation commands
/*  7 */ "Generic autopilot supporting the full mission command set", // Generic autopilot supporting the full mission command set
/*  8 */ "No valid autopilot",            // No valid autopilot, e.g. a GCS or other MAVLink component
/*  9 */ "PPZ UAV",                       // PPZ UAV - http://nongnu.org/paparazzi
/* 10 */ "UAV Dev Board",                 // UAV Dev Board
/* 11 */ "FlexiPilot",                    // FlexiPilot
/* 12 */ "PX4 Autopilot",                 // PX4 Autopilot - http://px4.io/
/* 13 */ "SMACCMPilot",                   // SMACCMPilot - http://smaccmpilot.org
/* 14 */ "AutoQuad",                      // AutoQuad -- http://autoquad.org
/* 15 */ "Armazila",                      // Armazila -- http://armazila.com
/* 16 */ "Aerob",                         // Aerob -- http://aerob.ru
/* 17 */ "ASLUAV autopilot",              // ASLUAV autopilot -- http://www.asl.ethz.ch
/* 18 */ "SmartAP Autopilot",             // SmartAP Autopilot - http://sky-drones.com
/* 19 */ "AirRails",                      // AirRails - http://uaventure.com
}};

std::string to_string(MAV_AUTOPILOT e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_autopilot_strings.size())
		return std::to_string(idx);

	return mav_autopilot_strings[idx];
}
// [[[end]]] (checksum: 4b5a1e0cd8f9d21b956818e7efa3bc2e)

// [[[cog:
// ename = 'MAV_TYPE'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = split_by(',-/.', e.description)
//     sp = make_whitespace(30, value)
//     cog.outl(f"""/* {k:>2} */ "{value}",{sp}// {e.description}""")
//
// cog.outl("}};")
// cog.outl()
// to_string_outl(ename)
// ]]]
//! MAV_TYPE values
static const std::array<const std::string, 33> mav_type_strings{{
/*  0 */ "Generic micro air vehicle",     // Generic micro air vehicle.
/*  1 */ "Fixed wing aircraft",           // Fixed wing aircraft.
/*  2 */ "Quadrotor",                     // Quadrotor
/*  3 */ "Coaxial helicopter",            // Coaxial helicopter
/*  4 */ "Normal helicopter with tail rotor", // Normal helicopter with tail rotor.
/*  5 */ "Ground installation",           // Ground installation
/*  6 */ "Operator control unit",         // Operator control unit / ground control station
/*  7 */ "Airship",                       // Airship, controlled
/*  8 */ "Free balloon",                  // Free balloon, uncontrolled
/*  9 */ "Rocket",                        // Rocket
/* 10 */ "Ground rover",                  // Ground rover
/* 11 */ "Surface vessel",                // Surface vessel, boat, ship
/* 12 */ "Submarine",                     // Submarine
/* 13 */ "Hexarotor",                     // Hexarotor
/* 14 */ "Octorotor",                     // Octorotor
/* 15 */ "Tricopter",                     // Tricopter
/* 16 */ "Flapping wing",                 // Flapping wing
/* 17 */ "Kite",                          // Kite
/* 18 */ "Onboard companion controller",  // Onboard companion controller
/* 19 */ "Two",                           // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
/* 20 */ "Quad",                          // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
/* 21 */ "Tiltrotor VTOL",                // Tiltrotor VTOL
/* 22 */ "VTOL reserved 2",               // VTOL reserved 2
/* 23 */ "VTOL reserved 3",               // VTOL reserved 3
/* 24 */ "VTOL reserved 4",               // VTOL reserved 4
/* 25 */ "VTOL reserved 5",               // VTOL reserved 5
/* 26 */ "Gimbal (standalone)",           // Gimbal (standalone)
/* 27 */ "ADSB system (standalone)",      // ADSB system (standalone)
/* 28 */ "Steerable",                     // Steerable, nonrigid airfoil
/* 29 */ "Dodecarotor",                   // Dodecarotor
/* 30 */ "Camera (standalone)",           // Camera (standalone)
/* 31 */ "Charging station",              // Charging station
/* 32 */ "FLARM collision avoidance system (standalone)", // FLARM collision avoidance system (standalone)
}};

std::string to_string(MAV_TYPE e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_type_strings.size())
		return std::to_string(idx);

	return mav_type_strings[idx];
}
// [[[end]]] (checksum: b441cfd88ee1a6fc8c8bc8f4e6df5825)

// [[[cog:
// ename = 'MAV_TYPE'
// enum_name_is_value_outl(ename, funcname='to_name', suffix='_names')
// ]]]
//! MAV_TYPE values
static const std::array<const std::string, 33> mav_type_names{{
/*  0 */ "GENERIC",                       // Generic micro air vehicle.
/*  1 */ "FIXED_WING",                    // Fixed wing aircraft.
/*  2 */ "QUADROTOR",                     // Quadrotor
/*  3 */ "COAXIAL",                       // Coaxial helicopter
/*  4 */ "HELICOPTER",                    // Normal helicopter with tail rotor.
/*  5 */ "ANTENNA_TRACKER",               // Ground installation
/*  6 */ "GCS",                           // Operator control unit / ground control station
/*  7 */ "AIRSHIP",                       // Airship, controlled
/*  8 */ "FREE_BALLOON",                  // Free balloon, uncontrolled
/*  9 */ "ROCKET",                        // Rocket
/* 10 */ "GROUND_ROVER",                  // Ground rover
/* 11 */ "SURFACE_BOAT",                  // Surface vessel, boat, ship
/* 12 */ "SUBMARINE",                     // Submarine
/* 13 */ "HEXAROTOR",                     // Hexarotor
/* 14 */ "OCTOROTOR",                     // Octorotor
/* 15 */ "TRICOPTER",                     // Tricopter
/* 16 */ "FLAPPING_WING",                 // Flapping wing
/* 17 */ "KITE",                          // Kite
/* 18 */ "ONBOARD_CONTROLLER",            // Onboard companion controller
/* 19 */ "VTOL_DUOROTOR",                 // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
/* 20 */ "VTOL_QUADROTOR",                // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
/* 21 */ "VTOL_TILTROTOR",                // Tiltrotor VTOL
/* 22 */ "VTOL_RESERVED2",                // VTOL reserved 2
/* 23 */ "VTOL_RESERVED3",                // VTOL reserved 3
/* 24 */ "VTOL_RESERVED4",                // VTOL reserved 4
/* 25 */ "VTOL_RESERVED5",                // VTOL reserved 5
/* 26 */ "GIMBAL",                        // Gimbal (standalone)
/* 27 */ "ADSB",                          // ADSB system (standalone)
/* 28 */ "PARAFOIL",                      // Steerable, nonrigid airfoil
/* 29 */ "DODECAROTOR",                   // Dodecarotor
/* 30 */ "CAMERA",                        // Camera (standalone)
/* 31 */ "CHARGING_STATION",              // Charging station
/* 32 */ "FLARM",                         // FLARM collision avoidance system (standalone)
}};

std::string to_name(MAV_TYPE e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_type_names.size())
		return std::to_string(idx);

	return mav_type_names[idx];
}
// [[[end]]] (checksum: 6accf424d6136b55b15efbb86bb0c5cd)

// [[[cog:
// ename = 'MAV_STATE'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = e.name[len(ename) + 1:].title()
//     sp = make_whitespace(30, value)
//     cog.outl("""/* {k:>2} */ "{value}",{sp}// {e.description}""".format(**locals()))
//
// cog.outl("}};")
// cog.outl()
// to_string_outl(ename)
// ]]]
//! MAV_STATE values
static const std::array<const std::string, 9> mav_state_strings{{
/*  0 */ "Uninit",                        // Uninitialized system, state is unknown.
/*  1 */ "Boot",                          // System is booting up.
/*  2 */ "Calibrating",                   // System is calibrating and not flight-ready.
/*  3 */ "Standby",                       // System is grounded and on standby. It can be launched any time.
/*  4 */ "Active",                        // System is active and might be already airborne. Motors are engaged.
/*  5 */ "Critical",                      // System is in a non-normal flight mode. It can however still navigate.
/*  6 */ "Emergency",                     // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
/*  7 */ "Poweroff",                      // System just initialized its power-down sequence, will shut down now.
/*  8 */ "Flight_Termination",            // System is terminating itself.
}};

std::string to_string(MAV_STATE e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_state_strings.size())
		return std::to_string(idx);

	return mav_state_strings[idx];
}
// [[[end]]] (checksum: 8af1e6916d0229c193aab7d3dc2c97e9)

// [[[cog:
// ename = "timesync_mode"
// ent = [ "NONE", "MAVLINK", "ONBOARD", "PASSTHROUGH", ]
//
// array_outl(ename, ent)
// for k, e in enumerate(ent):
//     cog.outl("""/* {k:>2} */ "{e}",""".format(**locals()))
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
	if (idx >= timesync_mode_strings.size())
		return std::to_string(idx);

	return timesync_mode_strings[idx];
}
// [[[end]]] (checksum: 2796eaa4f9361c2d7ca87f63e0401d4d)

timesync_mode timesync_mode_from_str(const std::string &mode)
{
	for (size_t idx = 0; idx < timesync_mode_strings.size(); idx++) {
		if (timesync_mode_strings[idx] == mode) {
			std::underlying_type<timesync_mode>::type rv = idx;
			return static_cast<timesync_mode>(rv);
		}
	}

	ROS_ERROR_STREAM_NAMED("uas", "TM: Unknown mode: " << mode);
	return timesync_mode::NONE;
}

// [[[cog:
// ename = 'ADSB_ALTITUDE_TYPE'
// enum_name_is_value_outl(ename)
// ]]]
//! ADSB_ALTITUDE_TYPE values
static const std::array<const std::string, 2> adsb_altitude_type_strings{{
/*  0 */ "PRESSURE_QNH",                  // Altitude reported from a Baro source using QNH reference
/*  1 */ "GEOMETRIC",                     // Altitude reported from a GNSS source
}};

std::string to_string(ADSB_ALTITUDE_TYPE e)
{
	size_t idx = enum_value(e);
	if (idx >= adsb_altitude_type_strings.size())
		return std::to_string(idx);

	return adsb_altitude_type_strings[idx];
}
// [[[end]]] (checksum: dc127bf29aefa513471d13c5a0e1e6ec)

// [[[cog:
// ename = 'ADSB_EMITTER_TYPE'
// enum_name_is_value_outl(ename)
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
	if (idx >= adsb_emitter_type_strings.size())
		return std::to_string(idx);

	return adsb_emitter_type_strings[idx];
}
// [[[end]]] (checksum: 713e0304603321e421131d8552d0f8e0)

// [[[cog:
// ename = 'MAV_ESTIMATOR_TYPE'
// enum_name_is_value_outl(ename)
// ]]]
//! MAV_ESTIMATOR_TYPE values
static const std::array<const std::string, 5> mav_estimator_type_strings{{
/*  1 */ "NAIVE",                         // This is a naive estimator without any real covariance feedback.
/*  2 */ "VISION",                        // Computer vision based estimate. Might be up to scale.
/*  3 */ "VIO",                           // Visual-inertial estimate.
/*  4 */ "GPS",                           // Plain GPS estimate.
/*  5 */ "GPS_INS",                       // Estimator integrating GPS and inertial sensing.
}};

std::string to_string(MAV_ESTIMATOR_TYPE e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_estimator_type_strings.size())
		return std::to_string(idx);

	return mav_estimator_type_strings[idx];
}
// [[[end]]] (checksum: 47674f004bf6c515fdf999987b99e806)

// [[[cog:
// ename = 'GPS_FIX_TYPE'
// enum_name_is_value_outl(ename)
// ]]]
//! GPS_FIX_TYPE values
static const std::array<const std::string, 9> gps_fix_type_strings{{
/*  0 */ "NO_GPS",                        // No GPS connected
/*  1 */ "NO_FIX",                        // No position information, GPS is connected
/*  2 */ "2D_FIX",                        // 2D position
/*  3 */ "3D_FIX",                        // 3D position
/*  4 */ "DGPS",                          // DGPS/SBAS aided 3D position
/*  5 */ "RTK_FLOAT",                     // RTK float, 3D position
/*  6 */ "RTK_FIXED",                     // RTK Fixed, 3D position
/*  7 */ "STATIC",                        // Static fixed, typically used for base stations
/*  8 */ "PPP",                           // PPP, 3D position.
}};

std::string to_string(GPS_FIX_TYPE e)
{
	size_t idx = enum_value(e);
	if (idx >= gps_fix_type_strings.size())
		return std::to_string(idx);

	return gps_fix_type_strings[idx];
}
// [[[end]]] (checksum: 7569b73b2d68ed1412bf0c36afeb131c)

// [[[cog:
// ename = 'MAV_MISSION_RESULT'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = e.description
//     sp = make_whitespace(30, value)
//     cog.outl("""/* {k:>2} */ "{value}",{sp}// {e.description}""".format(**locals()))
//
// cog.outl("}};")
// cog.outl()
// to_string_outl(ename)
// ]]]
//! MAV_MISSION_RESULT values
static const std::array<const std::string, 16> mav_mission_result_strings{{
/*  0 */ "mission accepted OK",           // mission accepted OK
/*  1 */ "Generic error / not accepting mission commands at all right now.", // Generic error / not accepting mission commands at all right now.
/*  2 */ "Coordinate frame is not supported.", // Coordinate frame is not supported.
/*  3 */ "Command is not supported.",     // Command is not supported.
/*  4 */ "Mission item exceeds storage space.", // Mission item exceeds storage space.
/*  5 */ "One of the parameters has an invalid value.", // One of the parameters has an invalid value.
/*  6 */ "param1 has an invalid value.",  // param1 has an invalid value.
/*  7 */ "param2 has an invalid value.",  // param2 has an invalid value.
/*  8 */ "param3 has an invalid value.",  // param3 has an invalid value.
/*  9 */ "param4 has an invalid value.",  // param4 has an invalid value.
/* 10 */ "x / param5 has an invalid value.", // x / param5 has an invalid value.
/* 11 */ "y / param6 has an invalid value.", // y / param6 has an invalid value.
/* 12 */ "z / param7 has an invalid value.", // z / param7 has an invalid value.
/* 13 */ "Mission item received out of sequence", // Mission item received out of sequence
/* 14 */ "Not accepting any mission commands from this communication partner.", // Not accepting any mission commands from this communication partner.
/* 15 */ "Current mission operation cancelled (e.g. mission upload, mission download).", // Current mission operation cancelled (e.g. mission upload, mission download).
}};

std::string to_string(MAV_MISSION_RESULT e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_mission_result_strings.size())
		return std::to_string(idx);

	return mav_mission_result_strings[idx];
}
// [[[end]]] (checksum: d42db24957df1950d06edbf9480dde46)

// [[[cog:
// ename = 'MAV_FRAME'
// enum_name_is_value_outl(ename)
// ]]]
//! MAV_FRAME values
static const std::array<const std::string, 20> mav_frame_strings{{
/*  0 */ "GLOBAL",                        // Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).
/*  1 */ "LOCAL_NED",                     // Local coordinate frame, Z-down (x: north, y: east, z: down).
/*  2 */ "MISSION",                       // NOT a coordinate frame, indicates a mission command.
/*  3 */ "GLOBAL_RELATIVE_ALT",           // Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
/*  4 */ "LOCAL_ENU",                     // Local coordinate frame, Z-up (x: east, y: north, z: up).
/*  5 */ "GLOBAL_INT",                    // Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL).
/*  6 */ "GLOBAL_RELATIVE_ALT_INT",       // Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
/*  7 */ "LOCAL_OFFSET_NED",              // Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
/*  8 */ "BODY_NED",                      // Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
/*  9 */ "BODY_OFFSET_NED",               // Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
/* 10 */ "GLOBAL_TERRAIN_ALT",            // Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
/* 11 */ "GLOBAL_TERRAIN_ALT_INT",        // Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
/* 12 */ "BODY_FRD",                      // Body fixed frame of reference, Z-down (x: forward, y: right, z: down).
/* 13 */ "BODY_FLU",                      // Body fixed frame of reference, Z-up (x: forward, y: left, z: up).
/* 14 */ "MOCAP_NED",                     // Odometry local coordinate frame of data given by a motion capture system, Z-down (x: north, y: east, z: down).
/* 15 */ "MOCAP_ENU",                     // Odometry local coordinate frame of data given by a motion capture system, Z-up (x: east, y: north, z: up).
/* 16 */ "VISION_NED",                    // Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: north, y: east, z: down).
/* 17 */ "VISION_ENU",                    // Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: east, y: north, z: up).
/* 18 */ "ESTIM_NED",                     // Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: north, y: east, z: down).
/* 19 */ "ESTIM_ENU",                     // Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: east, y: noth, z: up).
}};

std::string to_string(MAV_FRAME e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_frame_strings.size())
		return std::to_string(idx);

	return mav_frame_strings[idx];
}
// [[[end]]] (checksum: a075e35372e9be53a3a0ce79e45236c1)

// [[[cog:
// ename = 'MAV_COMPONENT'
// suffix = 'MAV_COMP_ID'
// enum = get_enum(ename)
//
// cog.outl(f"static const std::unordered_map<size_t, const std::string> {suffix.lower()}_strings{{{{")
// for k, e in enum:
//     name_short =  e.name[len(suffix) + 1:]
//     sp = make_whitespace(30, name_short)
//     if e.description:
//         cog.outl(f"""{{ {k:>3}, "{name_short}" }},{sp}// {e.description}""")
//     else:
//         cog.outl(f"""{{ {k:>3}, "{name_short}" }},""")
//
// cog.outl("}};")
// ]]]
static const std::unordered_map<size_t, const std::string> mav_comp_id_strings{{
{   0, "ALL" },                           // Used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces.
{   1, "AUTOPILOT1" },                    // System flight controller component ("autopilot"). Only one autopilot is expected in a particular system.
{ 100, "CAMERA" },                        // Camera #1.
{ 101, "CAMERA2" },                       // Camera #2.
{ 102, "CAMERA3" },                       // Camera #3.
{ 103, "CAMERA4" },                       // Camera #4.
{ 104, "CAMERA5" },                       // Camera #5.
{ 105, "CAMERA6" },                       // Camera #6.
{ 140, "SERVO1" },                        // Servo #1.
{ 141, "SERVO2" },                        // Servo #2.
{ 142, "SERVO3" },                        // Servo #3.
{ 143, "SERVO4" },                        // Servo #4.
{ 144, "SERVO5" },                        // Servo #5.
{ 145, "SERVO6" },                        // Servo #6.
{ 146, "SERVO7" },                        // Servo #7.
{ 147, "SERVO8" },                        // Servo #8.
{ 148, "SERVO9" },                        // Servo #9.
{ 149, "SERVO10" },                       // Servo #10.
{ 150, "SERVO11" },                       // Servo #11.
{ 151, "SERVO12" },                       // Servo #12.
{ 152, "SERVO13" },                       // Servo #13.
{ 153, "SERVO14" },                       // Servo #14.
{ 154, "GIMBAL" },                        // Gimbal component.
{ 155, "LOG" },                           // Logging component.
{ 156, "ADSB" },                          // Automatic Dependent Surveillance-Broadcast (ADS-B) component.
{ 157, "OSD" },                           // On Screen Display (OSD) devices for video links.
{ 158, "PERIPHERAL" },                    // Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice.
{ 159, "QX1_GIMBAL" },                    // Gimbal ID for QX1.
{ 160, "FLARM" },                         // FLARM collision alert component.
{ 190, "MISSIONPLANNER" },                // Component that can generate/supply a mission flight plan (e.g. GCS or developer API).
{ 195, "PATHPLANNER" },                   // Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.).
{ 196, "OBSTACLE_AVOIDANCE" },            // Component that plans a collision free path between two points.
{ 197, "VISUAL_INERTIAL_ODOMETRY" },      // Component that provides position estimates using VIO techniques.
{ 200, "IMU" },                           // Inertial Measurement Unit (IMU) #1.
{ 201, "IMU_2" },                         // Inertial Measurement Unit (IMU) #2.
{ 202, "IMU_3" },                         // Inertial Measurement Unit (IMU) #3.
{ 220, "GPS" },                           // GPS #1.
{ 221, "GPS2" },                          // GPS #2.
{ 240, "UDP_BRIDGE" },                    // Component to bridge MAVLink to UDP (i.e. from a UART).
{ 241, "UART_BRIDGE" },                   // Component to bridge to UART (i.e. from UDP).
{ 250, "SYSTEM_CONTROL" },                // Component for handling system messages (e.g. to ARM, takeoff, etc.).
}};
// [[[end]]] (checksum: 794ea890a87beb831d74eb2920000ea9)

std::string to_string(MAV_COMPONENT e)
{
	size_t idx = enum_value(e);
	auto it = mav_comp_id_strings.find(idx);

	if (it == mav_comp_id_strings.end())
		return std::to_string(idx);

	return it->second;
}

MAV_FRAME mav_frame_from_str(const std::string &mav_frame)
{
	for (size_t idx = 0; idx < mav_frame_strings.size(); idx++) {
		if (mav_frame_strings[idx] == mav_frame) {
			std::underlying_type<MAV_FRAME>::type rv = idx;
			return static_cast<MAV_FRAME>(rv);
		}
	}

	ROS_ERROR_STREAM_NAMED("uas", "FRAME: Unknown MAV_FRAME: " << mav_frame);
	return MAV_FRAME::LOCAL_NED;
}

MAV_TYPE mav_type_from_str(const std::string &mav_type)
{
	for (size_t idx = 0; idx < mav_type_names.size(); idx++) {
		if (mav_type_names[idx] == mav_type) {
			std::underlying_type<MAV_TYPE>::type rv = idx;
			return static_cast<MAV_TYPE>(rv);
		}
	}
	ROS_ERROR_STREAM_NAMED("uas", "TYPE: Unknown MAV_TYPE: " << mav_type);
	return MAV_TYPE::GENERIC;
}

// [[[cog:
// ename = 'MAV_DISTANCE_SENSOR'
// enum_name_is_value_outl(ename)
// ]]]
//! MAV_DISTANCE_SENSOR values
static const std::array<const std::string, 5> mav_distance_sensor_strings{{
/*  0 */ "LASER",                         // Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
/*  1 */ "ULTRASOUND",                    // Ultrasound rangefinder, e.g. MaxBotix units
/*  2 */ "INFRARED",                      // Infrared rangefinder, e.g. Sharp units
/*  3 */ "RADAR",                         // Radar type, e.g. uLanding units
/*  4 */ "UNKNOWN",                       // Broken or unknown type, e.g. analog units
}};

std::string to_string(MAV_DISTANCE_SENSOR e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_distance_sensor_strings.size())
		return std::to_string(idx);

	return mav_distance_sensor_strings[idx];
}
// [[[end]]] (checksum: 3f792ad01cdb3f2315a8907f578ab5b3)

// [[[cog:
// ename = 'LANDING_TARGET_TYPE'
// enum_name_is_value_outl(ename)
// ]]]
//! LANDING_TARGET_TYPE values
static const std::array<const std::string, 4> landing_target_type_strings{{
/*  0 */ "LIGHT_BEACON",                  // Landing target signaled by light beacon (ex: IR-LOCK)
/*  1 */ "RADIO_BEACON",                  // Landing target signaled by radio beacon (ex: ILS, NDB)
/*  2 */ "VISION_FIDUCIAL",               // Landing target represented by a fiducial marker (ex: ARTag)
/*  3 */ "VISION_OTHER",                  // Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
}};

std::string to_string(LANDING_TARGET_TYPE e)
{
	size_t idx = enum_value(e);
	if (idx >= landing_target_type_strings.size())
		return std::to_string(idx);

	return landing_target_type_strings[idx];
}
// [[[end]]] (checksum: a42789c10cbebd5bc253abca2a07289b)

LANDING_TARGET_TYPE landing_target_type_from_str(const std::string &landing_target_type)
{
	for (size_t idx = 0; idx < landing_target_type_strings.size(); idx++) {
		if (landing_target_type_strings[idx] == landing_target_type) {
			std::underlying_type<LANDING_TARGET_TYPE>::type rv = idx;
			return static_cast<LANDING_TARGET_TYPE>(rv);
		}
	}
	ROS_ERROR_STREAM_NAMED("uas", "TYPE: Unknown LANDING_TARGET_TYPE: " << landing_target_type << ". Defaulting to LIGHT_BEACON");
	return LANDING_TARGET_TYPE::LIGHT_BEACON;
}

}	// namespace utils
}	// namespace mavros
