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
#include <mavros/utils.h>
#include <ros/console.h>

namespace mavros {
namespace utils {
using mavlink::common::MAV_AUTOPILOT;
using mavlink::common::MAV_TYPE;
using mavlink::common::MAV_STATE;
using mavlink::common::MAV_ESTIMATOR_TYPE;
using mavlink::common::ADSB_ALTITUDE_TYPE;
using mavlink::common::ADSB_EMITTER_TYPE;
using mavlink::common::GPS_FIX_TYPE;
using mavlink::common::MAV_MISSION_RESULT;
using mavlink::common::MAV_FRAME;
using mavlink::common::MAV_DISTANCE_SENSOR;

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
// def ename_array_name(ename):
//     l = ename.rsplit('::', 1)
//     return (l[1] if len(l) > 1 else l[0]).lower() + '_strings'
//
// def array_outl(name, enum):
//     array = ename_array_name(name)
//     cog.outl("//! %s values" % name)
//     cog.outl("static const std::array<const std::string, %s> %s{{" % (len(enum), array))
//
// def to_string_outl(ename):
//     array = ename_array_name(ename)
//     cog.outl("std::string to_string({ename} e)".format(**locals()))
//     cog.outl("{")
//     cog.outl("	size_t idx = enum_value(e);")
//     cog.outl("	if (idx >= {array}.size())".format(**locals()))
//     cog.outl("		return std::to_string(idx);")
//     cog.outl()
//     cog.outl("	return {array}[idx];".format(**locals()))
//     cog.outl("}")
//
// ename = 'MAV_AUTOPILOT'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = split_by(',-/.', e.description)
//     sp = make_whitespace(30, value)
//     cog.outl("""/* {k:>2} */ "{value}",{sp}// {e.description}""".format(**locals()))
//
// cog.outl("}};")
// cog.outl()
// to_string_outl(ename)
// ]]]
//! MAV_AUTOPILOT values
static const std::array<const std::string, 19> mav_autopilot_strings{{
/*  0 */ "Generic autopilot",             // Generic autopilot, full support for everything
/*  1 */ "Reserved for future use",       // Reserved for future use.
/*  2 */ "SLUGS autopilot",               // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
/*  3 */ "ArduPilotMega / ArduCopter",    // ArduPilotMega / ArduCopter, http://diydrones.com
/*  4 */ "OpenPilot",                     // OpenPilot, http://openpilot.org
/*  5 */ "Generic autopilot only supporting simple waypoints", // Generic autopilot only supporting simple waypoints
/*  6 */ "Generic autopilot supporting waypoints and other simple navigation commands", // Generic autopilot supporting waypoints and other simple navigation commands
/*  7 */ "Generic autopilot supporting the full mission command set", // Generic autopilot supporting the full mission command set
/*  8 */ "No valid autopilot",            // No valid autopilot, e.g. a GCS or other MAVLink component
/*  9 */ "PPZ UAV",                       // PPZ UAV - http://nongnu.org/paparazzi
/* 10 */ "UAV Dev Board",                 // UAV Dev Board
/* 11 */ "FlexiPilot",                    // FlexiPilot
/* 12 */ "PX4 Autopilot",                 // PX4 Autopilot - http://pixhawk.ethz.ch/px4/
/* 13 */ "SMACCMPilot",                   // SMACCMPilot - http://smaccmpilot.org
/* 14 */ "AutoQuad",                      // AutoQuad -- http://autoquad.org
/* 15 */ "Armazila",                      // Armazila -- http://armazila.com
/* 16 */ "Aerob",                         // Aerob -- http://aerob.ru
/* 17 */ "ASLUAV autopilot",              // ASLUAV autopilot -- http://www.asl.ethz.ch
/* 18 */ "SmartAP Autopilot",             // SmartAP Autopilot - http://sky-drones.com
}};

std::string to_string(MAV_AUTOPILOT e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_autopilot_strings.size())
		return std::to_string(idx);

	return mav_autopilot_strings[idx];
}
// [[[end]]] (checksum: 5b451ba6ab334d133765faaa33a18c6a)

// [[[cog:
// ename = 'MAV_TYPE'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = split_by(',-/.', e.description)
//     sp = make_whitespace(30, value)
//     cog.outl("""/* {k:>2} */ "{value}",{sp}// {e.description}""".format(**locals()))
//
// cog.outl("}};")
// cog.outl()
// to_string_outl(ename)
// ]]]
//! MAV_TYPE values
static const std::array<const std::string, 30> mav_type_strings{{
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
/* 26 */ "Onboard gimbal",                // Onboard gimbal
/* 27 */ "Onboard ADSB peripheral",       // Onboard ADSB peripheral
/* 28 */ "Steerable",                     // Steerable, nonrigid airfoil
/* 29 */ "Dodecarotor",                   // Dodecarotor
}};

std::string to_string(MAV_TYPE e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_type_strings.size())
		return std::to_string(idx);

	return mav_type_strings[idx];
}
// [[[end]]] (checksum: 3955611cab161e8b54cc33f1ea67c946)

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
// def enum_name_is_value_outl(ename):
//     enum = get_enum(ename)
//
//     array_outl(ename, enum)
//     for k, e in enum:
//         name_short =  e.name[len(ename) + 1:]
//         sp = make_whitespace(30, name_short)
//         if e.description:
//             cog.outl("""/* {k:>2} */ "{name_short}",{sp}// {e.description}""".format(**locals()))
//         else:
//             cog.outl("""/* {k:>2} */ "{name_short}",""".format(**locals()))
//
//     cog.outl("}};")
//     cog.outl()
//     to_string_outl(ename)
//
//
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
static const std::array<const std::string, 15> mav_mission_result_strings{{
/*  0 */ "mission accepted OK",           // mission accepted OK
/*  1 */ "generic error / not accepting mission commands at all right now", // generic error / not accepting mission commands at all right now
/*  2 */ "coordinate frame is not supported", // coordinate frame is not supported
/*  3 */ "command is not supported",      // command is not supported
/*  4 */ "mission item exceeds storage space", // mission item exceeds storage space
/*  5 */ "one of the parameters has an invalid value", // one of the parameters has an invalid value
/*  6 */ "param1 has an invalid value",   // param1 has an invalid value
/*  7 */ "param2 has an invalid value",   // param2 has an invalid value
/*  8 */ "param3 has an invalid value",   // param3 has an invalid value
/*  9 */ "param4 has an invalid value",   // param4 has an invalid value
/* 10 */ "x/param5 has an invalid value", // x/param5 has an invalid value
/* 11 */ "y/param6 has an invalid value", // y/param6 has an invalid value
/* 12 */ "param7 has an invalid value",   // param7 has an invalid value
/* 13 */ "received waypoint out of sequence", // received waypoint out of sequence
/* 14 */ "not accepting any mission commands from this communication partner", // not accepting any mission commands from this communication partner
}};

std::string to_string(MAV_MISSION_RESULT e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_mission_result_strings.size())
		return std::to_string(idx);

	return mav_mission_result_strings[idx];
}
// [[[end]]] (checksum: 06dac7af3755763d02332dea1ebf6a91)

// [[[cog:
// ename = 'MAV_FRAME'
// enum_name_is_value_outl(ename)
// ]]]
//! MAV_FRAME values
static const std::array<const std::string, 12> mav_frame_strings{{
/*  0 */ "GLOBAL",                        // Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
/*  1 */ "LOCAL_NED",                     // Local coordinate frame, Z-up (x: north, y: east, z: down).
/*  2 */ "MISSION",                       // NOT a coordinate frame, indicates a mission command.
/*  3 */ "GLOBAL_RELATIVE_ALT",           // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
/*  4 */ "LOCAL_ENU",                     // Local coordinate frame, Z-down (x: east, y: north, z: up)
/*  5 */ "GLOBAL_INT",                    // Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
/*  6 */ "GLOBAL_RELATIVE_ALT_INT",       // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
/*  7 */ "LOCAL_OFFSET_NED",              // Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
/*  8 */ "BODY_NED",                      // Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
/*  9 */ "BODY_OFFSET_NED",               // Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
/* 10 */ "GLOBAL_TERRAIN_ALT",            // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
/* 11 */ "GLOBAL_TERRAIN_ALT_INT",        // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
}};

std::string to_string(MAV_FRAME e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_frame_strings.size())
		return std::to_string(idx);

	return mav_frame_strings[idx];
}
// [[[end]]] (checksum: ffbf4a7aacdc4b5229293f3791f63379)

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

}	// namespace utils
}	// namespace mavros
