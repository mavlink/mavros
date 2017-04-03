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
static const std::array<const std::string, 18> mav_autopilot_strings{{
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
}};

std::string to_string(MAV_AUTOPILOT e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_autopilot_strings.size())
		return std::to_string(idx);

	return mav_autopilot_strings[idx];
}
// [[[end]]] (checksum: c0f450ce84a31ce0f86d439c007cf805)

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
static const std::array<const std::string, 28> mav_type_strings{{
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
}};

std::string to_string(MAV_TYPE e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_type_strings.size())
		return std::to_string(idx);

	return mav_type_strings[idx];
}
// [[[end]]] (checksum: ff3fd0c445310aef4a3cfb14a18178e0)

// [[[cog:
// ename = 'MAV_STATE'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = e.name[10:].title()
//     sp = make_whitespace(30, value)
//     cog.outl("""/* {k:>2} */ "{value}",{sp}// {e.description}""".format(**locals()))
//
// cog.outl("}};")
// cog.outl()
// to_string_outl(ename)
// ]]]
//! MAV_STATE values
static const std::array<const std::string, 8> mav_state_strings{{
/*  0 */ "Uninit",                        // Uninitialized system, state is unknown.
/*  1 */ "Boot",                          // System is booting up.
/*  2 */ "Calibrating",                   // System is calibrating and not flight-ready.
/*  3 */ "Standby",                       // System is grounded and on standby. It can be launched any time.
/*  4 */ "Active",                        // System is active and might be already airborne. Motors are engaged.
/*  5 */ "Critical",                      // System is in a non-normal flight mode. It can however still navigate.
/*  6 */ "Emergency",                     // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
/*  7 */ "Poweroff",                      // System just initialized its power-down sequence, will shut down now.
}};

std::string to_string(MAV_STATE e)
{
	size_t idx = enum_value(e);
	if (idx >= mav_state_strings.size())
		return std::to_string(idx);

	return mav_state_strings[idx];
}
// [[[end]]] (checksum: 47dea7c5bd6ab53dbc75a6c51b35d312)


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

}	// namespace utils
}	// namespace mavros
