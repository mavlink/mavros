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

namespace mavros {
namespace utils {
using mavlink::common::MAV_AUTOPILOT;
using mavlink::common::MAV_TYPE;
using mavlink::common::MAV_STATE;


// [[[cog:
// import pymavlink.dialects.v20.common as common
// ename = 'MAV_AUTOPILOT'
//
// def get_enum(ename):
//     enum = common.enums[ename].items()
//     enum.sort()
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
// def array_outl(name, enum):
//     cog.outl("//! %s values" % name)
//     cog.outl("static const std::array<const std::string, %s> %s_strings{{" % (len(enum), name.lower()))
//
// enum = get_enum(ename)
// array_outl(ename, enum)
// for k, e in enum:
//     value = split_by(',-/.', e.description)
//     cog.outl("""/* {k:>2} */ "{value}",""".format(**locals()))
//
// cog.outl("}};")
// ]]]
//! MAV_AUTOPILOT values
static const std::array<const std::string, 18> mav_autopilot_strings{{
/*  0 */ "Generic autopilot",
/*  1 */ "Reserved for future use",
/*  2 */ "SLUGS autopilot",
/*  3 */ "ArduPilotMega / ArduCopter",
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
}};
// [[[end]]] (checksum: f81240054dda4883ce7ef1ea7509d079)

std::string to_string(MAV_AUTOPILOT ap)
{
	size_t idx = enum_value(ap);
	if (idx >= mav_autopilot_strings.size())
		return std::to_string(idx);

	return mav_autopilot_strings[idx];
}

// [[[cog:
// ename = 'MAV_TYPE'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = split_by(',-/.', e.description)
//     cog.outl("""/* {k:>2} */ "{value}",""".format(**locals()))
//
// cog.outl("}};")
// ]]]
//! MAV_TYPE values
static const std::array<const std::string, 28> mav_type_strings{{
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
/* 15 */ "Octorotor",
/* 16 */ "Flapping wing",
/* 17 */ "Flapping wing",
/* 18 */ "Onboard companion controller",
/* 19 */ "Two",
/* 20 */ "Quad",
/* 21 */ "Tiltrotor VTOL",
/* 22 */ "VTOL reserved 2",
/* 23 */ "VTOL reserved 3",
/* 24 */ "VTOL reserved 4",
/* 25 */ "VTOL reserved 5",
/* 26 */ "Onboard gimbal",
/* 27 */ "Onboard ADSB peripheral",
}};
// [[[end]]] (checksum: 1b8a0a4bdffb6b1d10fce6e854a19acd)

std::string to_string(MAV_TYPE type)
{
	size_t idx = enum_value(type);
	if (idx >= mav_type_strings.size())
		return std::to_string(idx);

	return mav_type_strings[idx];
}

// [[[cog:
// ename = 'MAV_STATE'
// enum = get_enum(ename)
//
// array_outl(ename, enum)
// for k, e in enum:
//     value = e.name[10:].title()
//     cog.outl("""/* {k:>2} */ "{value}",""".format(**locals()))
//
// cog.outl("}};")
// ]]]
//! MAV_STATE values
static const std::array<const std::string, 8> mav_state_strings{{
/*  0 */ "Uninit",
/*  1 */ "Boot",
/*  2 */ "Calibrating",
/*  3 */ "Standby",
/*  4 */ "Active",
/*  5 */ "Critical",
/*  6 */ "Emergency",
/*  7 */ "Poweroff",
}};
// [[[end]]] (checksum: 263270d5d28ab2a8b5866e6a111934ef)

std::string to_string(MAV_STATE st)
{
	size_t idx = enum_value(st);
	if (idx >= mav_state_strings.size())
		return std::to_string(idx);

	return mav_state_strings[idx];
}

}	// namespace utils
}	// namespace mavros
