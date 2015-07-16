/**
 * @brief Sensor orientation helper function
 * @file uas_sensor_orientation.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <array>
#include <angles/angles.h>
#include <mavros/mavros_uas.h>

#define DEG_TO_RAD (M_PI / 180.0f)

using namespace mavros;

// internal type: name - rotation
typedef std::pair<const std::string, const Eigen::Quaterniond> OrientationPair;

// internal data initializer
static const OrientationPair make_orientation(const std::string &name,
		const double roll,
		const double pitch,
		const double yaw)
{
	const Eigen::Quaterniond rot = UAS::quaternion_from_rpy(Eigen::Vector3d(roll, pitch, yaw) * DEG_TO_RAD);
	return std::make_pair(name, rot);
}

/** @todo Combine the bellow with string representation */

// Note: usage of double { makes YouCompleteMe happy
static const std::array<const OrientationPair, 39> sensor_orientations = {{
	/*  0 */ make_orientation("", 0.0,   0.0,    0.0),
	/*  1 */ make_orientation("", 0.0,   0.0,    45.0),
	/*  2 */ make_orientation("", 0.0,   0.0,    90.0),
	/*  3 */ make_orientation("", 0.0,   0.0,    135.0),
	/*  4 */ make_orientation("", 0.0,   0.0,    180.0),
	/*  5 */ make_orientation("", 0.0,   0.0,    225.0),
	/*  6 */ make_orientation("", 0.0,   0.0,    270.0),
	/*  7 */ make_orientation("", 0.0,   0.0,    315.0),
	/*  8 */ make_orientation("", 180.0, 0.0,    0.0),
	/*  9 */ make_orientation("", 180.0, 0.0,    45.0),
	/* 10 */ make_orientation("", 180.0, 0.0,    90.0),
	/* 11 */ make_orientation("", 180.0, 0.0,    135.0),
	/* 12 */ make_orientation("", 0.0,   180.0,  0.0),
	/* 13 */ make_orientation("", 180.0, 0.0,    225.0),
	/* 14 */ make_orientation("", 180.0, 0.0,    270.0),
	/* 15 */ make_orientation("", 180.0, 0.0,    315.0),
	/* 16 */ make_orientation("", 90.0,  0.0,    0.0),
	/* 17 */ make_orientation("", 90.0,  0.0,    45.0),
	/* 18 */ make_orientation("", 90.0,  0.0,    90.0),
	/* 19 */ make_orientation("", 90.0,  0.0,    135.0),
	/* 20 */ make_orientation("", 270.0, 0.0,    0.0),
	/* 21 */ make_orientation("", 270.0, 0.0,    45.0),
	/* 22 */ make_orientation("", 270.0, 0.0,    90.0),
	/* 23 */ make_orientation("", 270.0, 0.0,    135.0),
	/* 24 */ make_orientation("", 0.0,   90.0,   0.0),
	/* 25 */ make_orientation("", 0.0,   270.0,  0.0),
	/* 26 */ make_orientation("", 0.0,   180.0,  90.0),
	/* 27 */ make_orientation("", 0.0,   180.0,  270.0),
	/* 28 */ make_orientation("", 90.0,  90.0,   0.0),
	/* 29 */ make_orientation("", 180.0, 90.0,   0.0),
	/* 30 */ make_orientation("", 270.0, 90.0,   0.0),
	/* 31 */ make_orientation("", 90.0,  180.0,  0.0),
	/* 32 */ make_orientation("", 270.0, 180.0,  0.0),
	/* 33 */ make_orientation("", 90.0,  270.0,  0.0),
	/* 34 */ make_orientation("", 180.0, 270.0,  0.0),
	/* 35 */ make_orientation("", 270.0, 270.0,  0.0),
	/* 36 */ make_orientation("", 90.0,  180.0,  90.0),
	/* 37 */ make_orientation("", 90.0,  0.0,    270.0),
	/* 38 */ make_orientation("", 315.0, 315.0,  315.0)
}};

Eigen::Quaterniond UAS::sensor_orientation_matching(MAV_SENSOR_ORIENTATION orientation)
{
	size_t idx = size_t(orientation);
	if (idx >= sensor_orientations.size()) {
		ROS_WARN_NAMED("uas", "SENSOR: wrong orintation index: %zu", idx);
		return Eigen::Quaterniond::Identity();
	}

	return sensor_orientations[idx].second;
}
