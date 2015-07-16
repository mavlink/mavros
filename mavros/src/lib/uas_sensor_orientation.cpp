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

using namespace mavros;

/** @todo Combine the bellow with string representation */

static const std::array<const Eigen::Vector3d, 39> sensor_orientation = {
	/*  0 */ Eigen::Vector3d(0.0,   0.0,    0.0),
	/*  1 */ Eigen::Vector3d(0.0,   0.0,    45.0),
	/*  2 */ Eigen::Vector3d(0.0,   0.0,    90.0),
	/*  3 */ Eigen::Vector3d(0.0,   0.0,    135.0),
	/*  4 */ Eigen::Vector3d(0.0,   0.0,    180.0),
	/*  5 */ Eigen::Vector3d(0.0,   0.0,    225.0),
	/*  6 */ Eigen::Vector3d(0.0,   0.0,    270.0),
	/*  7 */ Eigen::Vector3d(0.0,   0.0,    315.0),
	/*  8 */ Eigen::Vector3d(180.0, 0.0,    0.0),
	/*  9 */ Eigen::Vector3d(180.0, 0.0,    45.0),
	/* 10 */ Eigen::Vector3d(180.0, 0.0,    90.0),
	/* 11 */ Eigen::Vector3d(180.0, 0.0,    135.0),
	/* 12 */ Eigen::Vector3d(0.0,   180.0,  0.0),
	/* 13 */ Eigen::Vector3d(180.0, 0.0,    225.0),
	/* 14 */ Eigen::Vector3d(180.0, 0.0,    270.0),
	/* 15 */ Eigen::Vector3d(180.0, 0.0,    315.0),
	/* 16 */ Eigen::Vector3d(90.0,  0.0,    0.0),
	/* 17 */ Eigen::Vector3d(90.0,  0.0,    45.0),
	/* 18 */ Eigen::Vector3d(90.0,  0.0,    90.0),
	/* 19 */ Eigen::Vector3d(90.0,  0.0,    135.0),
	/* 20 */ Eigen::Vector3d(270.0, 0.0,    0.0),
	/* 21 */ Eigen::Vector3d(270.0, 0.0,    45.0),
	/* 22 */ Eigen::Vector3d(270.0, 0.0,    90.0),
	/* 23 */ Eigen::Vector3d(270.0, 0.0,    135.0),
	/* 24 */ Eigen::Vector3d(0.0,   90.0,   0.0),
	/* 25 */ Eigen::Vector3d(0.0,   270.0,  0.0),
	/* 26 */ Eigen::Vector3d(0.0,   180.0,  90.0),
	/* 27 */ Eigen::Vector3d(0.0,   180.0,  270.0),
	/* 28 */ Eigen::Vector3d(90.0,  90.0,   0.0),
	/* 29 */ Eigen::Vector3d(180.0, 90.0,   0.0),
	/* 30 */ Eigen::Vector3d(270.0, 90.0,   0.0),
	/* 31 */ Eigen::Vector3d(90.0,  180.0,  0.0),
	/* 32 */ Eigen::Vector3d(270.0, 180.0,  0.0),
	/* 33 */ Eigen::Vector3d(90.0,  270.0,  0.0),
	/* 34 */ Eigen::Vector3d(180.0, 270.0,  0.0),
	/* 35 */ Eigen::Vector3d(270.0, 270.0,  0.0),
	/* 36 */ Eigen::Vector3d(90.0,  180.0,  90.0),
	/* 37 */ Eigen::Vector3d(90.0,  0.0,    270.0),
	/* 38 */ Eigen::Vector3d(315.0, 315.0,  315.0)
};

Eigen::Vector3d UAS::sensor_orientation_matching(MAV_SENSOR_ORIENTATION orientation)
{
	size_t idx = size_t(orientation);
	if (idx >= sensor_orientation.size()) {
		ROS_WARN_NAMED("uas", "SENSOR: wrong orintation index: %zu", idx);
		return Eigen::Vector3d();
	}

	return Eigen::Vector3d(angles::from_degrees(sensor_orientation[idx].x()),
						angles::from_degrees(sensor_orientation[idx].y()),
						angles::from_degrees(sensor_orientation[idx].z()));
};
