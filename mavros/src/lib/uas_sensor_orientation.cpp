/**
 * @brief Sensor orientation helper function
 * @file sensor_orientation.cpp
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
#include <mavros/mavros_uas.h>

using namespace mavros;

// XXX do we need combine that with string representation?

static const std::array<const tf::Vector3, 39> sensor_orientation = {
	/*  0 */ tf::Vector3(0.0,   0.0,    0.0),
	/*  1 */ tf::Vector3(0.0,   0.0,    45.0),
	/*  2 */ tf::Vector3(0.0,   0.0,    90.0),
	/*  3 */ tf::Vector3(0.0,   0.0,    135.0),
	/*  4 */ tf::Vector3(0.0,   0.0,    180.0),
	/*  5 */ tf::Vector3(0.0,   0.0,    225.0),
	/*  6 */ tf::Vector3(0.0,   0.0,    270.0),
	/*  7 */ tf::Vector3(0.0,   0.0,    315.0),
	/*  8 */ tf::Vector3(180.0, 0.0,    0.0),
	/*  9 */ tf::Vector3(180.0, 0.0,    45.0),
	/* 10 */ tf::Vector3(180.0, 0.0,    90.0),
	/* 11 */ tf::Vector3(180.0, 0.0,    135.0),
	/* 12 */ tf::Vector3(0.0,   180.0,  0.0),
	/* 13 */ tf::Vector3(180.0, 0.0,    225.0),
	/* 14 */ tf::Vector3(180.0, 0.0,    270.0),
	/* 15 */ tf::Vector3(180.0, 0.0,    315.0),
	/* 16 */ tf::Vector3(90.0,  0.0,    0.0),
	/* 17 */ tf::Vector3(90.0,  0.0,    45.0),
	/* 18 */ tf::Vector3(90.0,  0.0,    90.0),
	/* 19 */ tf::Vector3(90.0,  0.0,    135.0),
	/* 20 */ tf::Vector3(270.0, 0.0,    0.0),
	/* 21 */ tf::Vector3(270.0, 0.0,    45.0),
	/* 22 */ tf::Vector3(270.0, 0.0,    90.0),
	/* 23 */ tf::Vector3(270.0, 0.0,    135.0),
	/* 24 */ tf::Vector3(0.0,   90.0,   0.0),
	/* 25 */ tf::Vector3(0.0,   270.0,  0.0),
	/* 26 */ tf::Vector3(0.0,   180.0,  90.0),
	/* 27 */ tf::Vector3(0.0,   180.0,  270.0),
	/* 28 */ tf::Vector3(90.0,  90.0,   0.0),
	/* 29 */ tf::Vector3(180.0, 90.0,   0.0),
	/* 30 */ tf::Vector3(270.0, 90.0,   0.0),
	/* 31 */ tf::Vector3(90.0,  180.0,  0.0),
	/* 32 */ tf::Vector3(270.0, 180.0,  0.0),
	/* 33 */ tf::Vector3(90.0,  270.0,  0.0),
	/* 34 */ tf::Vector3(180.0, 270.0,  0.0),
	/* 35 */ tf::Vector3(270.0, 270.0,  0.0),
	/* 36 */ tf::Vector3(90.0,  180.0,  90.0),
	/* 37 */ tf::Vector3(90.0,  0.0,    270.0),
	/* 38 */ tf::Vector3(315.0, 315.0,  315.0)
};

tf::Vector3 UAS::sensor_orientation_matching(MAV_SENSOR_ORIENTATION orientation)
{
	// XXX should it return radians?

	size_t idx = size_t(orientation);
	if (idx >= sensor_orientation.size()) {
		ROS_WARN_NAMED("uas", "SENSOR: wrong orintation index: %zu", idx);
		return tf::Vector3();
	}

	return sensor_orientation[idx];
};
