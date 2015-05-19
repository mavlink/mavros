/**
 * @brief Sensor orientation helper function
 * @file sensor_orientation.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 20.014 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <array>
#include <mavros/utils.h>

using namespace mavutils;

const std::array<std::array<double, 3>, 39> sensor_orientation = {{
									  /* 0 MAV_SENSOR_ROTATION_NONE*/ {0.0,    0.0,    0.0},
									  /* 1 MAV_SENSOR_ROTATION_YAW_45*/ {0.0,    0.0,    45.0},
									  /* 2 MAV_SENSOR_ROTATION_YAW_90*/ {0.0,    0.0,    90.0},
									  /* 3 MAV_SENSOR_ROTATION_YAW_135*/ {0.0,    0.0,    135.0},
									  /* 4 MAV_SENSOR_ROTATION_YAW_180*/ {0.0,    0.0,    180.0},
									  /* 5 MAV_SENSOR_ROTATION_YAW_225*/ {0.0,    0.0,    225.0},
									  /* 6 MAV_SENSOR_ROTATION_YAW_270*/ {0.0,    0.0,    270.0},
									  /* 7 MAV_SENSOR_ROTATION_YAW_315*/ {0.0,    0.0,    315.0},
									  /* 8 MAV_SENSOR_ROTATION_ROLL_180*/ {180.0, 0.0,     0.0},
									  /* 9 MAV_SENSOR_ROTATION_ROLL_180_YAW_45*/ {180.0, 0.0,     45.0},
									  /* 10 MAV_SENSOR_ROTATION_ROLL_180_YAW_90*/ {180.0, 0.0,     90.0},
									  /* 11 MAV_SENSOR_ROTATION_ROLL_180_YAW_135*/ {180.0, 0.0,     135.0},
									  /* 12 MAV_SENSOR_ROTATION_PITCH_180*/	{0.0,    180.0,  0.0},
									  /* 13 MAV_SENSOR_ROTATION_ROLL_180_YAW_225*/ {180.0, 0.0,     225.0},
									  /* 14 MAV_SENSOR_ROTATION_ROLL_180_YAW_270*/ {180.0, 0.0,     270.0},
									  /* 15 MAV_SENSOR_ROTATION_ROLL_180_YAW_315*/ {180.0, 0.0,     315.0},
									  /* 16 MAV_SENSOR_ROTATION_ROLL_90*/ {90.0,   0.0,    0.0},
									  /* 17 MAV_SENSOR_ROTATION_ROLL_90_YAW_45*/ {90.0,   0.0,    45.0},
									  /* 18 MAV_SENSOR_ROTATION_ROLL_90_YAW_90*/ {90.0,   0.0,    90.0},
									  /* 19 MAV_SENSOR_ROTATION_ROLL_90_YAW_135*/ {90.0,   0.0,    135.0},
									  /* 20 MAV_SENSOR_ROTATION_ROLL_270*/ {270.0, 0.0,     0.0},
									  /* 21 MAV_SENSOR_ROTATION_ROLL_270_YAW_45*/ {270.0, 0.0,     45.0},
									  /* 22 MAV_SENSOR_ROTATION_ROLL_270_YAW_90*/ {270.0, 0.0,     90.0},
									  /* 23 MAV_SENSOR_ROTATION_ROLL_270_YAW_135*/ {270.0, 0.0,     135.0},
									  /* 24 MAV_SENSOR_ROTATION_PITCH_90*/ {0.0,    90.0,   0.0},
									  /* 25 MAV_SENSOR_ROTATION_PITCH_270*/	{0.0,    270.0,  0.0},
									  /* 26 MAV_SENSOR_ROTATION_PITCH_180_YAW_90*/ {0.0,    180.0,  90.0},
									  /* 27 MAV_SENSOR_ROTATION_PITCH_180_YAW_270*/	{0.0,    180.0,  270.0},
									  /* 28 MAV_SENSOR_ROTATION_ROLL_90_PITCH_90*/ {90.0,   90.0,   0.0},
									  /* 29 MAV_SENSOR_ROTATION_ROLL_180_PITCH_90*/	{180.0, 90.0,    0.0},
									  /* 30 MAV_SENSOR_ROTATION_ROLL_270_PITCH_90*/	{270.0, 90.0,    0.0},
									  /* 31 MAV_SENSOR_ROTATION_ROLL_90_PITCH_180*/	{90.0,   180.0,  0.0},
									  /* 32 MAV_SENSOR_ROTATION_ROLL_270_PITCH_180*/ {270.0, 180.0,   0.0},
									  /* 33 MAV_SENSOR_ROTATION_ROLL_90_PITCH_270*/	{90.0,   270.0,  0.0},
									  /* 34 MAV_SENSOR_ROTATION_ROLL_180_PITCH_270*/ {180.0, 270.0,   0.0},
									  /* 35 MAV_SENSOR_ROTATION_ROLL_270_PITCH_270*/ {270.0, 270.0,   0.0},
									  /* 36 MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90*/ {90.0,   180.0,  90.0},
									  /* 37 MAV_SENSOR_ROTATION_ROLL_90_YAW_270*/ {90.0,   0.0,    270.0},
									  /* 38 MAV_SENSOR_ROTATION_MAX*/ {315.0, 315.0,   315.0}
								  }};

std::array<double, 3> orientation_matching(uint8_t orientation){
	return sensor_orientation[orientation];
};