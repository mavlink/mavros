/**
 * @brief MAVROS UAS manager (data part)
 * @file uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <array>
#include <unordered_map>
#include <stdexcept>
#include <mavros/mavros_uas.h>
#include <mavros/utils.h>
#include <mavros/px4_custom_mode.h>

using namespace mavros;

UAS::UAS() :
	type(MAV_TYPE_GENERIC),
	autopilot(MAV_AUTOPILOT_GENERIC),
	target_system(1),
	target_component(1),
	connected(false),
	imu_orientation(),
	imu_angular_velocity(),
	imu_linear_acceleration(),
	gps_eph(NAN),
	gps_epv(NAN),
	gps_fix_type(0),
	gps_satellites_visible(0),
	fcu_caps_known(false),
	fcu_capabilities(0)
{}

void UAS::stop(void)
{}


/* -*- heartbeat handlers -*- */

void UAS::update_heartbeat(uint8_t type_, uint8_t autopilot_)
{
	type = type_;
	autopilot = autopilot_;
}

void UAS::update_connection_status(bool conn_)
{
	if (conn_ != connected) {
		connected = conn_;
		sig_connection_changed(connected);
	}
}


/* -*- autopilot version -*- */

static uint64_t get_default_caps(uint8_t ap_type)
{
	// TODO: return default caps mask for known FCU's
	return 0;
}

uint64_t UAS::get_capabilities()
{
	if (fcu_caps_known) {
		uint64_t caps = fcu_capabilities;
		return caps;
	}
	else {
		return get_default_caps(get_autopilot());
	}
}

void UAS::update_capabilities(bool known, uint64_t caps)
{
	fcu_caps_known = known;
	fcu_capabilities = caps;
}


/* -*- IMU data -*- */

void UAS::update_attitude_imu(tf::Quaternion &q, tf::Vector3 &av, tf::Vector3 &lacc)
{
	lock_guard lock(mutex);

	imu_orientation = q;
	imu_angular_velocity = av;
	imu_linear_acceleration = lacc;
}

tf::Vector3 UAS::get_attitude_angular_velocity()
{
	lock_guard lock(mutex);
	return imu_angular_velocity;
}

tf::Vector3 UAS::get_attitude_linear_acceleration()
{
	lock_guard lock(mutex);
	return imu_linear_acceleration;
}

tf::Quaternion UAS::get_attitude_orientation()
{
	lock_guard lock(mutex);
	return imu_orientation;
}


/* -*- GPS data -*- */

void UAS::update_gps_fix_epts(sensor_msgs::NavSatFix::Ptr &fix,
		float eph, float epv,
		int fix_type, int satellites_visible)
{
	lock_guard lock(mutex);

	gps_fix = fix;
	gps_eph = eph;
	gps_epv = epv;
	gps_fix_type = fix_type;
	gps_satellites_visible = satellites_visible;
}

//! Returns EPH, EPV, Fix type and satellites visible
void UAS::get_gps_epts(float &eph, float &epv, int &fix_type, int &satellites_visible)
{
	lock_guard lock(mutex);

	eph = gps_eph;
	epv = gps_epv;
	fix_type = gps_fix_type;
	satellites_visible = gps_satellites_visible;
}

//! Retunrs last GPS RAW message
sensor_msgs::NavSatFix::Ptr UAS::get_gps_fix()
{
	lock_guard lock(mutex);
	return gps_fix;
}

