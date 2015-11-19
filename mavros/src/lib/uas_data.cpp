/**
 * @brief MAVROS UAS manager (data part)
 * @file uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov.
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
	tf2_listener(tf2_buffer, true),
	type(MAV_TYPE_GENERIC),
	autopilot(MAV_AUTOPILOT_GENERIC),
	base_mode(0),
	target_system(1),
	target_component(1),
	connected(false),
	gps_eph(NAN),
	gps_epv(NAN),
	gps_fix_type(0),
	gps_satellites_visible(0),
	time_offset(0),
	fcu_caps_known(false),
	fcu_capabilities(0)
{}

void UAS::stop(void)
{}


/* -*- heartbeat handlers -*- */

void UAS::update_heartbeat(uint8_t type_, uint8_t autopilot_, uint8_t base_mode_)
{
	type = type_;
	autopilot = autopilot_;
	base_mode = base_mode_;
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

void UAS::update_attitude_imu(sensor_msgs::Imu::Ptr &imu)
{
	lock_guard lock(mutex);
	imu_data = imu;
}

sensor_msgs::Imu::Ptr UAS::get_attitude_imu()
{
	lock_guard lock(mutex);
	return imu_data;
}

geometry_msgs::Quaternion UAS::get_attitude_orientation()
{
	lock_guard lock(mutex);
	if (imu_data)
		return imu_data->orientation;
	else {
		// fallback - return identity
		geometry_msgs::Quaternion q;
		q.w = 1.0; q.x = q.y = q.z = 0.0;
		return q;
	}
}

geometry_msgs::Vector3 UAS::get_attitude_angular_velocity()
{
	lock_guard lock(mutex);
	if (imu_data)
		return imu_data->angular_velocity;
	else {
		// fallback
		geometry_msgs::Vector3 v;
		v.x = v.y = v.z = 0.0;
		return v;
	}
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

