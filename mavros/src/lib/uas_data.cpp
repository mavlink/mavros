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
using utils::enum_value;

UAS::UAS() :
	tf2_listener(tf2_buffer, true),
	type(enum_value(MAV_TYPE::GENERIC)),
	autopilot(enum_value(MAV_AUTOPILOT::GENERIC)),
	base_mode(0),
	target_system(1),
	target_component(1),
	connected(false),
	gps_eph(NAN),
	gps_epv(NAN),
	gps_fix_type(0),
	gps_satellites_visible(0),
	time_offset(0),
	tsync_mode(UAS::timesync_mode::NONE),
	fcu_caps_known(false),
	fcu_capabilities(0)
{
	try {
		// Using smallest dataset with 5' grid,
		// From default location,
		// Use cubic interpolation, Thread safe
		egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);
	}
	catch (const std::exception &e) {
		// catch exception and shutdown node
		ROS_FATAL_STREAM("UAS: GeographicLib exception: " << e.what() <<
				" | Run install_geographiclib_dataset.sh script in order to install Geoid Model dataset!");
		ros::shutdown();
	}

	// Publish helper TFs used for frame transformation in the odometry plugin
	std::vector<geometry_msgs::TransformStamped> transform_vector;
	add_static_transform("map", "map_ned", Eigen::Affine3d(ftf::quaternion_from_rpy(M_PI, 0, M_PI_2)),transform_vector);
	add_static_transform("odom", "odom_ned", Eigen::Affine3d(ftf::quaternion_from_rpy(M_PI, 0, M_PI_2)),transform_vector);
	add_static_transform("base_link", "base_link_frd", Eigen::Affine3d(ftf::quaternion_from_rpy(M_PI, 0, 0)),transform_vector);

	tf2_static_broadcaster.sendTransform(transform_vector);
}

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

		// call all change cb's
		for (auto &cb : connection_cb_vec)
			cb(conn_);
	}
}

void UAS::add_connection_change_handler(UAS::ConnectionCb cb)
{
	lock_guard lock(mutex);
	connection_cb_vec.push_back(cb);
}

/* -*- autopilot version -*- */

static uint64_t get_default_caps(UAS::MAV_AUTOPILOT ap_type)
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

// This function may need a mutex now
void UAS::update_capabilities(bool known, uint64_t caps)
{
	bool process_cb_queue = false;

	if (known != fcu_caps_known) {
		if (!fcu_caps_known) {
			process_cb_queue = true;
		}
		fcu_caps_known = known;
	} else if (fcu_caps_known) {	// Implies fcu_caps_known == known
		if (caps != fcu_capabilities) {
			process_cb_queue = true;
		}
	}
	else {}	// Capabilities werent known before and arent known after update

	if (process_cb_queue) {
		fcu_capabilities = caps;
		for (auto &cb : capabilities_cb_vec) {
			cb(static_cast<MAV_CAP>(caps));
		}
	}
}

void UAS::add_capabilities_change_handler(UAS::CapabilitiesCb cb)
{
	lock_guard lock(mutex);
	capabilities_cb_vec.push_back(cb);
}

/* -*- IMU data -*- */

void UAS::update_attitude_imu_enu(sensor_msgs::Imu::Ptr &imu)
{
	lock_guard lock(mutex);
	imu_enu_data = imu;
}

void UAS::update_attitude_imu_ned(sensor_msgs::Imu::Ptr &imu)
{
	lock_guard lock(mutex);
	imu_ned_data = imu;
}

sensor_msgs::Imu::Ptr UAS::get_attitude_imu_enu()
{
	lock_guard lock(mutex);
	return imu_enu_data;
}

sensor_msgs::Imu::Ptr UAS::get_attitude_imu_ned()
{
	lock_guard lock(mutex);
	return imu_ned_data;
}

geometry_msgs::Quaternion UAS::get_attitude_orientation_enu()
{
	lock_guard lock(mutex);
	if (imu_enu_data)
		return imu_enu_data->orientation;
	else {
		// fallback - return identity
		geometry_msgs::Quaternion q;
		q.w = 1.0; q.x = q.y = q.z = 0.0;
		return q;
	}
}

geometry_msgs::Quaternion UAS::get_attitude_orientation_ned()
{
	lock_guard lock(mutex);
	if (imu_ned_data)
		return imu_ned_data->orientation;
	else {
		// fallback - return identity
		geometry_msgs::Quaternion q;
		q.w = 1.0; q.x = q.y = q.z = 0.0;
		return q;
	}
}

geometry_msgs::Vector3 UAS::get_attitude_angular_velocity_enu()
{
	lock_guard lock(mutex);
	if (imu_enu_data)
		return imu_enu_data->angular_velocity;
	else {
		// fallback
		geometry_msgs::Vector3 v;
		v.x = v.y = v.z = 0.0;
		return v;
	}
}

geometry_msgs::Vector3 UAS::get_attitude_angular_velocity_ned()
{
	lock_guard lock(mutex);
	if (imu_ned_data)
		return imu_ned_data->angular_velocity;
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

/* -*- transform -*- */

//! Stack static transform into vector
void UAS::add_static_transform(const std::string &frame_id, const std::string &child_id, const Eigen::Affine3d &tr, std::vector<geometry_msgs::TransformStamped>& vector)
{
	geometry_msgs::TransformStamped static_transform;

	static_transform.header.stamp = ros::Time::now();
	static_transform.header.frame_id = frame_id;
	static_transform.child_frame_id = child_id;
	tf::transformEigenToMsg(tr, static_transform.transform);

	vector.emplace_back(static_transform);
}

//! Publishes static transform
void UAS::publish_static_transform(const std::string &frame_id, const std::string &child_id, const Eigen::Affine3d &tr)
{
	geometry_msgs::TransformStamped static_transformStamped;

	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = frame_id;
	static_transformStamped.child_frame_id = child_id;
	tf::transformEigenToMsg(tr, static_transformStamped.transform);

	tf2_static_broadcaster.sendTransform(static_transformStamped);
}
