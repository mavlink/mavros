/**
 * @brief MAVROS Plugin context
 * @file mavros_uas.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#pragma once

#include <mutex>
#include <atomic>
#include <tf/transform_datatypes.h>
#include <mavconn/interface.h>

namespace mavros {

/**
 * @brief helper accessor to FCU link interface
 */
#define UAS_FCU(uasobjptr)				\
	((uasobjptr)->fcu_link)

/**
 * @brief helper for mavlink_msg_*_pack_chan()
 *
 * Filler for first arguments of *_pack_chan functions.
 */
#define UAS_PACK_CHAN(uasobjptr)			\
	UAS_FCU(uasobjptr)->get_system_id(), 		\
	UAS_FCU(uasobjptr)->get_component_id(), 	\
	UAS_FCU(uasobjptr)->get_channel()

/**
 * @brief helper for pack messages with target fields
 *
 * Filler for target_system, target_component fields.
 */
#define UAS_PACK_TGT(uasobjptr)				\
	(uasobjptr)->get_tgt_system(), 			\
	(uasobjptr)->get_tgt_component()

/**
 * @brief UAS for plugins
 *
 * This class stores some useful data and
 * provides fcu connection, mode stringify utilities.
 *
 * Currently it stores:
 * - FCU link interface
 * - FCU System & Component ID pair
 * - Connection status (@a mavplugin::SystemStatusPlugin)
 * - Autopilot type (@a mavplugin::SystemStatusPlugin)
 * - Vehicle type (@a mavplugin::SystemStatusPlugin)
 * - IMU data (@a mavplugin::IMUPubPlugin)
 * - GPS data (@a mavplugin::GPSPlugin)
 */
class UAS {
public:
	typedef std::lock_guard<std::recursive_mutex> lock_guard;
	typedef std::unique_lock<std::recursive_mutex> unique_lock;

	UAS();
	~UAS() {};

	/**
	 * Stop UAS
	 */
	void stop(void);

	/**
	 * @brief MAVLink FCU device conection
	 */
	mavconn::MAVConnInterface::Ptr fcu_link;

	/* -*- HEARTBEAT data -*- */

	/**
	 * Update autopilot type on every HEARTBEAT
	 */
	void update_heartbeat(uint8_t type_, uint8_t autopilot_) {
		type = type_;
		autopilot = autopilot_;
	}

	/**
	 * Update autopilot connection status (every HEARTBEAT/conn_timeout)
	 */
	void update_connection_status(bool conn_) {
		if (conn_ != connected) {
			connected = conn_;
			sig_connection_changed(connected);
		}
	}

	/**
	 * @brief This signal emit when status was changed
	 *
	 * @param bool connection status
	 */
	boost::signals2::signal<void(bool)> sig_connection_changed;

	/**
	 * @brief Returns connection status
	 */
	inline bool get_connection_status() {
		return connected;
	}

	/**
	 * @brief Returns vehicle type
	 */
	inline enum MAV_TYPE get_type() {
		uint8_t type_ = type;
		return static_cast<enum MAV_TYPE>(type_);
	}

	/**
	 * @brief Returns autopilot type
	 */
	inline enum MAV_AUTOPILOT get_autopilot() {
		uint8_t autopilot_ = autopilot;
		return static_cast<enum MAV_AUTOPILOT>(autopilot_);
	}

	/* -*- FCU target id pair -*- */

	/**
	 * @brief Return communication target system
	 */
	inline uint8_t get_tgt_system() {
		return target_system; // not changed after configuration
	}

	/**
	 * @brief Return communication target component
	 */
	inline uint8_t get_tgt_component() {
		return target_component; // not changed after configuration
	}

	inline void set_tgt(uint8_t sys, uint8_t comp) {
		target_system = sys;
		target_component = comp;
	}

	/* -*- IMU data -*- */

	/**
	 * @brief Get Attitude angular velocity vector
	 * @return angilar velocity [ENU, body-fixed]
	 */
	inline tf::Vector3 get_attitude_angular_velocity() {
		lock_guard lock(mutex);
		return angular_velocity;
	}

	/**
	 * @brief Store Attitude angular velocity vector
	 * @param[in] vec angular velocity [ENU, body-fixed]
	 */
	inline void set_attitude_angular_velocity(tf::Vector3 &vec) {
		lock_guard lock(mutex);
		angular_velocity = vec;
	}

	/**
	 * @brief Get Attitude linear acceleration vector
	 * @return linear acceleration [ENU, body-fixed]
	 */
	inline tf::Vector3 get_attitude_linear_acceleration() {
		lock_guard lock(mutex);
		return linear_acceleration;
	}

	/**
	 * @brief Store Attitude linear acceleration vector
	 * @param[in] vec linear acceleration [ENU, body-fixed]
	 */
	inline void set_attitude_linear_acceleration(tf::Vector3 &vec) {
		lock_guard lock(mutex);
		linear_acceleration = vec;
	}

	/**
	 * @brief Get Attitude orientation quaternion
	 * @return orientation quaternion [ENU, body-fixed]
	 */
	inline tf::Quaternion get_attitude_orientation() {
		lock_guard lock(mutex);
		return orientation;
	}

	/**
	 * @brief Store Attitude orientation quaternion
	 * @param[in] quat orientation [ENU, body-fixed]
	 */
	inline void set_attitude_orientation(tf::Quaternion &quat) {
		lock_guard lock(mutex);
		orientation = quat;
	}

	/* -*- GPS data -*- */

	/**
	 * @brief Store GPS Lat/Long/Alt and EPH/EPV data
	 *
	 * @param[in] latitude  in deg
	 * @param[in] longitude in deg
	 * @param[in] altitude  in m
	 * @param[in] eph       in m
	 * @param[in] epv       in m
	 */
	inline void set_gps_llae(double latitude, double longitude, double altitude,
			double eph, double epv) {
		lock_guard lock(mutex);
		gps_latitude = latitude;
		gps_longitude = longitude;
		gps_altitude = altitude;
		gps_eph = eph;
		gps_epv = epv;
	}

	inline double get_gps_latitude() {
		lock_guard lock(mutex);
		return gps_latitude;
	}

	inline double get_gps_longitude() {
		lock_guard lock(mutex);
		return gps_longitude;
	}

	inline double get_gps_altitude() {
		lock_guard lock(mutex);
		return gps_altitude;
	}

	inline double get_gps_eph() {
		lock_guard lock(mutex);
		return gps_eph;
	}

	inline double get_gps_epv() {
		lock_guard lock(mutex);
		return gps_epv;
	}

	inline void set_gps_status(bool fix_status_) {
		fix_status = fix_status_;
	}

	inline bool get_gps_status() {
		return fix_status;
	}

	/* -*- time sync -*- */
	inline void set_time_offset(uint64_t offset_ns) {
		time_offset = offset_ns;
	}

	inline uint64_t get_time_offset(void) {
		return time_offset;
	}

	/* -*- utils -*- */

	/**
	 * @brief Check that FCU is APM
	 */
	inline bool is_ardupilotmega() {
		return MAV_AUTOPILOT_ARDUPILOTMEGA == get_autopilot();
	}

	/**
	 * @brief Check that FCU is PX4
	 */
	inline bool is_px4() {
		return MAV_AUTOPILOT_PX4 == get_autopilot();
	}

	/**
	 * @brief Represent FCU mode as string
	 *
	 * Port pymavlink mavutil.mode_string_v10
	 *
	 * Supported FCU's:
	 * - APM:Plane
	 * - APM:Copter
	 * - PX4
	 *
	 * @param[in] base_mode    base mode
	 * @param[in] custom_mode  custom mode data
	 */
	std::string str_mode_v10(uint8_t base_mode, uint32_t custom_mode);

	/**
	 * @brief Lookup custom mode for given string
	 *
	 * Complimentary to @a str_mode_v10()
	 *
	 * @param[in]  cmode_str   string representation of mode
	 * @param[out] custom_mode decoded value
	 * @return true if success
	 */
	bool cmode_from_str(std::string cmode_str, uint32_t &custom_mode);

	/**
	 * @brief Compute FCU message time from time_boot_ms or time_usec field
	 *
	 * Uses time_offset for calculation
	 *
	 * @return FCU time if it is known else current wall time.
	 */
	ros::Time synchronise_stamp(uint32_t time_boot_ms);
	ros::Time synchronise_stamp(uint64_t time_usec);

private:
	std::recursive_mutex mutex;
	std::atomic<uint8_t> type;
	std::atomic<uint8_t> autopilot;
	uint8_t target_system;
	uint8_t target_component;
	std::atomic<bool> connected;
	tf::Vector3 angular_velocity;
	tf::Vector3 linear_acceleration;
	tf::Quaternion orientation;
	double gps_latitude;
	double gps_longitude;
	double gps_altitude;
	double gps_eph;
	double gps_epv;
	std::atomic<bool> fix_status;
	std::atomic<uint64_t> time_offset;
};

}; // namespace mavros
