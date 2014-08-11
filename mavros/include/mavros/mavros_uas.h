/**
 * @brief MAVROS Plugin context
 * @file mavros_plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
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
#include <mavros/mavconn_interface.h>

namespace mavplugin {
typedef std::lock_guard<std::recursive_mutex> lock_guard;
typedef std::unique_lock<std::recursive_mutex> unique_lock;

/**
 * @brief helper for mavlink_msg_*_pack_chan()
 *
 * Filler for first arguments of *_pack_chan functions.
 */
#define UAS_PACK_CHAN(uasobjptr)			\
	(uasobjptr)->mav_link->get_system_id(), 	\
	(uasobjptr)->mav_link->get_component_id(), 	\
	(uasobjptr)->mav_link->get_channel()

/**
 * @brief helper for pack messages with target fields
 *
 * Filler for target_system, target_component fields.
 */
#define UAS_PACK_TGT(uasobjptr)				\
	(uasobjptr)->get_tgt_system(), 			\
	(uasobjptr)->get_tgt_component()


/**
 * @brief UAS handler for plugins
 */
class UAS {
public:
	UAS();
	~UAS() {};

	/**
	 * Stop UAS
	 */
	void stop(void);

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
		lock_guard lock(mutex);

		if (conn_ != connected) {
			connected = conn_;
			sig_connection_changed(connected);
		}
	}

	inline enum MAV_TYPE get_type() {
		uint8_t type_ = type;
		return static_cast<enum MAV_TYPE>(type_);
	};

	inline enum MAV_AUTOPILOT get_autopilot() {
		uint8_t autopilot_ = autopilot;
		return static_cast<enum MAV_AUTOPILOT>(autopilot_);
	};

	/**
	 * @brief Return communication target system
	 */
	inline uint8_t get_tgt_system() {
		return target_system; // not changed after configuration
	};

	/**
	 * @brief Return communication target component
	 */
	inline uint8_t get_tgt_component() {
		return target_component; // not changed after configuration
	};

	inline void set_tgt(uint8_t sys, uint8_t comp) {
		target_system = sys;
		target_component = comp;
	};

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

	/**
	 * For APM quirks
	 */
	inline bool is_ardupilotmega() {
		return MAV_AUTOPILOT_ARDUPILOTMEGA == get_autopilot();
	};

	/**
	 * For PX4 quirks
	 */
	inline bool is_px4() {
		return MAV_AUTOPILOT_PX4 == get_autopilot();
	}

	/**
	 * This signal emith when status was changes
	 *
	 * @param bool connection status
	 */
	boost::signals2::signal<void(bool)> sig_connection_changed;

	inline bool get_connection_status() {
		lock_guard lock(mutex);
		return connected;
	};

	/**
	 * MAVLink device conection
	 */
	boost::shared_ptr<mavconn::MAVConnInterface> mav_link;

	inline void set_mav_link(const boost::shared_ptr<mavconn::MAVConnInterface> &link_) {
		mav_link = link_;
	};

private:
	std::recursive_mutex mutex;
	std::atomic<uint8_t> type;
	std::atomic<uint8_t> autopilot;
	uint8_t target_system;
	uint8_t target_component;
	bool connected;
	tf::Vector3 angular_velocity;
	tf::Vector3 linear_acceleration;
	tf::Quaternion orientation;
};

}; // namespace mavplugin
