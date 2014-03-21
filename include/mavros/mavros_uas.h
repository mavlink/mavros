/**
 * @brief MAVROS Plugin context
 * @file mavros_plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
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

#include <mavros/mavconn_interface.h>

namespace mavplugin {

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
	void update_heartbeat(uint8_t type_, uint8_t autopilot_);

	/**
	 * Update autopilot connection status (every HEARTBEAT/conn_timeout)
	 */
	void update_connection_status(bool conn_);

	inline enum MAV_TYPE get_type() {
		boost::recursive_mutex::scoped_lock lock(mutex);
		return type;
	};

	inline enum MAV_AUTOPILOT get_autopilot() {
		boost::recursive_mutex::scoped_lock lock(mutex);
		return autopilot;
	};

	inline uint8_t get_tgt_system() {
		return target_system; // not changed after configuration
	};

	inline uint8_t get_tgt_component() {
		return target_component; // not changed after configuration
	};

	inline void set_tgt(uint8_t sys, uint8_t comp) {
		target_system = sys;
		target_component = comp;
	};

	/**
	 * For APM quirks
	 */
	inline bool is_ardupilotmega() {
		return MAV_AUTOPILOT_ARDUPILOTMEGA == get_autopilot();
	};

	/**
	 * This signal emith when status was changes
	 *
	 * @param bool connection status
	 */
	boost::signals2::signal<void(bool)> sig_connection_changed;

	inline bool get_connection_status() {
		boost::recursive_mutex::scoped_lock lock(mutex);
		return connected;
	};

	/**
	 * MAVLink device conection
	 */
	boost::shared_ptr<mavconn::MAVConnInterface> mav_link;

	inline void set_mav_link(const boost::shared_ptr<mavconn::MAVConnInterface> &link_) {
		mav_link = link_;
	};

	/**
	 * for plugin timers
	 */
	boost::asio::io_service timer_service;

private:
	boost::recursive_mutex mutex;
	enum MAV_TYPE type;
	enum MAV_AUTOPILOT autopilot;
	uint8_t target_system;
	uint8_t target_component;
	bool connected;
	std::unique_ptr<boost::asio::io_service::work> timer_work;
	boost::thread timer_thread;
};

}; // namespace mavplugin
