/**
 * @brief MAVROS Plugin context
 * @file mavros_uas.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2014,2015,2016,2017 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <array>
#include <mutex>
#include <atomic>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>
#include <mavros/utils.h>
#include <mavros/frame_tf.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

namespace mavros {
/**
 * @brief helper accessor to FCU link interface
 */
#define UAS_FCU(uasobjptr)				\
	((uasobjptr)->fcu_link)

/**
 * @brief helper accessor to diagnostic updater
 */
#define UAS_DIAG(uasobjptr)				\
	((uasobjptr)->diag_updater)


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
	using ConnectionCb = std::function<void(bool)>;
	using lock_guard = std::lock_guard<std::recursive_mutex>;
	using unique_lock = std::unique_lock<std::recursive_mutex>;

	// common enums used by UAS
	using MAV_TYPE = mavlink::common::MAV_TYPE;
	using MAV_AUTOPILOT = mavlink::common::MAV_AUTOPILOT;
	using MAV_MODE_FLAG = mavlink::common::MAV_MODE_FLAG;
	using MAV_STATE = mavlink::common::MAV_STATE;
	using timesync_mode = utils::timesync_mode;

	UAS();
	~UAS() {};

	/**
	 * @brief MAVLink FCU device conection
	 */
	mavconn::MAVConnInterface::Ptr fcu_link;

	/**
	 * @brief Mavros diagnostic updater
	 */
	diagnostic_updater::Updater diag_updater;

	/**
	 * @brief Return connection status
	 */
	inline bool is_connected() {
		return connected;
	}

	/* -*- HEARTBEAT data -*- */

	/**
	 * Update autopilot type on every HEARTBEAT
	 */
	void update_heartbeat(uint8_t type_, uint8_t autopilot_, uint8_t base_mode_);

	/**
	 * Update autopilot connection status (every HEARTBEAT/conn_timeout)
	 */
	void update_connection_status(bool conn_);

	/**
	 * @brief Add connection change handler callback
	 */
	void add_connection_change_handler(ConnectionCb cb);

	/**
	 * @brief Returns vehicle type
	 */
	inline MAV_TYPE get_type() {
		std::underlying_type<MAV_TYPE>::type type_ = type;
		return static_cast<MAV_TYPE>(type_);
	}

	/**
	 * @brief Returns autopilot type
	 */
	inline MAV_AUTOPILOT get_autopilot() {
		std::underlying_type<MAV_AUTOPILOT>::type autopilot_ = autopilot;
		return static_cast<MAV_AUTOPILOT>(autopilot_);
	}

	/**
	 * @brief Returns arming status
	 *
	 * @note There may be race condition between SET_MODE and HEARTBEAT.
	 */
	inline bool get_armed() {
		uint8_t base_mode_ = base_mode;
		return base_mode_ & utils::enum_value(MAV_MODE_FLAG::SAFETY_ARMED);
	}

	/**
	 * @brief Returns HIL status
	 */
	inline bool get_hil_state() {
		uint8_t base_mode_ = base_mode;
		return base_mode_ & utils::enum_value(MAV_MODE_FLAG::HIL_ENABLED);
	}

	/* -*- FCU target id pair -*- */

	/**
	 * @brief Return communication target system
	 */
	inline uint8_t get_tgt_system() {
		return target_system;	// not changed after configuration
	}

	/**
	 * @brief Return communication target component
	 */
	inline uint8_t get_tgt_component() {
		return target_component;// not changed after configuration
	}

	inline void set_tgt(uint8_t sys, uint8_t comp) {
		target_system = sys;
		target_component = comp;
	}


	/* -*- IMU data -*- */

	//! Store IMU data
	void update_attitude_imu(sensor_msgs::Imu::Ptr &imu);

	//! Get IMU data
	sensor_msgs::Imu::Ptr get_attitude_imu();

	/**
	 * @brief Get Attitude orientation quaternion
	 * @return orientation quaternion [ENU]
	 */
	geometry_msgs::Quaternion get_attitude_orientation();

	/**
	 * @brief Get angular velocity from IMU data
	 * @return vector3
	 */
	geometry_msgs::Vector3 get_attitude_angular_velocity();


	/* -*- GPS data -*- */

	//! Store GPS RAW data
	void update_gps_fix_epts(sensor_msgs::NavSatFix::Ptr &fix,
			float eph, float epv,
			int fix_type, int satellites_visible);

	//! Returns EPH, EPV, Fix type and satellites visible
	void get_gps_epts(float &eph, float &epv, int &fix_type, int &satellites_visible);

	//! Retunrs last GPS RAW message
	sensor_msgs::NavSatFix::Ptr get_gps_fix();


	/* -*- transform -*- */

	tf2_ros::Buffer tf2_buffer;
	tf2_ros::TransformListener tf2_listener;
	tf2_ros::TransformBroadcaster tf2_broadcaster;

	/* -*- time sync -*- */

	inline void set_time_offset(uint64_t offset_ns) {
		time_offset = offset_ns;
	}

	inline uint64_t get_time_offset(void) {
		return time_offset;
	}

	inline void set_timesync_mode(timesync_mode mode) {
		tsync_mode = mode;
	}

	inline timesync_mode get_timesync_mode(void) {
		return tsync_mode;
	}

	/* -*- autopilot version -*- */
	uint64_t get_capabilities();
	void update_capabilities(bool known, uint64_t caps = 0);

	/**
	 * @brief Compute FCU message time from time_boot_ms or time_usec field
	 *
	 * Uses time_offset for calculation
	 *
	 * @return FCU time if it is known else current wall time.
	 */
	ros::Time synchronise_stamp(uint32_t time_boot_ms);
	ros::Time synchronise_stamp(uint64_t time_usec);

	/**
	 * @brief Create message header from time_boot_ms or time_usec stamps and frame_id.
	 *
	 * Setting frame_id and stamp are pretty common, this little helper should reduce LOC.
	 *
	 * @param[in] frame_id    frame for header
	 * @param[in] time_stamp  mavlink message time
	 * @return Header with syncronized stamp and frame id
	 */
	template<typename T>
	inline std_msgs::Header synchronized_header(const std::string &frame_id, const T time_stamp) {
		std_msgs::Header out;
		out.frame_id = frame_id;
		out.stamp = synchronise_stamp(time_stamp);
		return out;
	}

	/* -*- utils -*- */

	/**
	 * Helper template to set target id's of message.
	 */
	template<typename _T>
	inline void msg_set_target(_T &msg) {
		msg.target_system = get_tgt_system();
		msg.target_component = get_tgt_component();
	}

	/**
	 * @brief Check that sys/comp id's is my target
	 */
	inline bool is_my_target(uint8_t sysid, uint8_t compid) {
		return sysid == get_tgt_system() && compid == get_tgt_component();
	}

	/**
	 * @brief Check that system id is my target
	 */
	inline bool is_my_target(uint8_t sysid) {
		return sysid == get_tgt_system();
	}

	/**
	 * @brief Check that FCU is APM
	 */
	inline bool is_ardupilotmega() {
		return MAV_AUTOPILOT::ARDUPILOTMEGA == get_autopilot();
	}

	/**
	 * @brief Check that FCU is PX4
	 */
	inline bool is_px4() {
		return MAV_AUTOPILOT::PX4 == get_autopilot();
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

private:
	std::recursive_mutex mutex;

	std::atomic<uint8_t> type;
	std::atomic<uint8_t> autopilot;
	std::atomic<uint8_t> base_mode;

	uint8_t target_system;
	uint8_t target_component;

	std::atomic<bool> connected;
	std::vector<ConnectionCb> connection_cb_vec;

	sensor_msgs::Imu::Ptr imu_data;

	sensor_msgs::NavSatFix::Ptr gps_fix;
	float gps_eph;
	float gps_epv;
	int gps_fix_type;
	int gps_satellites_visible;

	std::atomic<uint64_t> time_offset;
	timesync_mode tsync_mode;

	std::atomic<bool> fcu_caps_known;
	std::atomic<uint64_t> fcu_capabilities;
};
}	// namespace mavros
