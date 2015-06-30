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
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <array>
#include <mutex>
#include <atomic>
#include <tf/transform_datatypes.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>

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
 * @brief helper for mavlink_msg_*_pack_chan()
 *
 * Filler for first arguments of *_pack_chan functions.
 */
#define UAS_PACK_CHAN(uasobjptr)			\
	UAS_FCU(uasobjptr)->get_system_id(),		\
	UAS_FCU(uasobjptr)->get_component_id(),		\
	UAS_FCU(uasobjptr)->get_channel()

/**
 * @brief helper for pack messages with target fields
 *
 * Filler for target_system, target_component fields.
 */
#define UAS_PACK_TGT(uasobjptr)				\
	(uasobjptr)->get_tgt_system(),			\
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
	typedef boost::array<double, 9> Covariance3x3;
	typedef boost::array<double, 36> Covariance6x6;

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

	/**
	 * @brief Mavros diagnostic updater
	 */
	diagnostic_updater::Updater diag_updater;

	/**
	 * @brief This signal emit when status was changed
	 *
	 * @param bool connection status
	 */
	boost::signals2::signal<void(bool)> sig_connection_changed;

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
	void update_heartbeat(uint8_t type_, uint8_t autopilot_);

	/**
	 * Update autopilot connection status (every HEARTBEAT/conn_timeout)
	 */
	void update_connection_status(bool conn_);

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
	void update_attitude_imu(tf::Quaternion &q, tf::Vector3 &av, tf::Vector3 &lacc);

	/**
	 * @brief Get Attitude angular velocity vector
	 * @return angilar velocity [ENU, body-fixed]
	 */
	tf::Vector3 get_attitude_angular_velocity();

	/**
	 * @brief Get Attitude linear acceleration vector
	 * @return linear acceleration [ENU, body-fixed]
	 */
	tf::Vector3 get_attitude_linear_acceleration();

	/**
	 * @brief Get Attitude orientation quaternion
	 * @return orientation quaternion [ENU, body-fixed]
	 */
	tf::Quaternion get_attitude_orientation();


	/* -*- GPS data -*- */

	//! Store GPS RAW data
	void update_gps_fix_epts(sensor_msgs::NavSatFix::Ptr &fix,
			float eph, float epv,
			int fix_type, int satellites_visible);

	//! Returns EPH, EPV, Fix type and satellites visible
	void get_gps_epts(float &eph, float &epv, int &fix_type, int &satellites_visible);

	//! Retunrs last GPS RAW message
	sensor_msgs::NavSatFix::Ptr get_gps_fix();


	/* -*- time sync -*- */

	inline void set_time_offset(uint64_t offset_ns) {
		time_offset = offset_ns;
	}

	inline uint64_t get_time_offset(void) {
		return time_offset;
	}

	/* -*- autopilot version -*- */
	uint64_t get_capabilities();
	void update_capabilities(bool known, uint64_t caps = 0);

	/* -*- utils -*- */

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

	/**
	 * @brief Represent MAV_AUTOPILOT as string
	 */
	static std::string str_autopilot(enum MAV_AUTOPILOT ap);

	/**
	 * @brief Represent MAV_TYPE as string
	 */
	static std::string str_type(enum MAV_TYPE type);

	/**
	 * @brief Represent MAV_STATE as string
	 */
	static std::string str_system_status(enum MAV_STATE st);

	/**
	 * @brief Function to match the received orientation received by DISTANCE_SENSOR msg
	 *        and the rotation of the sensor relative to the FCU.
	 */
	static tf::Vector3 sensor_orientation_matching(MAV_SENSOR_ORIENTATION orientation);

	static tf::Vector3 transform_frame_xyz(double _x, double _y, double _z);
	static tf::Quaternion transform_frame_attitude_q(tf::Quaternion qo);
	static tf::Vector3 transform_frame_attitude_rpy(double _roll, double _pitch, double _yaw);
	static Covariance6x6 transform_frame_covariance_pose6x6(Covariance6x6 &_covariance);
	static Covariance3x3 transform_frame_covariance_general3x3(Covariance3x3 &_covariance);

	/**
	 * @brief Function to convert general XYZ values from ENU to NED frames
	 * @param _x: X coordinate/direction value
	 * @param _y: Y coordinate/direction value
	 * @param _z: Z coordinate/direction value
	 * @return Translated XYZ values in NED frame
	 */
	static inline tf::Vector3 transform_frame_enu_ned_xyz(double _x, double _y, double _z){
		return transform_frame_xyz(_x, _y, _z);
	}

	/**
	 * @brief Function to convert general XYZ values from NED to ENU frames
	 * @param _x: X coordinate value
	 * @param _y: Y coordinate value
	 * @param _z: Z coordinate value
	 * @return Translated XYZ values in ENU frame
	 */
	static inline tf::Vector3 transform_frame_ned_enu_xyz(double _x, double _y, double _z){
		return transform_frame_xyz(_x, _y, _z);
	}

	/**
	 * @brief Function to convert attitude quaternion values from ENU to NED frames
	 * @param qo: tf::Quaternion format
	 * @return Rotated Quaternion values in NED frame
	 */
	static inline tf::Quaternion transform_frame_enu_ned_attitude_q(tf::Quaternion qo){
		return transform_frame_attitude_q(qo);
	}

	/**
	 * @brief Function to convert attitude quaternion values from NED to ENU frames
	 * @param qo: tf::Quaternion format
	 * @return Rotated Quaternion values in ENU frame
	 */
	static inline tf::Quaternion transform_frame_ned_enu_attitude_q(tf::Quaternion qo){
		return transform_frame_attitude_q(qo);
	}

	/**
	 * @brief Function to convert attitude euler angle values from ENU to NED frames
	 * @param _roll: Roll value
	 * @param _pitch: Pitch value
	 * @param _yaw: Yaw value
	 * @return Rotated RPY angles values in NED frame
	 */
	static inline tf::Vector3 transform_frame_enu_ned_attitude_rpy(double _roll, double _pitch, double _yaw){
		return transform_frame_attitude_rpy(_roll, _pitch, _yaw);
	}

	/**
	 * @brief Function to convert attitude euler angle values from NED to ENU frames
	 * @param _roll: Roll value
	 * @param _pitch: Pitch value
	 * @param _yaw: Yaw value
	 * @return Rotated RPY angles values in ENU frame
	 */
	static inline tf::Vector3 transform_frame_ned_enu_attitude_rpy(double _roll, double _pitch, double _yaw){
		return transform_frame_attitude_rpy(_roll, _pitch, _yaw);
	}

	/**
	 * @brief Function to convert full 6D pose covariance matrix values from ENU to NED frames
	 * @details Full 6D pose covariance matrix format: a 3D position plus three attitude angles: roll, pitch and yaw.
	 * @param _covariance: 6x6 double precision covariance matrix
	 * @return Propagated 6x6 covariance matrix in NED frame, if _covariance[0] != -1
	 */
	static inline Covariance6x6 transform_frame_enu_ned_covariance_pose6x6(Covariance6x6 _covariance){
		return transform_frame_covariance_pose6x6(_covariance);
	}

	/**
	 * @brief Function to convert full 6D pose covariance matrix values from NED to ENU frames
	 * @details Full 6D pose covariance matrix format: a 3D position plus three attitude angles: roll, pitch and yaw.
	 * @param _covariance: 6x6 double precision covariance matrix
	 * @return Propagated 6x6 covariance matrix in ENU frame, if _covariance[0] != -1
	 */
	static inline Covariance6x6 transform_frame_ned_enu_covariance_pose6x6(Covariance6x6 _covariance){
		return transform_frame_covariance_pose6x6(_covariance);
	}

	/**
	 * Matrix formats for the above funtions:
	 *
	 * Cov_matrix =	| var_x  cov_xy cov_xz cov_xZ cov_xY cov_xX |
	 *              | cov_yx var_y  cov_yz cov_yZ cov_yY cov_yX |
	 *              | cov_zx cov_zy var_z  cov_zZ cov_zY cov_zX |
	 *              | cov_Zx cov_Zy cov_Zz var_Z  cov_ZY cov_ZX |
	 *              | cov_Yx cov_Yy cov_Yz cov_YZ var_Y  cov_YX |
	 *              | cov_Xx cov_Xy cov_Xz cov_XZ cov_XY var_X  |
	 *
	 * Transf_matrix = | 1	 0	 0	 0	 0	 0 |
	 *                 | 0	-1       0	 0	 0	 0 |
	 *                 | 0	 0	-1	 0	 0	 0 |
	 *                 | 0	 0	 0	 1	 0	 0 |
	 *                 | 0	 0	 0	 0	-1	 0 |
	 *                 | 0	 0	 0	 0	 0	-1 |
	 *
	 * Compute Covariance matrix in another frame: (according to the law of propagation of covariance)
	 *
	 *                      C' = R * C * R^T
	 */

	/**
	 * @brief Function to convert position, linear acceleration, angular velocity or attitude RPY covariance matrix values from ENU to NED frames
	 * @param _covariance: 3x3 double precision covariance matrix
	 * @return Propagated 3x3 covariance matrix in NED frame, if _covariance[0] != -1
	 */
	static inline Covariance3x3 transform_frame_covariance_enu_ned_general3x3(Covariance3x3 _covariance){
		return transform_frame_covariance_general3x3(_covariance);
	}

	/**
	 * @brief Function to convert position, linear acceleration, angular velocity or attitude RPY covariance matrix values from NED to ENU frames
	 * @param _covariance: 3x3 double precision covariance matrix
	 * @return Propagated 3x3 covariance matrix in ENU frame, if _covariance[0] != -1
	 */
	static inline Covariance3x3 transform_frame_covariance_ned_enu_general3x3(Covariance3x3 _covariance){
		return transform_frame_covariance_general3x3(_covariance);
	}

	/**
	 * Matrix formats for the above funtions:
	 *
	 * Pos_Cov_matrix =	| var_x  cov_xy cov_xz |
	 *                      | cov_yx var_y  cov_yz |
	 *                      | cov_zx cov_zy var_z  |
	 *
	 * Vel_Cov_matrix =	| var_vx  cov_vxvy cov_vxvz |
	 *                      | cov_vyvx var_vy  cov_vyvz |
	 *                      | cov_vzvx cov_vzvy var_vz  |
	 *
	 * Att_Cov_matrix =	| var_Z  cov_ZY cov_ZX |
	 *                      | cov_YZ var_Y  cov_YX |
	 *                      | cov_XZ cov_XY var_X  |
	 *
	 * Note that for ROS<->ENU frame transformations, the transformation matrix is the same for position and attitude.
	 *
	 * Rot_matrix = | 1	 0	 0 |
	 *              | 0	-1       0 |
	 *              | 0	 0	-1 |
	 *
	 * Compute Covariance matrix in another frame: (according to the law of propagation of covariance)
	 *
	 *                      C' = T * C * T^t
	 */

private:
	std::recursive_mutex mutex;

	std::atomic<uint8_t> type;
	std::atomic<uint8_t> autopilot;

	uint8_t target_system;
	uint8_t target_component;

	std::atomic<bool> connected;

	tf::Quaternion imu_orientation;
	tf::Vector3 imu_angular_velocity;
	tf::Vector3 imu_linear_acceleration;

	sensor_msgs::NavSatFix::Ptr gps_fix;
	float gps_eph;
	float gps_epv;
	int gps_fix_type;
	int gps_satellites_visible;

	std::atomic<uint64_t> time_offset;

	std::atomic<bool> fcu_caps_known;
	std::atomic<uint64_t> fcu_capabilities;
};
};	// namespace mavros
