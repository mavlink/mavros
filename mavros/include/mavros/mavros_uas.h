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
 * Copyright 2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <array>
#include <mutex>
#include <atomic>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>

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

	//! Type matching rosmsg for covariance 3x3
	typedef boost::array<double, 9> Covariance3d;
	//! Type matching rosmsg for covarince 6x6
	typedef boost::array<double, 36> Covariance6d;

	//! Eigen::Map for Covariance3d
	typedef Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > EigenMapCovariance3d;
	typedef Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > EigenMapConstCovariance3d;
	//! Eigen::Map for Covariance6d
	typedef Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > EigenMapCovariance6d;
	typedef Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > EigenMapConstCovariance6d;

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
	void update_heartbeat(uint8_t type_, uint8_t autopilot_, uint8_t base_mode_);

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

	/**
	 * @brief Returns arming status
	 *
	 * @note There may be race condition between SET_MODE and HEARTBEAT.
	 */
	inline bool get_armed() {
		uint8_t base_mode_ = base_mode;
		return base_mode_ & MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/**
	 * @brief Returns HIL status
	 */
	inline bool get_hil_state() {
		uint8_t base_mode_ = base_mode;
		return base_mode_ & MAV_MODE_FLAG_HIL_ENABLED;
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
	 * @brief Function to match the received orientation received by MAVLink msg
	 *        and the rotation of the sensor relative to the FCU.
	 */
	static Eigen::Quaterniond sensor_orientation_matching(MAV_SENSOR_ORIENTATION orientation);

	/**
	 * @brief Retrieve alias of the orientation received by MAVLink msg.
	 */
	static std::string str_sensor_orientation(MAV_SENSOR_ORIENTATION orientation);

	/**
	 * @brief Retrieve sensor orientation number from alias name.
	 */
	static int orientation_from_str(const std::string &sensor_orientation);

	/* -*- frame conversion utilities -*- */

	/**
	 * @brief Convert euler angles to quaternion.
	 */
	static Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);

	/**
	 * @brief Convert euler angles to quaternion.
	 *
	 * @return quaternion, same as @a tf::quaternionFromRPY() but in Eigen format.
	 */
	static inline Eigen::Quaterniond quaternion_from_rpy(const double roll, const double pitch, const double yaw) {
		return quaternion_from_rpy(Eigen::Vector3d(roll, pitch, yaw));
	}

	/**
	 * @brief Convert quaternion to euler angles
	 *
	 * Reverse operation to @a quaternion_from_rpy()
	 */
	static Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q);

	/**
	 * @brief Convert quaternion to euler angles
	 */
	static inline void quaternion_to_rpy(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
		const auto rpy = quaternion_to_rpy(q);
		roll = rpy.x();
		pitch = rpy.y();
		yaw = rpy.z();
	}

	/**
	 * @brief Get Yaw angle from quaternion
	 *
	 * Replacement function for @a tf::getYaw()
	 */
	static double quaternion_get_yaw(const Eigen::Quaterniond &q);

	/**
	 * @brief Store Quaternion to MAVLink float[4] format
	 *
	 * MAVLink uses wxyz order, wile Eigen::Quaterniond uses xyzw internal order,
	 * so it can't be stored to array using Eigen::Map.
	 */
	static inline void quaternion_to_mavlink(const Eigen::Quaterniond &q, float qmsg[4]) {
		qmsg[0] = q.w();
		qmsg[1] = q.x();
		qmsg[2] = q.y();
		qmsg[3] = q.z();
	}

	/**
	 * @brief Orientation transform options when applying rotations to data
	 */
	enum class STATIC_TRANSFORM : uint8_t {
		NED_TO_ENU,	//!< will change orinetation from being expressed WRT NED frame to WRT ENU frame
		ENU_TO_NED,	//!< change from expressed WRT ENU frame to WRT NED frame
		AIRCRAFT_TO_BASELINK,	//!< change from expressed WRT aircraft frame to WRT to baselink frame
		BASELINK_TO_AIRCRAFT	//!< change from expressed WRT baselnk to WRT aircraft
	};

	/**
	 * @brief Transform representation of attitude from 1 frame to another
	 * (e.g. transfrom attitude from representing  from base_link -> NED
	 *               to representing base_link -> ENU)
	 *
	 * General function. Please use specialized enu-ned and ned-enu variants.
	 */
	static Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const STATIC_TRANSFORM transform);

	/**
	 * @brief Transform data experessed in one frame to another frame.
	 *
	 * General function. Please use specialized enu-ned and ned-enu variants.
	 */
	static Eigen::Vector3d transform_frame(const Eigen::Vector3d &vec, const Eigen::Quaterniond &q);

	/**
	 * @brief Transform convariance expressed in one frame to another
	 *
	 * General function. Please use specialized enu-ned and ned-enu variants.
	 */
	static Covariance3d transform_frame(const Covariance3d &cov, const Eigen::Quaterniond &q);

	// XXX TODO implement that function
	static Covariance6d transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q);

	/**
	 * @brief Transform data experessed in one frame to another frame.
	 *
	 * General function. Please use specialized enu-ned and ned-enu variants.
	 */
	static Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const STATIC_TRANSFORM transform);

	/**
	 * @brief Transform convariance expressed in one frame to another
	 *
	 * General function. Please use specialized enu-ned and ned-enu variants.
	 */
	static Covariance3d transform_static_frame(const Covariance3d &cov, const STATIC_TRANSFORM transform);

	// XXX TODO implement that function
	static Covariance6d transform_static_frame(const Covariance6d &cov, const STATIC_TRANSFORM transform);

	/**
	 * @brief Transform from attitude represented WRT NED frame to attitude
	 *		  represented WRT ENU frame
	 */
	template<class T>
	static inline T transform_orientation_ned_enu(const T &in) {
		return transform_orientation(in, STATIC_TRANSFORM::NED_TO_ENU);
	}

	/**
	 * @brief Transform from attitude represented WRT ENU frame to
	 *		  attitude represented WRT NED frame
	 */
	template<class T>
	static inline T transform_orientation_enu_ned(const T &in) {
		return transform_orientation(in, STATIC_TRANSFORM::ENU_TO_NED);
	}

	/**
	 * @brief Transform from attitude represented WRT aircraft frame to
	 *		  attitude represented WRT base_link frame
	 */
	template<class T>
	static inline T transform_orientation_aircraft_baselink(const T &in) {
		return transform_orientation(in, STATIC_TRANSFORM::AIRCRAFT_TO_BASELINK);
	}

	/**
	 * @brief Transform from attitude represented WRT baselink frame to
	 *		  attitude represented WRT body frame
	 */
	template<class T>
	static inline T transform_orientation_baselink_aircraft(const T &in) {
		return transform_orientation(in, STATIC_TRANSFORM::BASELINK_TO_AIRCRAFT);
	}

	/**
	 * @brief Transform data expressed in NED to ENU frame.
	 *
	 */
	template<class T>
	static inline T transform_frame_ned_enu(const T &in) {
		return transform_static_frame(in, STATIC_TRANSFORM::NED_TO_ENU);
	}

	/**
	 * @brief Transform data expressed in ENU to NED frame.
	 *
	 */
	template<class T>
	static inline T transform_frame_enu_ned(const T &in) {
		return transform_static_frame(in, STATIC_TRANSFORM::ENU_TO_NED);
	}

	/**
	 * @brief Transform data expressed in Aircraft frame to Baselink frame.
	 *
	 */
	template<class T>
	static inline T transform_frame_aircraft_baselink(const T &in) {
		return transform_static_frame(in, STATIC_TRANSFORM::AIRCRAFT_TO_BASELINK);
	}

	/**
	 * @brief Transform data expressed in Baselink frame to Aircraft frame.
	 *
	 */
	template<class T>
	static inline T transform_frame_baselink_aircraft(const T &in) {
		return transform_static_frame(in, STATIC_TRANSFORM::BASELINK_TO_AIRCRAFT);
	}

	/**
	 * @brief Transform data expressed in aircraft frame to NED frame.
	 * Assumes quaternion represents rotation from aircraft frame to NED frame.
	 */
	template<class T>
	static inline T transform_frame_aircraft_ned(const T &in,const Eigen::Quaterniond &q) {
		return transform_frame(in, q);
	}

	/**
	 * @brief Transform data expressed in NED to aircraft frame.
	 * Assumes quaternion represents rotation from NED to aircraft frame.
	 */
	template<class T>
	static inline T transform_frame_ned_aircraft(const T &in,const Eigen::Quaterniond &q) {
		return transform_frame(in, q);
	}

	/**
	 * @brief Transform data expressed in aircraft frame to ENU frame.
	 * Assumes quaternion represents rotation from aircraft frame to ENU frame.
	 */
	template<class T>
	static inline T transform_frame_aircraft_enu(const T &in,const Eigen::Quaterniond &q) {
		return transform_frame(in, q);
	}

	/**
	 * @brief Transform data expressed in ENU to aircraft frame.
	 * Assumes quaternion represents rotation from ENU to aircraft frame.
	 */
	template<class T>
	static inline T transform_frame_enu_aircraft(const T &in,const Eigen::Quaterniond &q) {
		return transform_frame(in, q);
	}

	/**
	 * @brief Transform data expressed in ENU to base_link frame.
	 * Assumes quaternion represents rotation from ENU to base_link frame.
	 */
	template<class T>
	static inline T transform_frame_enu_baselink(const T &in,const Eigen::Quaterniond &q) {
		return transform_frame(in, q);
	}

	/**
	 * @brief Transform data expressed in baselink to ENU frame.
	 * Assumes quaternion represents rotation from basel_link to ENU frame.
	 */
	template<class T>
	static inline T transform_frame_baselink_enu(const T &in,const Eigen::Quaterniond &q) {
		return transform_frame(in, q);
	}

	/**
	 * @brief Transform heading from ROS to FCU frame.
	 */
	static inline double transform_frame_yaw_enu_ned(double yaw) {
		return transform_frame_yaw(yaw);
	}

	/**
	 * @brief Transform heading from FCU to ROS frame.
	 */
	static inline double transform_frame_yaw_ned_enu(double yaw) {
		return transform_frame_yaw(yaw);
	}

private:
	std::recursive_mutex mutex;

	std::atomic<uint8_t> type;
	std::atomic<uint8_t> autopilot;
	std::atomic<uint8_t> base_mode;

	uint8_t target_system;
	uint8_t target_component;

	std::atomic<bool> connected;

	sensor_msgs::Imu::Ptr imu_data;

	sensor_msgs::NavSatFix::Ptr gps_fix;
	float gps_eph;
	float gps_epv;
	int gps_fix_type;
	int gps_satellites_visible;

	std::atomic<uint64_t> time_offset;

	std::atomic<bool> fcu_caps_known;
	std::atomic<uint64_t> fcu_capabilities;

	static inline double transform_frame_yaw(double yaw) {
		return -yaw;
	}
};
};	// namespace mavros
