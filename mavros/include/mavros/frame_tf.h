/**
 * @brief Frame transformation utilities
 * @file frame_tf.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <array>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// for Covariance types
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace mavros {
namespace ftf {

//! Type matching rosmsg for covariance 3x3
using Covariance3d = sensor_msgs::Imu::_angular_velocity_covariance_type;

//! Type matching rosmsg for covariance 6x6
using Covariance6d = geometry_msgs::PoseWithCovariance::_covariance_type;

//! Eigen::Map for Covariance3d
using EigenMapCovariance3d = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;
using EigenMapConstCovariance3d = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;

//! Eigen::Map for Covariance6d
using EigenMapCovariance6d = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >;
using EigenMapConstCovariance6d = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >;

/**
 * @brief Orientation transform options when applying rotations to data
 */
enum class StaticTF {
	NED_TO_ENU,		//!< will change orinetation from being expressed WRT NED frame to WRT ENU frame
	ENU_TO_NED,		//!< change from expressed WRT ENU frame to WRT NED frame
	AIRCRAFT_TO_BASELINK,	//!< change from expressed WRT aircraft frame to WRT to baselink frame
	BASELINK_TO_AIRCRAFT	//!< change from expressed WRT baselnk to WRT aircraft
};

namespace detail {

/**
 * @brief Transform representation of attitude from 1 frame to another
 * (e.g. transfrom attitude from representing  from base_link -> NED
 *               to representing base_link -> ENU)
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform);

/**
 * @brief Transform data experessed in one frame to another frame.
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Eigen::Vector3d transform_frame(const Eigen::Vector3d &vec, const Eigen::Quaterniond &q);

/**
 * @brief Transform convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance3d transform_frame(const Covariance3d &cov, const Eigen::Quaterniond &q);

// XXX TODO implement that function
//Covariance6d transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform data experessed in one frame to another frame.
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const StaticTF transform);

/**
 * @brief Transform convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance3d transform_static_frame(const Covariance3d &cov, const StaticTF transform);

// XXX TODO implement that function
//Covariance6d transform_static_frame(const Covariance6d &cov, const StaticTF transform);

inline double transform_frame_yaw(double yaw) {
	return -yaw;
}

}	// namespace detail

// -*- frame tf -*-

/**
 * @brief Transform from attitude represented WRT NED frame to attitude
 *		  represented WRT ENU frame
 */
template<class T>
inline T transform_orientation_ned_enu(const T &in) {
	return detail::transform_orientation(in, StaticTF::NED_TO_ENU);
}

/**
 * @brief Transform from attitude represented WRT ENU frame to
 *		  attitude represented WRT NED frame
 */
template<class T>
inline T transform_orientation_enu_ned(const T &in) {
	return detail::transform_orientation(in, StaticTF::ENU_TO_NED);
}

/**
 * @brief Transform from attitude represented WRT aircraft frame to
 *		  attitude represented WRT base_link frame
 */
template<class T>
inline T transform_orientation_aircraft_baselink(const T &in) {
	return detail::transform_orientation(in, StaticTF::AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform from attitude represented WRT baselink frame to
 *		  attitude represented WRT body frame
 */
template<class T>
inline T transform_orientation_baselink_aircraft(const T &in) {
	return detail::transform_orientation(in, StaticTF::BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform data expressed in NED to ENU frame.
 */
template<class T>
inline T transform_frame_ned_enu(const T &in) {
	return detail::transform_static_frame(in, StaticTF::NED_TO_ENU);
}

/**
 * @brief Transform data expressed in ENU to NED frame.
 *
 */
template<class T>
inline T transform_frame_enu_ned(const T &in) {
	return detail::transform_static_frame(in, StaticTF::ENU_TO_NED);
}

/**
 * @brief Transform data expressed in Aircraft frame to Baselink frame.
 *
 */
template<class T>
inline T transform_frame_aircraft_baselink(const T &in) {
	return detail::transform_static_frame(in, StaticTF::AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform data expressed in Baselink frame to Aircraft frame.
 *
 */
template<class T>
inline T transform_frame_baselink_aircraft(const T &in) {
	return detail::transform_static_frame(in, StaticTF::BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform data expressed in aircraft frame to NED frame.
 * Assumes quaternion represents rotation from aircraft frame to NED frame.
 */
template<class T>
inline T transform_frame_aircraft_ned(const T &in,const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in NED to aircraft frame.
 * Assumes quaternion represents rotation from NED to aircraft frame.
 */
template<class T>
inline T transform_frame_ned_aircraft(const T &in,const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in aircraft frame to ENU frame.
 * Assumes quaternion represents rotation from aircraft frame to ENU frame.
 */
template<class T>
inline T transform_frame_aircraft_enu(const T &in,const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in ENU to aircraft frame.
 * Assumes quaternion represents rotation from ENU to aircraft frame.
 */
template<class T>
inline T transform_frame_enu_aircraft(const T &in,const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in ENU to base_link frame.
 * Assumes quaternion represents rotation from ENU to base_link frame.
 */
template<class T>
inline T transform_frame_enu_baselink(const T &in,const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in baselink to ENU frame.
 * Assumes quaternion represents rotation from basel_link to ENU frame.
 */
template<class T>
inline T transform_frame_baselink_enu(const T &in,const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform heading from ROS to FCU frame.
 */
inline double transform_frame_yaw_enu_ned(double yaw) {
	return detail::transform_frame_yaw(yaw);
}

/**
 * @brief Transform heading from FCU to ROS frame.
 */
inline double transform_frame_yaw_ned_enu(double yaw) {
	return detail::transform_frame_yaw(yaw);
}


// -*- utils -*-


/**
 * @brief Convert euler angles to quaternion.
 */
Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @return quaternion, same as @a tf::quaternionFromRPY() but in Eigen format.
 */
inline Eigen::Quaterniond quaternion_from_rpy(const double roll, const double pitch, const double yaw) {
	return quaternion_from_rpy(Eigen::Vector3d(roll, pitch, yaw));
}

/**
 * @brief Convert quaternion to euler angles
 *
 * Reverse operation to @a quaternion_from_rpy()
 */
Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q);

/**
 * @brief Convert quaternion to euler angles
 */
inline void quaternion_to_rpy(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
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
double quaternion_get_yaw(const Eigen::Quaterniond &q);

/**
 * @brief Store Quaternion to MAVLink float[4] format
 *
 * MAVLink uses wxyz order, wile Eigen::Quaterniond uses xyzw internal order,
 * so it can't be stored to array using Eigen::Map.
 */
inline void quaternion_to_mavlink(const Eigen::Quaterniond &q, std::array<float, 4> &qmsg)
{
	qmsg[0] = q.w();
	qmsg[1] = q.x();
	qmsg[2] = q.y();
	qmsg[3] = q.z();
}

}	// namespace ftf
}	// namespace mavros
