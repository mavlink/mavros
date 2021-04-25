/**
 * @brief Frame transformation utilities
 * @file frame_tf.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Eddy Scott <scott.edward@aurora.aero>
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2016,2017 Vladimir Ermakov.
 * Copyright 2017,2018 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <array>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <ros/assert.h>

// for Covariance types
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace mavros {
namespace ftf {
//! Type matching rosmsg for 3x3 covariance matrix
using Covariance3d = sensor_msgs::Imu::_angular_velocity_covariance_type;

//! Type matching rosmsg for 6x6 covariance matrix
using Covariance6d = geometry_msgs::PoseWithCovariance::_covariance_type;

//! Type matching rosmsg for 9x9 covariance matrix
using Covariance9d = boost::array<double, 81>;

//! Eigen::Map for Covariance3d
using EigenMapCovariance3d = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;
using EigenMapConstCovariance3d = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;

//! Eigen::Map for Covariance6d
using EigenMapCovariance6d = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >;
using EigenMapConstCovariance6d = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >;

//! Eigen::Map for Covariance9d
using EigenMapCovariance9d = Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> >;
using EigenMapConstCovariance9d = Eigen::Map<const Eigen::Matrix<double, 9, 9, Eigen::RowMajor> >;

/**
 * @brief Orientation transform options when applying rotations to data
 */
enum class StaticTF {
	NED_TO_ENU,		//!< will change orientation from being expressed WRT NED frame to WRT ENU frame
	ENU_TO_NED,		//!< change from expressed WRT ENU frame to WRT NED frame
	AIRCRAFT_TO_BASELINK,	//!< change from expressed WRT aircraft frame to WRT to baselink frame
	BASELINK_TO_AIRCRAFT,	//!< change from expressed WRT baselnk to WRT aircraft
	ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK,//!< change orientation from being expressed in aircraft frame to baselink frame in an absolute frame of reference.
	ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT,//!< change orientation from being expressed in baselink frame to aircraft frame in an absolute frame of reference
};

/**
 * @brief Orientation transform options when applying rotations to data, for ECEF.
 */
enum class StaticEcefTF {
	ECEF_TO_ENU,		//!< change from expressed WRT ECEF frame to WRT ENU frame
	ENU_TO_ECEF		//!< change from expressed WRT ENU frame to WRT ECEF frame
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
 * @brief Transform data expressed in one frame to another frame.
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Eigen::Vector3d transform_frame(const Eigen::Vector3d &vec, const Eigen::Quaterniond &q);

/**
 * @brief Transform 3x3 convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance3d transform_frame(const Covariance3d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform 6x6 convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance6d transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform 9x9 convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance9d transform_frame(const Covariance9d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform data expressed in one frame to another frame.
 *
 * General function. Please use specialized variants.
 */
Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const StaticTF transform);

/**
 * @brief Transform 3d convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance3d transform_static_frame(const Covariance3d &cov, const StaticTF transform);

/**
 * @brief Transform 6d convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance6d transform_static_frame(const Covariance6d &cov, const StaticTF transform);

/**
 * @brief Transform 9d convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance9d transform_static_frame(const Covariance9d &cov, const StaticTF transform);

/**
 * @brief Transform data expressed in one frame to another frame
 * with additional map origin parameter.
 *
 * General function. Please use specialized variants.
 */
Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const Eigen::Vector3d &map_origin, const StaticEcefTF transform);

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
 * @brief Transform from attitude represented WRT aircraft frame to
 *		  attitude represented WRT base_link frame, treating aircraft frame 
 *		  as in an absolute frame of reference (local NED).
 */
template<class T>
inline T transform_orientation_absolute_frame_aircraft_baselink(const T &in) {
	return detail::transform_orientation(in, StaticTF::ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform from attitude represented WRT baselink frame to
 *		  attitude represented WRT body frame, treating baselink frame 
 *		  as in an absolute frame of reference (local NED).
 */
template<class T>
inline T transform_orientation_absolute_frame_baselink_aircraft(const T &in) {
	return detail::transform_orientation(in, StaticTF::ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT);
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
 * @brief Transform data expressed in ECEF frame to ENU frame.
 *
 * @param in          local ECEF coordinates [m]
 * @param map_origin  geodetic origin [lla]
 * @returns local ENU coordinates [m].
 */
template<class T>
inline T transform_frame_ecef_enu(const T &in, const T &map_origin) {
	return detail::transform_static_frame(in, map_origin, StaticEcefTF::ECEF_TO_ENU);
}

/**
 * @brief Transform data expressed in ENU frame to ECEF frame.
 *
 * @param in          local ENU coordinates [m]
 * @param map_origin  geodetic origin [lla]
 * @returns local ECEF coordinates [m].
 */
template<class T>
inline T transform_frame_enu_ecef(const T &in, const T &map_origin) {
	return detail::transform_static_frame(in, map_origin, StaticEcefTF::ENU_TO_ECEF);
}

/**
 * @brief Transform data expressed in aircraft frame to NED frame.
 * Assumes quaternion represents rotation from aircraft frame to NED frame.
 */
template<class T>
inline T transform_frame_aircraft_ned(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in NED to aircraft frame.
 * Assumes quaternion represents rotation from NED to aircraft frame.
 */
template<class T>
inline T transform_frame_ned_aircraft(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in aircraft frame to ENU frame.
 * Assumes quaternion represents rotation from aircraft frame to ENU frame.
 */
template<class T>
inline T transform_frame_aircraft_enu(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in ENU to aircraft frame.
 * Assumes quaternion represents rotation from ENU to aircraft frame.
 */
template<class T>
inline T transform_frame_enu_aircraft(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in ENU to base_link frame.
 * Assumes quaternion represents rotation from ENU to base_link frame.
 */
template<class T>
inline T transform_frame_enu_baselink(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in baselink to ENU frame.
 * Assumes quaternion represents rotation from basel_link to ENU frame.
 */
template<class T>
inline T transform_frame_baselink_enu(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
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
template <typename _Scalar, typename std::enable_if<std::is_floating_point<_Scalar>::value, bool>::type = true>
inline void quaternion_to_mavlink(const Eigen::Quaternion<_Scalar> &q, std::array<float, 4> &qmsg)
{
	qmsg[0] = q.w();
	qmsg[1] = q.x();
	qmsg[2] = q.y();
	qmsg[3] = q.z();
}

/**
 * @brief Convert Mavlink float[4] quaternion to Eigen
 */
inline Eigen::Quaterniond mavlink_to_quaternion(const std::array<float, 4> &q)
{
	return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
}

/**
 * @brief Convert covariance matrix to MAVLink float[n] format
 */
template<class T, std::size_t SIZE>
inline void covariance_to_mavlink(const T &cov, std::array<float, SIZE> &covmsg)
{
	std::copy(cov.cbegin(), cov.cend(), covmsg.begin());
}

/**
 * @brief Convert upper right triangular of a covariance matrix to MAVLink float[n] format
 */
template<class T, std::size_t ARR_SIZE>
inline void covariance_urt_to_mavlink(const T &covmap, std::array<float, ARR_SIZE> &covmsg)
{
	auto m = covmap;
	std::size_t COV_SIZE = m.rows() * (m.rows() + 1) / 2;
	ROS_ASSERT_MSG(COV_SIZE == ARR_SIZE,
				"frame_tf: covariance matrix URT size (%lu) is different from Mavlink msg covariance field size (%lu)",
				COV_SIZE, ARR_SIZE);

	auto out = covmsg.begin();

	for (size_t x = 0; x < m.cols(); x++) {
		for (size_t y = x; y < m.rows(); y++)
			*out++ = m(y, x);
	}
}

/**
 * @brief Convert MAVLink float[n] format (upper right triangular of a covariance matrix)
 * to Eigen::MatrixXd<n,n> full covariance matrix
 */
template<class T, std::size_t ARR_SIZE>
inline void mavlink_urt_to_covariance_matrix(const std::array<float, ARR_SIZE> &covmsg, T &covmat)
{
	std::size_t COV_SIZE = covmat.rows() * (covmat.rows() + 1) / 2;
	ROS_ASSERT_MSG(COV_SIZE == ARR_SIZE,
				"frame_tf: covariance matrix URT size (%lu) is different from Mavlink msg covariance field size (%lu)",
				COV_SIZE, ARR_SIZE);

	auto in = covmsg.begin();

	for (size_t x = 0; x < covmat.cols(); x++) {
		for (size_t y = x; y < covmat.rows(); y++) {
			covmat(x, y) = static_cast<double>(*in++);
			covmat(y, x) = covmat(x, y);
		}
	}
}

// [[[cog:
// def make_to_eigen(te, tr, fields):
//     cog.outl("""//! @brief Helper to convert common ROS geometry_msgs::{tr} to Eigen::{te}""".format(**locals()))
//     cog.outl("""inline Eigen::{te} to_eigen(const geometry_msgs::{tr} r) {{""".format(**locals()))
//     cog.outl("""\treturn Eigen::{te}({fl});""".format(te=te, fl=", ".join(["r." + f for f in fields])))
//     cog.outl("""}""")
//
// make_to_eigen("Vector3d", "Point", "xyz")
// make_to_eigen("Vector3d", "Vector3", "xyz")
// make_to_eigen("Quaterniond", "Quaternion", "wxyz")
// ]]]
//! @brief Helper to convert common ROS geometry_msgs::Point to Eigen::Vector3d
inline Eigen::Vector3d to_eigen(const geometry_msgs::Point r) {
	return Eigen::Vector3d(r.x, r.y, r.z);
}
//! @brief Helper to convert common ROS geometry_msgs::Vector3 to Eigen::Vector3d
inline Eigen::Vector3d to_eigen(const geometry_msgs::Vector3 r) {
	return Eigen::Vector3d(r.x, r.y, r.z);
}
//! @brief Helper to convert common ROS geometry_msgs::Quaternion to Eigen::Quaterniond
inline Eigen::Quaterniond to_eigen(const geometry_msgs::Quaternion r) {
	return Eigen::Quaterniond(r.w, r.x, r.y, r.z);
}
// [[[end]]] (checksum: 1b3ada1c4245d4e31dcae9768779b952)
}	// namespace ftf
}	// namespace mavros
