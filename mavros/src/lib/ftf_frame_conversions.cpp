/**
 * @brief Frame conversions helper functions
 * @file uas_frame_conversions.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 * Copyright 2016 Vladimir Ermakov
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/frame_tf.h>

namespace mavros {
namespace ftf {
namespace detail {
/**
 * @brief Static quaternion needed for rotating between ENU and NED frames
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 */
static const auto NED_ENU_Q = quaternion_from_rpy(M_PI, 0.0, M_PI_2);

/**
 * @brief Static quaternion needed for rotating between aircraft and base_link frames
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
 * Fto Forward, Left, Up (base_link) frames.
 */
static const auto AIRCRAFT_BASELINK_Q = quaternion_from_rpy(M_PI, 0.0, 0.0);

/**
 * @brief Static vector needed for rotating between ENU and NED frames
 * +PI rotation around X (North) axis follwed by +PI/2 rotation about Z (Down)
 * gives the ENU frame.  Similarly, a +PI rotation about X (East) followed by
 * a +PI/2 roation about Z (Up) gives the NED frame.
 */
static const Eigen::Affine3d NED_ENU_AFFINE(NED_ENU_Q);

/**
 * @brief Static vector needed for rotating between aircraft and base_link frames
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
 * Fto Forward, Left, Up (base_link) frames.
 */
static const Eigen::Affine3d AIRCRAFT_BASELINK_AFFINE(AIRCRAFT_BASELINK_Q);

/**
 * @brief 3-D matrices to fill 6-D rotation matrix applied to change covariance matrices coordinate frames
 */
static const auto NED_ENU_R = NED_ENU_Q.normalized().toRotationMatrix();
static const auto AIRCRAFT_BASELINK_R = AIRCRAFT_BASELINK_Q.normalized().toRotationMatrix();

/**
 * @brief Use reflections instead of rotations for NED <-> ENU transformation
 * to avoid NaN/Inf floating point pollution across different axes
 * since in NED <-> ENU the axes are perfectly aligned.
 */
static const Eigen::PermutationMatrix<3> NED_ENU_REFLECTION_XY(Eigen::Vector3i(1,0,2));
static const Eigen::DiagonalMatrix<double,3> NED_ENU_REFLECTION_Z(1,1,-1);

/**
 * @brief Auxiliar matrices to Covariance transforms
 */
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix9d = Eigen::Matrix<double, 9, 9>;


Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform)
{
	// Transform the attitude representation from frame to frame.
	// The proof for this transform can be seen
	// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
		return NED_ENU_Q * q;

	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		return q * AIRCRAFT_BASELINK_Q;
	}
}


Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const StaticTF transform)
{
	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
		return NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z * vec);

	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		return AIRCRAFT_BASELINK_AFFINE * vec;
	}
}

Covariance3d transform_static_frame(const Covariance3d &cov, const StaticTF transform)
{
	Covariance3d cov_out_;
	EigenMapConstCovariance3d cov_in(cov.data());
	EigenMapCovariance3d cov_out(cov_out_.data());

	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
		cov_out = NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z * cov_in * NED_ENU_REFLECTION_Z) *
		          NED_ENU_REFLECTION_XY.transpose();
		return cov_out_;

	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		cov_out = cov_in * AIRCRAFT_BASELINK_Q;
		return cov_out_;
	}
}

Covariance6d transform_static_frame(const Covariance6d &cov, const StaticTF transform)
{
	Covariance6d cov_out_;
	Matrix6d R = Matrix6d::Zero();	// not `auto` because Zero ret is const

	EigenMapConstCovariance6d cov_in(cov.data());
	EigenMapCovariance6d cov_out(cov_out_.data());

	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
	{
		Eigen::PermutationMatrix<6> NED_ENU_REFLECTION_XY_6 (NED_ENU_REFLECTION_XY.indices().replicate<2,1>());
		NED_ENU_REFLECTION_XY_6.indices().middleRows<3>(3).array() += 3;
		Eigen::DiagonalMatrix<double,6> NED_ENU_REFLECTION_Z_6(NED_ENU_REFLECTION_Z.diagonal().replicate<2,1>());

		cov_out = NED_ENU_REFLECTION_XY_6 * (NED_ENU_REFLECTION_Z_6 * cov_in * NED_ENU_REFLECTION_Z_6) *
		          NED_ENU_REFLECTION_XY_6.transpose();

		return cov_out_;
        }
	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		R.block<3, 3>(0, 0) =
			R.block<3, 3>(3, 3) = AIRCRAFT_BASELINK_R;

		cov_out = R * cov_in * R.transpose();
		return cov_out_;
	}
}

Covariance9d transform_static_frame(const Covariance9d &cov, const StaticTF transform)
{
	Covariance9d cov_out_;
	Matrix9d R = Matrix9d::Zero();

	EigenMapConstCovariance9d cov_in(cov.data());
	EigenMapCovariance9d cov_out(cov_out_.data());

	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
	{
		Eigen::PermutationMatrix<9> NED_ENU_REFLECTION_XY_9 (NED_ENU_REFLECTION_XY.indices().replicate<3,1>());
		NED_ENU_REFLECTION_XY_9.indices().middleRows<3>(3).array() += 3;
		NED_ENU_REFLECTION_XY_9.indices().middleRows<3>(6).array() += 6;
		Eigen::DiagonalMatrix<double,9> NED_ENU_REFLECTION_Z_9(NED_ENU_REFLECTION_Z.diagonal().replicate<3,1>());

		cov_out = NED_ENU_REFLECTION_XY_9 * (NED_ENU_REFLECTION_Z_9 * cov_in * NED_ENU_REFLECTION_Z_9) *
		          NED_ENU_REFLECTION_XY_9.transpose();

		return cov_out_;
        }
	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		R.block<3, 3>(0, 0) =
			R.block<3, 3>(3, 3) =
				R.block<3, 3>(6, 6) = AIRCRAFT_BASELINK_R;

		cov_out = R * cov_in * R.transpose();
		return cov_out_;
	}
}

Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const Eigen::Vector3d &map_origin, const StaticTF transform)
{
	//! Degrees to radians
	static constexpr double DEG_TO_RAD = (M_PI / 180.0);

	// Don't forget to convert from degrees to radians
	const double sin_lat = std::sin(map_origin.x() * DEG_TO_RAD);
	const double sin_lon = std::sin(map_origin.y() * DEG_TO_RAD);
	const double cos_lat = std::cos(map_origin.x() * DEG_TO_RAD);
	const double cos_lon = std::cos(map_origin.y() * DEG_TO_RAD);

	/**
	 * @brief Compute transform from ECEF to ENU:
	 * http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
	 * ϕ = latitude
	 * λ = longitude
	 * The rotation is composed by a counter-clockwise rotation over the Z-axis
	 * by an angle of 90 + λ followed by a counter-clockwise rotation over the east-axis by
	 * an angle of 90 - ϕ.
	 * R = [-sinλ         cosλ         0.0
	 *      -cosλ*sinϕ   -sinλ*sinϕ    cosϕ
	 *       cosλ*cosϕ    sinλ*cosϕ    sinϕ   ]
	 * [East, North, Up] = R * [∂x, ∂y, ∂z]
	 * where both [East, North, Up] and [∂x, ∂y, ∂z] are local coordinates relative to map origin.
	 */
	Eigen::Matrix3d R;
	R << -sin_lon,            cos_lon,           0.0,
	     -cos_lon * sin_lat, -sin_lon * sin_lat, cos_lat,
	      cos_lon * cos_lat,  sin_lon * cos_lat, sin_lat;

	switch (transform) {
	case StaticTF::ECEF_TO_ENU:
		return R * vec;

	case StaticTF::ENU_TO_ECEF:
		// ENU to ECEF rotation is just an inverse rotation from ECEF to ENU, which means transpose.
		R.transposeInPlace();
		return R * vec;
	}
}

Eigen::Vector3d transform_frame(const Eigen::Vector3d &vec, const Eigen::Quaterniond &q)
{
	Eigen::Affine3d transformation(q);
	return transformation * vec;
}

Covariance3d transform_frame(const Covariance3d &cov, const Eigen::Quaterniond &q)
{
	Covariance3d cov_out_;
	EigenMapConstCovariance3d cov_in(cov.data());
	EigenMapCovariance3d cov_out(cov_out_.data());

	cov_out = cov_in * q;
	return cov_out_;
}

Covariance6d transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q)
{
	Covariance6d cov_out_;
	Matrix6d R = Matrix6d::Zero();

	EigenMapConstCovariance6d cov_in(cov.data());
	EigenMapCovariance6d cov_out(cov_out_.data());

	R.block<3, 3>(0, 0) =
		R.block<3, 3>(3, 3) = q.normalized().toRotationMatrix();

	cov_out = R * cov_in * R.transpose();
	return cov_out_;
}

Covariance9d transform_frame(const Covariance9d &cov, const Eigen::Quaterniond &q)
{
	Covariance9d cov_out_;
	Matrix9d R = Matrix9d::Zero();

	EigenMapConstCovariance9d cov_in(cov.data());
	EigenMapCovariance9d cov_out(cov_out_.data());

	R.block<3, 3>(0, 0) =
		R.block<3, 3>(3, 3) =
			R.block<3, 3>(6, 6) = q.normalized().toRotationMatrix();

	cov_out = R * cov_in * R.transpose();
	return cov_out_;
}

}	// namespace detail
}	// namespace ftf
}	// namespace mavros
