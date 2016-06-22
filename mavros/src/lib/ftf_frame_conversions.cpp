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

// Static quaternion needed for rotating between ENU and NED frames
// +PI rotation around X (North) axis follwed by +PI/2 rotation about Z (Down)
// gives the ENU frame.  Similarly, a +PI rotation about X (East) followed by
// a +PI/2 roation about Z (Up) gives the NED frame.
static const Eigen::Quaterniond NED_ENU_Q = quaternion_from_rpy(M_PI, 0.0, M_PI_2);

// Static quaternion needed for rotating between aircraft and base_link frames
// +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
// Fto Forward, Left, Up (base_link) frames.
static const Eigen::Quaterniond AIRCRAFT_BASELINK_Q = quaternion_from_rpy(M_PI, 0.0, 0.0);

static const Eigen::Affine3d NED_ENU_AFFINE(NED_ENU_Q);
static const Eigen::Affine3d AIRCRAFT_BASELINK_AFFINE(AIRCRAFT_BASELINK_Q);


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
		return NED_ENU_AFFINE * vec;

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
		cov_out = cov_in * NED_ENU_Q;
		return cov_out_;

	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		cov_out = cov_in * AIRCRAFT_BASELINK_Q;
		return cov_out_;
	}
}

#if 0
Covariance6d transform_static_frame(const Covariance6d &cov, const StaticTF transform)
{
	//! @todo implement me!!!
	switch (transform) {
	default: {
		Covariance6d cov_out_;
		EigenMapConstCovariance6d cov_in(cov.data());
		EigenMapCovariance6d cov_out(cov_out_.data());
		ROS_BREAK();
		return cov_out_;
	}
	}
}
#endif

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

#if 0
Covariance6d transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q)
{
	Covariance6d cov_out_;
	EigenMapConstCovariance6d cov_in(cov.data());
	EigenMapCovariance6d cov_out(cov_out_.data());

	//! @todo implement me!!!
	ROS_BREAK();
	return cov_out_;
}
#endif

}	// namespace detail
}	// namespace ftf
}	// namespace mavros
