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
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <array>
#include <mavros/mavros_uas.h>

using namespace mavros;

// Static quaternion needed for rotating between ENU and NED frames
// +PI rotation around X (North) axis follwed by +PI/2 rotation about Z (Down)
// gives the ENU frame.  Similarly, a +PI rotation about X (East) followed by
// a +PI/2 roation about Z (Up) gives the NED frame.
static const Eigen::Quaterniond NED_ENU_Q = UAS::quaternion_from_rpy(M_PI, 0.0, M_PI_2);

// Static quaternion needed for rotating between aircraft and base_link frames
// +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
// Fto Forward, Left, Up (base_link) frames.
static const Eigen::Quaterniond AIRCRAFT_BASELINK_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);

static const Eigen::Affine3d NED_ENU_AFFINE(NED_ENU_Q);
static const Eigen::Affine3d AIRCRAFT_BASELINK_AFFINE(AIRCRAFT_BASELINK_Q);

static inline const Eigen::Affine3d &get_static_transform_affine(const UAS::STATIC_TRANSFORM transform)
{
	switch (transform) {
	case UAS::STATIC_TRANSFORM::NED_TO_ENU:
	case UAS::STATIC_TRANSFORM::ENU_TO_NED: {
		return NED_ENU_AFFINE;
		break;
	}
	case UAS::STATIC_TRANSFORM::AIRCRAFT_TO_BASELINK:
	case UAS::STATIC_TRANSFORM::BASELINK_TO_AIRCRAFT: {
		return AIRCRAFT_BASELINK_AFFINE;
		break;
	}
	default: {
		ROS_BREAK();
	}
	}
}


//! Transform for vector3
//static const Eigen::Transform<double, 3, Eigen::Affine> FRAME_TRANSFORM_VECTOR3(FRAME_ROTATE_Q);


Eigen::Quaterniond UAS::transform_orientation(const Eigen::Quaterniond &q, const UAS::STATIC_TRANSFORM transform)
{
	//Transform the attitude representation from frame to frame.  The proof for this transform can
	//be seen http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
	switch (transform) {
	case STATIC_TRANSFORM::NED_TO_ENU:
	case STATIC_TRANSFORM::ENU_TO_NED: {
		return NED_ENU_Q * q;
		break;
	}
	case STATIC_TRANSFORM::AIRCRAFT_TO_BASELINK:
	case STATIC_TRANSFORM::BASELINK_TO_AIRCRAFT: {
		return q * AIRCRAFT_BASELINK_Q;
		break;
	}
	default: {
		//We don't know how to express the attitude WRT an undefined
		//frame.
		ROS_BREAK();
		return q;
		break;
	}
	}
}

Eigen::Vector3d UAS::transform_static_frame(const Eigen::Vector3d &vec, const UAS::STATIC_TRANSFORM transform)
{
	auto affine_transform = get_static_transform_affine(transform);
	return affine_transform * vec;
}

UAS::Covariance3d UAS::transform_static_frame(const Covariance3d &cov, const UAS::STATIC_TRANSFORM transform)
{
	Covariance3d cov_out_;
	EigenMapConstCovariance3d cov_in(cov.data());
	EigenMapCovariance3d cov_out(cov_out_.data());
	switch (transform) {
	case STATIC_TRANSFORM::NED_TO_ENU:
	case STATIC_TRANSFORM::ENU_TO_NED: {
		// code from imu_transformer tf2_sensor_msgs.h
		//cov_out = FRAME_ROTATE_Q * cov_in * FRAME_ROTATE_Q.inverse();
		// from comments on github about tf2_sensor_msgs.h
		cov_out = cov_in * NED_ENU_Q;
		return cov_out_;
		break;
	}
	case STATIC_TRANSFORM::AIRCRAFT_TO_BASELINK:
	case STATIC_TRANSFORM::BASELINK_TO_AIRCRAFT: {
		// code from imu_transformer tf2_sensor_msgs.h
		//cov_out = FRAME_ROTATE_Q * cov_in * FRAME_ROTATE_Q.inverse();
		// from comments on github about tf2_sensor_msgs.h
		cov_out = cov_in * AIRCRAFT_BASELINK_Q;
		return cov_out_;
		break;
	}
	default: {
		//We don't know the transform between the static frames.
		//throw error and return given covariance
		ROS_BREAK();
		return cov;
		break;
	}
	}
}

UAS::Covariance6d UAS::transform_static_frame(const Covariance6d &cov, const UAS::STATIC_TRANSFORM transform)
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

Eigen::Vector3d UAS::transform_frame(const Eigen::Vector3d &vec, const Eigen::Quaterniond &q)
{
	Eigen::Affine3d FRAME_TRANSFORM_VECTOR3(q);
	return FRAME_TRANSFORM_VECTOR3 * vec;
}

UAS::Covariance3d UAS::transform_frame(const Covariance3d &cov, const Eigen::Quaterniond &q)
{
	Covariance3d cov_out_;
	EigenMapConstCovariance3d cov_in(cov.data());
	EigenMapCovariance3d cov_out(cov_out_.data());

	// code from imu_transformer tf2_sensor_msgs.h
	//cov_out = FRAME_ROTATE_Q * cov_in * FRAME_ROTATE_Q.inverse();
	// from comments on github about tf2_sensor_msgs.h
	cov_out = cov_in * q;
	return cov_out_;
}

UAS::Covariance6d UAS::transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q)
{
	Covariance6d cov_out_;
	EigenMapConstCovariance6d cov_in(cov.data());
	EigenMapCovariance6d cov_out(cov_out_.data());

	//! @todo implement me!!!
	ROS_BREAK();
	return cov_out_;
}