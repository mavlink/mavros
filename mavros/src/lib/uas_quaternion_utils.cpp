/**
 * @brief Eigen::Quaternion helter functions
 * @file uas_quaternion_utils.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <mavros/mavros_uas.h>

using namespace mavros;

/*
 * Note: order of axis are match tf2::LinearMath (bullet).
 * Compatibility checked by unittests.
 */

Eigen::Quaterniond UAS::quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
	// YPR - ZYX
	return Eigen::Quaterniond(
			Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
			);
}

Eigen::Vector3d UAS::quaternion_to_rpy(const Eigen::Quaterniond &q)
{
	// Following the example of Vladimir Ermakov, I use the equations from this wikipedia article
	// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	// This function is injective for the range: yaw in [-pi,pi], pitch in [-pi/2,+pi/2] and roll in [-pi,+pi].
	// Note that the Eigen implementation is not good for YPR, because of the output range, which would be:
	//      [code using Eigen EulerAngles] rpy = q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
	//    [in docs    Eigen EulerAngles] valid range of EulerAngles: yaw, pitch, roll in the ranges [0:pi]x[-pi:pi]x[-pi:pi]
	//    see: https://eigen.tuxfamily.org/dox/group__Geometry__Module.html#gac3d90b12b21e1aaa2a9de9b0e45e6b7c

	const double &qw = q.w();
	const double &qx = q.x();
	const double &qy = q.y();
	const double &qz = q.z();

	Eigen::Vector3d rpy;
	/* roll  */ rpy.x() = std::atan2(2. * (qw * qx + qy * qz), 1. - 2. * (qx * qx + qy * qy));
	double sin_pitch = 2. * (qw * qy - qz * qx);
	sin_pitch = sin_pitch >  1.0 ?  1.0 : sin_pitch;
	sin_pitch = sin_pitch < -1.0 ? -1.0 : sin_pitch;
	/* pitch */ rpy.y() = std::asin( sin_pitch  );
	/* yaw   */ rpy.z() = std::atan2(2. * (qw * qz + qx * qy), 1. - 2. * (qy * qy + qz * qz));
	return rpy;
}

double UAS::quaternion_get_yaw(const Eigen::Quaterniond &q)
{
	// to match equation from:
	// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	const double &q0 = q.w();
	const double &q1 = q.x();
	const double &q2 = q.y();
	const double &q3 = q.z();

	return std::atan2(2. * (q0*q3 + q1*q2), 1. - 2. * (q2*q2 + q3*q3));
}
