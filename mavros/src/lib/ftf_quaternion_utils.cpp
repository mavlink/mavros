/**
 * @brief Eigen::Quaternion helter functions
 * @file uas_quaternion_utils.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <mavros/frame_tf.h>

namespace mavros {
namespace ftf {

/*
 * Note: order of axis are match tf2::LinearMath (bullet).
 * YPR rotation convention -> YAW first, Pitch second, Roll third
 * Compatibility checked by unittests.
 */

Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
	// YPR - ZYX
	return Eigen::Quaterniond(
			Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
			);
}

Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q)
{
	// YPR - ZYX
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

double quaternion_get_yaw(const Eigen::Quaterniond &q)
{
	// to match equation from:
	// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	const double &q0 = q.w();
	const double &q1 = q.x();
	const double &q2 = q.y();
	const double &q3 = q.z();

	return std::atan2(2. * (q0*q3 + q1*q2), 1. - 2. * (q2*q2 + q3*q3));
}

}	// namesapace ftf
}	// namesapace mavros
