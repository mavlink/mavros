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
	// YPR - ZYX
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

