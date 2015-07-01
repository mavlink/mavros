/**
 * @brief Frame conversions helper functions
 * @file uas_frame_conversions.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
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
#include <boost/math/constants/constants.hpp>

using namespace mavros;


tf::Vector3 UAS::transform_frame_xyz(double _x, double _y, double _z)
{
	double x =  _x;
	double y = -_y;
	double z = -_z;
	return tf::Vector3(x, y, z);
}

tf::Quaternion UAS::transform_frame_attitude_q(tf::Quaternion qo)
{
	// XXX temporary
	// same reason as for rpy: return old math.
#if 0
	double roll = M_PI, pitch = 0.0, yaw = 0.0;
	tf::Quaternion qr = tf::createQuaternionFromRPY(roll, pitch, yaw);
	tf::Quaternion qt = qo * qr;
	return qt;
#endif
	return tf::Quaternion(qo.x(), -qo.y(), -qo.z(), qo.w());
}

tf::Vector3 UAS::transform_frame_attitude_rpy(double _roll, double _pitch, double _yaw)
{
	// XXX temporary comment-out new method.
	// hand-test in rviz + imu plugin show wrong rotation direction on yaw.
	// APM Planner and MAVProxy shows CW, RVIZ CCW => mavros bug.
	// return old math.
#if 0
	double roll = _roll + M_PI;
	double pitch = _pitch;
	double yaw = _yaw;
	return tf::Vector3(roll, pitch, yaw);
#endif
	return transform_frame_xyz(_roll, _pitch, _yaw);
}

UAS::Covariance6x6 UAS::transform_frame_covariance_pose6x6(UAS::Covariance6x6 &_covariance)
{
	const UAS::Covariance6x6 rotation = {
		1,  0,  0,  0,  0,  0,
		0, -1,  0,  0,  0,  0,
		0,  0, -1,  0,  0,  0,
		0,  0,  0,  1,  0,  0,
		0,  0,  0,  0, -1,  0,
		0,  0,  0,  0,  0, -1
	};

	UAS::Covariance6x6 covariance;
	UAS::Covariance6x6 temp;		// temporary matrix = T * C

	// The transformation matrix in this case is a orthogonal matrix so T = T^t

	/**
	 * @note According to ROS convention, if one has no estimate for one of the data elements,
	 * element 0 of the associated covariance matrix to is -1; So no transformation has be applied,
	 * as the covariance is invalid/unknown; so, it returns the same cov matrix without transformation.
	 */
	if (_covariance.at(0) != -1) {
		// XXX this doesn't multiply matrices correctly. We need Eigen on code!
		std::transform(rotation.begin(), rotation.end(), _covariance.begin(), temp.begin(), std::multiplies<double>());
		std::transform(temp.begin(), temp.end(), rotation.begin(), covariance.begin(), std::multiplies<double>());
		return covariance;
	}
	else {
		_covariance.at(0) = -1;
		return _covariance;
	}
}

UAS::Covariance3x3 UAS::transform_frame_covariance_general3x3(UAS::Covariance3x3 &_covariance)
{
	const UAS::Covariance3x3 rotation = {
		1,  0,  0,
		0, -1,  0,
		0,  0, -1
	};

	UAS::Covariance3x3 covariance;
	UAS::Covariance3x3 temp;		// temporary matrix = T * C

	// The transformation matrix in this case is a orthogonal matrix so T = T^t

	/**
	 * @note According to ROS convention, if one has no estimate for one of the data elements,
	 * element 0 of the associated covariance matrix to is -1; So no transformation has be applied,
	 * as the covariance is invalid/unknown; so, it returns the same cov matrix without transformation.
	 */
	if (_covariance.at(0) != -1) {
		// XXX this doesn't multiply matrices correctly. We need Eigen on code!
		std::transform(rotation.begin(), rotation.end(), _covariance.begin(), temp.begin(), std::multiplies<double>());
		std::transform(temp.begin(), temp.end(), rotation.begin(), covariance.begin(), std::multiplies<double>());
		return covariance;
	}
	else {
		return _covariance;
	}
}
