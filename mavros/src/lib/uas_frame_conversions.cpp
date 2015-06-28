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

const double PI = boost::math::constants::pi<double>();

using namespace mavros;

tf::Vector3 UAS::transform_frame_xyz(double _x, double _y, double _z){
	double x =  _x;
	double y = -_y;
	double z = -_z;
	return tf::Vector3(x, y, z);
};

tf::Quaternion UAS::transform_frame_attitude_q(tf::Quaternion qo){
	double roll = PI, pitch = 0.0 , yaw = 0.0f; 
	tf::Quaternion qr = tf::createQuaternionFromRPY(roll, pitch, yaw);
	tf::Quaternion qt = qo * qr; 
	return qt;
};

tf::Vector3 UAS::transform_frame_attitude_rpy(double _roll, double _pitch, double _yaw){
	double roll = _roll + PI;
	double pitch = _pitch;
	double yaw = _yaw;
	return tf::Vector3(roll, pitch, yaw);
};

UAS::Covariance6x6 UAS::transform_frame_covariance_pose6x6(UAS::Covariance6x6 &_covariance){

	UAS::Covariance6x6 rotation = { 1 ,0,0,0,0,0,
    					0,-1,0,0,0,0,
    					0,0,-1,0,0,0,
    					0,0,0, 1,0,0,
    					0,0,0,0,-1,0,
    					0,0,0,0,0,-1};

   	UAS::Covariance6x6 covariance;							
	UAS::Covariance6x6 temp;		// temporary matrix = R * C

	// The rotation matrix in this case is a diagonal matrix so R = R^T
	
	std::transform(rotation.begin(), rotation.end(), _covariance.begin(), temp.begin(), std::multiplies<double>());
	std::transform(temp.begin(), temp.end(), rotation.begin(), covariance.begin(), std::multiplies<double>());

	return covariance;
};

UAS::Covariance3x3 UAS::transform_frame_covariance_general3x3(UAS::Covariance3x3 &_covariance){

	UAS::Covariance3x3 rotation = { 1 ,0,0,
    					0,-1,0,
    					0,0,-1};

   	UAS::Covariance3x3 covariance;							
	UAS::Covariance3x3 temp;		// temporary matrix = R * C

	// The rotation matrix in this case is a diagonal matrix so R = R^T
	
	std::transform(rotation.begin(), rotation.end(), _covariance.begin(), temp.begin(), std::multiplies<double>());
	std::transform(temp.begin(), temp.end(), rotation.begin(), covariance.begin(), std::multiplies<double>());

	return covariance;
};
