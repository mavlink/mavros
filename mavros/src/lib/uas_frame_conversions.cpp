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

static tf::Vector3 transform_frame_general_xyz(float _x, float _y, float _z){
	float x =  _x;
	float y = -_y;
	float z = -_z;
	return tf::Vector3(x, y, z);
};

static tf::Quaternion transform_frame_attitude_q(tf::Quaternion qo){
	double roll = PI, pitch = 0.0 , yaw = 0.0f; 
	tf::Quaternion qr = tf::createQuaternionFromRPY(roll, pitch, yaw);
	tf::Quaternion qt = qo * qr; 
	return qt;
};

static tf::Vector3 transform_frame_attitude_rpy(float _roll, float _pitch, float _yaw){
	float roll = _roll + PI;
	float pitch = _pitch;
	float yaw = _yaw;
	return tf::Vector3(roll, pitch, yaw);
};

static std::array<float, 36> transform_frame_covariance_pose6x6(std::array<float, 36> _covariance){

	std::array<float, 36> rotation = {1 ,0,0,0,0,0,
    					  0,-1,0,0,0,0,
    					  0,0,-1,0,0,0,
    					  0,0,0, 1,0,0,
    					  0,0,0,0,-1,0,
    					  0,0,0,0,0,-1};

   	std::array<float, 36> covariance;							
	std::array<float, 36> temp;		// temporary matrix = R * C

	// The rotation matrix in this case is a diagonal matrix so R = R^T
	
	std::transform(rotation.begin()+1, rotation.end(), _covariance.begin()+1, temp.begin(), std::multiplies<float>());
	std::transform(temp.begin()+1, temp.end(), rotation.begin()+1, covariance.begin(), std::multiplies<float>());

	return covariance;
};

static std::array<float, 9> transform_frame_covariance_general3x3(std::array<float, 9> _covariance){

	std::array<float, 9> rotation = {1 ,0,0,
    					 0,-1,0,
    					 0,0,-1};

   	std::array<float, 9> covariance;							
	std::array<float, 9> temp;		// temporary matrix = R * C

	// The rotation matrix in this case is a diagonal matrix so R = R^T
	
	std::transform(rotation.begin()+1, rotation.end(), _covariance.begin()+1, temp.begin(), std::multiplies<float>());
	std::transform(temp.begin()+1, temp.end(), rotation.begin()+1, covariance.begin(), std::multiplies<float>());

	return covariance;
};
