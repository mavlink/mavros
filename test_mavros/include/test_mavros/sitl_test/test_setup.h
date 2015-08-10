/**
 * @brief SITL tests setup
 * @file test_setup.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup sitl_test
 * @{
 *  @brief SITL tests setup
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <test_mavros/utils/pid_controller.h>

namespace testsetup {
class TestSetup {
public:
	TestSetup() :
		nh("~")
	{ };
	~TestSetup() {};

	ros::NodeHandle nh;

	bool use_pid;
	double rate;
	int num_of_tests;

	void setup(const ros::NodeHandle &nh){
		nh.param("use_pid", use_pid, true);
		nh.param("rate", rate, 10.0);
		nh.param("num_of_tests", num_of_tests, 10);
	}
};
};	// namespace testsetup
