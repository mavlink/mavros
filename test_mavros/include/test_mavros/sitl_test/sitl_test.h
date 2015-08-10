/**
 * @brief SitlTest node implementation class
 * @file sitl_test.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup sitl_test
 * @{
 *  @brief SITL test node implementation
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <ros/ros.h>
#include <test_mavros/sitl_test/test_setup.h>
#include <test_mavros/tests/offboard_control.h>

namespace sitltest {
/**
 * @brief SITL test node class
 *
 * This class implements sitl_test_node
 */
class SitlTest
{
public:
	SitlTest();
	~SitlTest() {};

	static void spin(int argc, char *argv[]);
};
};	// namespace sitltest
