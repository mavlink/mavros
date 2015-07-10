/**
 * @brief SITL test class
 * @file sitl_test.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros.h>
#include <sitl_test/sitl_test.h>
#include <sitl_test/test_type.h>
#include <ros/console.h>

using namespace sitltest;
using namespace testtype;

SitlTest::SitlTest()
{}

void SitlTest::spin(int argc, char *argv[])
{	
	if (strcmp(argv[1],"offboard_posctl_square") == 0)
	{
		ros::init(argc, argv, "offboard_posctl_square");
		testtype::OffboardPosCtrlSquare offboard_posctl_square;
		offboard_posctl_square.spin(argc, argv);
	}

	/** @todo add more testing structures */
}
