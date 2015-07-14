/**
 * @brief SITL test type
 * @file test_type.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup sitl_test
 * @{
 *  @brief SITL tests system
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <tests/offboard_control.h>

namespace testtype {
class TestType {
public:
	virtual ~TestType() {};

private:
	/*
	 * TODO: Here we can implement test specifities, conditions, others
	 */

protected:
	TestType() {};
};
};	// namespace testtype
