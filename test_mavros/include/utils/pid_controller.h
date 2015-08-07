/**
 * @brief PID controller header
 * @file pid_controller.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup test_utils
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
#include <control_toolbox/pid.h>
#include <eigen_conversions/eigen_msg.h>

namespace pidcontroller {
class PIDController
{
public:
	PIDController();
	~PIDController() {};

	/* *-* PID Setup *-* */
	/**
	 * @brief
	 */
	static void setup_linvel_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min, const ros::NodeHandle &node);

	/**
	 * @brief
	 */
	static void setup_yawrate_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min, const ros::NodeHandle &node);

	/* *-* Effort computation *-* */
	/**
	 * @brief
	 */
	static Eigen::Vector3d compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time);

	/**
	 * @brief
	 */
	static double compute_yawrate_effort(double goal, double current, ros::Time last_time);
};
};	// namespace pidcontroller
