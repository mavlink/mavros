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

#pragma once

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
	 * @brief Sets up the PID values for computation of the linear velocities effort
	 */
	void setup_linvel_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min, const ros::NodeHandle &node);

	/**
	 * @brief Sets up the PID values for computation of the yaw rate effort
	 */
	void setup_yawrate_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min, const ros::NodeHandle &node);

	/* *-* Effort computation *-* */
	/**
	 * @brief Computes the linear velocity effort to apply to each axis
	 */
	Eigen::Vector3d compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time);

	/**
	 * @brief Computes the yaw rate effort to apply
	 */
	double compute_yawrate_effort(double goal, double current, ros::Time last_time);

private:
	// Control toolbox PID controllers
	control_toolbox::Pid pid_linvel_x;
	control_toolbox::Pid pid_linvel_y;
	control_toolbox::Pid pid_linvel_z;
	control_toolbox::Pid pid_yaw_rate;

};
};	// namespace pidcontroller
