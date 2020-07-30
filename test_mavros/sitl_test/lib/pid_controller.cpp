/**
 * @brief PID controller class
 * @file pid_controller.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <test_mavros/utils/pid_controller.h>

using namespace pidcontroller;

PIDController::PIDController()
{};

void PIDController::setup_linvel_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min, const ros::NodeHandle &node){
	// PID values
	std::array<double, 3> linvel_pid = { {p_gain, i_gain, d_gain} };

	// Min/max bounds for the integral windup
	double linvel_imin = i_min;
	double linvel_imax = i_max;

#ifdef CONTROL_TOOLBOX_PRE_1_14
	pid_linvel_x.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, node);
	pid_linvel_y.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, node);
	pid_linvel_z.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, node);
#else
	pid_linvel_x.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, false, node);
	pid_linvel_y.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, false, node);
	pid_linvel_z.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, false, node);
#endif
}

void PIDController::setup_yawrate_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min, const ros::NodeHandle &node){
	// PID values
	std::array<double, 3> yawrate_pid = { {p_gain, i_gain, d_gain} };

	// Min/max bounds for the integral windup
	double yawrate_imin = i_min;
	double yawrate_imax = i_max;

#ifdef CONTROL_TOOLBOX_PRE_1_14
	pid_yaw_rate.initPid(yawrate_pid[0], yawrate_pid[1], yawrate_pid[2], yawrate_imax, yawrate_imin, node);
#else
	pid_yaw_rate.initPid(yawrate_pid[0], yawrate_pid[1], yawrate_pid[2], yawrate_imax, yawrate_imin, false, node);
#endif
}

Eigen::Vector3d PIDController::compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time){
	double lin_vel_x = pid_linvel_x.computeCommand(goal.x() - current.x(), ros::Time::now() - last_time);
	double lin_vel_y = pid_linvel_y.computeCommand(goal.y() - current.y(), ros::Time::now() - last_time);
	double lin_vel_z = pid_linvel_z.computeCommand(goal.z() - current.z(), ros::Time::now() - last_time);

	return Eigen::Vector3d(lin_vel_x, lin_vel_y, lin_vel_z);
}

double PIDController::compute_yawrate_effort(double goal, double current, ros::Time last_time){
	double yaw_rate = pid_yaw_rate.computeCommand(goal - current, ros::Time::now() - last_time);

	return yaw_rate;
}
