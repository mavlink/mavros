/**
 * @brief Offboard control test
 * @file offboard_control.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Andre Nguyen <andre-phu-van.nguyen@polymtl.ca>
 *
 * @addtogroup sitl_test
 * @{
 */
/*
 * Copyright 2015 Nuno Marques, Andre Nguyen.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <array>
#include <random>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <test_mavros/sitl_test/sitl_test.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace testsetup {
/**
 * @brief Offboard controller tester
 *
 * Tests offboard position, velocity and acceleration control
 *
 */

typedef enum {
	POSITION,
	VELOCITY,
	ACCELERATION
} control_mode;

typedef enum {
	SQUARE,
	CIRCLE,
	EIGHT,
	ELLIPSE
} path_shape;

class OffboardControl {
public:
	OffboardControl() :
		nh_sp(test.nh),
		local_pos_sp_pub(nh_sp.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10)),
		vel_sp_pub(nh_sp.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10)),
		local_pos_sub(nh_sp.subscribe("/mavros/local_position/local", 10, &OffboardControl::local_pos_cb, this)),
		threshold(threshold_definition())
	{ };

	void init() {
		/**
		 * @brief Setup of the test conditions
		 */
		test.setup(nh_sp);
		rate = test.rate;
		use_pid = test.use_pid;
		num_of_tests = test.num_of_tests;

		if (use_pid) {
			/**
			 * @note some of these are based on values defaulted @ https://bitbucket.org/enddl22/ardrone_side_project/
			 * tweaks to them so to get a better velocity response are welcomed!
			 */

			// Linear velocity PID gains and bound of integral windup
			nh_sp.param("linvel_p_gain", linvel_p_gain, 0.4);
			nh_sp.param("linvel_i_gain", linvel_i_gain, 0.05);
			nh_sp.param("linvel_d_gain", linvel_d_gain, 0.12);
			nh_sp.param("linvel_i_max", linvel_i_max, 0.1);
			nh_sp.param("linvel_i_min", linvel_i_min, -0.1);

			// Yaw rate PID gains and bound of integral windup
			nh_sp.param("yawrate_p_gain", yawrate_p_gain, 0.011);
			nh_sp.param("yawrate_i_gain", yawrate_i_gain, 0.00058);
			nh_sp.param("yawrate_d_gain", yawrate_d_gain, 0.12);
			nh_sp.param("yawrate_i_max", yawrate_i_max, 0.005);
			nh_sp.param("yawrate_i_min", yawrate_i_min, -0.005);

			// Setup of the PID controllers
			pid.setup_linvel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min, nh_sp);
			pid.setup_yawrate_pid(yawrate_p_gain, yawrate_i_gain, yawrate_d_gain, yawrate_i_max, yawrate_i_min, nh_sp);
		}

		/**
		 * @brief Setpoint control mode selector
		 *
		 * Available modes:
		 * - position
		 * - velocity
		 * - acceleration
		 */
		std::string mode_;
		nh_sp.param<std::string>("mode", mode_, "position");

		/**
		 * @brief Setpoint path shape selector
		 *
		 * Available shapes:
		 * - square
		 * - circle
		 * - eight
		 * - ellipse (in 3D space)
		 */
		std::string shape_;
		nh_sp.param<std::string>("shape", shape_, "square");

		if (mode_ == "position")
			mode = POSITION;
		else if (mode_ == "velocity")
			mode = VELOCITY;
		else if (mode_ == "acceleration")
			mode = ACCELERATION;
		else {
			ROS_ERROR_NAMED("sitl_test", "Control mode: wrong/unexistant control mode name %s", mode_.c_str());
			return;
		}

		if (shape_ == "square")
			shape = SQUARE;
		else if (shape_ == "circle")
			shape = CIRCLE;
		else if (shape_ == "eight")
			shape = EIGHT;
		else if (shape_ == "ellipse")
			shape = ELLIPSE;
		else {
			ROS_ERROR_NAMED("sitl_test", "Path shape: wrong/unexistant path shape name %s", shape_.c_str());
			return;
		}
	}

	/* -*- main routine -*- */

	void spin(int argc, char *argv[]) {
		init();
		ros::Rate loop_rate(rate);

		ROS_INFO("SITL Test: Offboard control test running!");

		if (mode == POSITION) {
			ROS_INFO("Position control mode selected.");
		}
		else if (mode == VELOCITY) {
			ROS_INFO("Velocity control mode selected.");
		}
		else if (mode == ACCELERATION) {
			ROS_INFO("Acceleration control mode selected.");
			ROS_ERROR_NAMED("sitl_test", "Control mode: acceleration control mode not supported in PX4 current Firmware.");
			/**
			 * @todo: lacks firmware support, for now
			 */
			return;
		}

		if (shape == SQUARE) {
			ROS_INFO("Test option: square-shaped path...");
			square_path_motion(loop_rate, mode);
		}
		else if (shape == CIRCLE) {
			ROS_INFO("Test option: circle-shaped path...");
			circle_path_motion(loop_rate, mode);
		}
		else if (shape == EIGHT) {
			ROS_INFO("Test option: eight-shaped path...");
			eight_path_motion(loop_rate, mode);
		}
		else if (shape == ELLIPSE) {
			ROS_INFO("Test option: ellipse-shaped path...");
			ellipse_path_motion(loop_rate, mode);
		}
	}

private:
	TestSetup test;
	pidcontroller::PIDController pid;

	double rate;
	bool use_pid;
	int num_of_tests; //TODO: find a way to use this...

	double linvel_p_gain;
	double linvel_i_gain;
	double linvel_d_gain;
	double linvel_i_max;
	double linvel_i_min;

	double yawrate_p_gain;
	double yawrate_i_gain;
	double yawrate_d_gain;
	double yawrate_i_max;
	double yawrate_i_min;

	control_mode mode;
	path_shape shape;

	ros::NodeHandle nh_sp;
	ros::Publisher local_pos_sp_pub;
	ros::Publisher vel_sp_pub;
	ros::Subscriber local_pos_sub;

	geometry_msgs::PoseStamped localpos, ps;
	geometry_msgs::TwistStamped vs;

	Eigen::Vector3d current;

	std::array<double, 100> threshold;

	/* -*- helper functions -*- */

	/**
	 * @brief Defines single position setpoint
	 */
	Eigen::Vector3d pos_setpoint(int tr_x, int tr_y, int tr_z){
		/** @todo Give possibility to user define amplitude of movement (square corners coordinates)*/
		return Eigen::Vector3d(tr_x * 2.0f, tr_y * 2.0f, tr_z * 1.0f);	// meters
	}

	/**
	 * @brief Defines circle path
	 */
	Eigen::Vector3d circle_shape(int angle){
		/** @todo Give possibility to user define amplitude of movement (circle radius)*/
		double r = 5.0f;	// 5 meters radius

		return Eigen::Vector3d(r * cos(angles::from_degrees(angle)),
				r * sin(angles::from_degrees(angle)),
				1.0f);
	}

	/**
	 * @brief Defines Gerono lemniscate path
	 */
	Eigen::Vector3d eight_shape(int angle){
		/** @todo Give possibility to user define amplitude of movement (vertical tangent size)*/
		double a = 5.0f;	// vertical tangent with 5 meters size

		return Eigen::Vector3d(a * cos(angles::from_degrees(angle)),
				a * sin(angles::from_degrees(angle)) * cos(angles::from_degrees(angle)),
				1.0f);
	}

	/**
	 * @brief Defines ellipse path
	 */
	Eigen::Vector3d ellipse_shape(int angle){
		/** @todo Give possibility to user define amplitude of movement (tangent sizes)*/
		double a = 5.0f;	// major axis
		double b = 2.0f;	// minor axis

		// rotation around y-axis
		return Eigen::Vector3d(a * cos(angles::from_degrees(angle)),
				0.0f,
				2.5f + b * sin(angles::from_degrees(angle)));
	}

	/**
	 * @brief Square path motion routine
	 */
	void square_path_motion(ros::Rate loop_rate, control_mode mode){
		uint8_t pos_target = 1;

		ROS_INFO("Testing...");

		while (ros::ok()) {
			wait_and_move(ps);

			// motion routine
			switch (pos_target) {
			case 1:
				tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
				break;
			case 2:
				tf::pointEigenToMsg(pos_setpoint(-1, 1, 1), ps.pose.position);
				break;
			case 3:
				tf::pointEigenToMsg(pos_setpoint(-1, -1, 1), ps.pose.position);
				break;
			case 4:
				tf::pointEigenToMsg(pos_setpoint(1, -1, 1), ps.pose.position);
				break;
			case 5:
				tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
				break;
			default:
				break;
			}

			if (pos_target == 6) {
				ROS_INFO("Test complete!");
				ros::shutdown();
			}
			else
				++pos_target;

			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	/**
	 * @brief Circle path motion routine
	 */
	void circle_path_motion(ros::Rate loop_rate, control_mode mode){
		ROS_INFO("Testing...");
		ros::Time last_time = ros::Time::now();

		while (ros::ok()) {
			tf::pointMsgToEigen(localpos.pose.position, current);

			// starting point
			if (mode == POSITION) {
				tf::pointEigenToMsg(Eigen::Vector3d(5.0f, 0.0f, 1.0f), ps.pose.position);
				local_pos_sp_pub.publish(ps);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(
								Eigen::Vector3d(5.0f, 0.0f, 1.0f), current, last_time), vs.twist.linear);
				else
					tf::vectorEigenToMsg(Eigen::Vector3d(5.0f - current.x(), -current.y(), 1.0f - current.z()), vs.twist.linear);
				vel_sp_pub.publish(vs);
			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}

			wait_and_move(ps);

			// motion routine
			for (int theta = 0; theta <= 360; theta++) {
				tf::pointMsgToEigen(localpos.pose.position, current);

				if (mode == POSITION) {
					tf::pointEigenToMsg(circle_shape(theta), ps.pose.position);
					local_pos_sp_pub.publish(ps);
				}
				else if (mode == VELOCITY) {
					if (use_pid)
						tf::vectorEigenToMsg(pid.compute_linvel_effort(circle_shape(theta), current, last_time), vs.twist.linear);
					else
						tf::vectorEigenToMsg(circle_shape(theta) - current, vs.twist.linear);
					vel_sp_pub.publish(vs);
				}
				else if (mode == ACCELERATION) {
					// TODO
					return;
				}
				if (theta == 360) {
					ROS_INFO("Test complete!");
					ros::shutdown();
				}
				last_time = ros::Time::now();
				loop_rate.sleep();
				ros::spinOnce();
			}
		}
	}

	/**
	 * @brief Eight path motion routine
	 */
	void eight_path_motion(ros::Rate loop_rate, control_mode mode){
		ROS_INFO("Testing...");
		ros::Time last_time = ros::Time::now();

		while (ros::ok()) {
			tf::pointMsgToEigen(localpos.pose.position, current);

			// starting point
			if (mode == POSITION) {
				tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 1.0f), ps.pose.position);
				local_pos_sp_pub.publish(ps);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(
								Eigen::Vector3d(0.0f, 0.0f, 1.0f), current, last_time), vs.twist.linear);
				else
					tf::vectorEigenToMsg(Eigen::Vector3d(-current.x(), -current.y(), 1.0f - current.z()), vs.twist.linear);
				vel_sp_pub.publish(vs);
			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}

			wait_and_move(ps);

			// motion routine
			for (int theta = -180; theta <= 180; theta++) {
				tf::pointMsgToEigen(localpos.pose.position, current);

				if (mode == POSITION) {
					tf::pointEigenToMsg(eight_shape(theta), ps.pose.position);
					local_pos_sp_pub.publish(ps);
				}
				else if (mode == VELOCITY) {
					if (use_pid)
						tf::vectorEigenToMsg(pid.compute_linvel_effort(eight_shape(theta), current, last_time), vs.twist.linear);
					else
						tf::vectorEigenToMsg(eight_shape(theta) - current, vs.twist.linear);
					vel_sp_pub.publish(vs);
				}
				else if (mode == ACCELERATION) {
					// TODO
					return;
				}
				if (theta == 180) {
					ROS_INFO("Test complete!");
					ros::shutdown();
				}
				last_time = ros::Time::now();
				loop_rate.sleep();
				ros::spinOnce();
			}
		}
	}

	/**
	 * @brief Ellipse path motion routine
	 */
	void ellipse_path_motion(ros::Rate loop_rate, control_mode mode){
		ROS_INFO("Testing...");
		ros::Time last_time = ros::Time::now();

		while (ros::ok()) {
			tf::pointMsgToEigen(localpos.pose.position, current);

			// starting point
			if (mode == POSITION) {
				tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 2.5f), ps.pose.position);
				local_pos_sp_pub.publish(ps);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(
								Eigen::Vector3d(0.0f, 0.0f, 2.5f), current, last_time), vs.twist.linear);
				else
					tf::vectorEigenToMsg(Eigen::Vector3d(-current.x(), -current.y(), 2.5f - current.z()), vs.twist.linear);
				vel_sp_pub.publish(vs);
			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}

			wait_and_move(ps);

			// motion routine
			for (int theta = 0; theta <= 360; theta++) {
				tf::pointMsgToEigen(localpos.pose.position, current);

				if (mode == POSITION) {
					tf::pointEigenToMsg(ellipse_shape(theta), ps.pose.position);
					local_pos_sp_pub.publish(ps);
				}
				else if (mode == VELOCITY) {
					if (use_pid)
						tf::vectorEigenToMsg(pid.compute_linvel_effort(ellipse_shape(theta), current, last_time), vs.twist.linear);
					else
						tf::vectorEigenToMsg(ellipse_shape(theta) - current, vs.twist.linear);
					vel_sp_pub.publish(vs);
				}
				else if (mode == ACCELERATION) {
					// TODO
					return;
				}
				if (theta == 360) {
					ROS_INFO("Test complete!");
					ros::shutdown();
				}
				last_time = ros::Time::now();
				loop_rate.sleep();
				ros::spinOnce();
			}
		}
	}

	/**
	 * @brief Defines the accepted threshold to the destination/target position
	 * before moving to the next setpoint.
	 */
	void wait_and_move(geometry_msgs::PoseStamped target){
		ros::Rate loop_rate(rate);
		ros::Time last_time = ros::Time::now();
		bool stop = false;

		Eigen::Vector3d dest;

		double distance;
		double err_th = threshold[rand() % threshold.size()];

		ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", err_th);

		while (ros::ok() && !stop) {
			tf::pointMsgToEigen(target.pose.position, dest);
			tf::pointMsgToEigen(localpos.pose.position, current);

			distance = sqrt((dest - current).x() * (dest - current).x() +
					(dest - current).y() * (dest - current).y() +
					(dest - current).z() * (dest - current).z());

			if (distance <= err_th)
				stop = true;

			if (mode == POSITION) {
				local_pos_sp_pub.publish(target);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(dest, current, last_time), vs.twist.linear);
				else
					tf::vectorEigenToMsg(dest - current, vs.twist.linear);
				vel_sp_pub.publish(vs);
			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	/**
	 * @brief Gaussian noise generator for accepted position threshold
	 */
	std::array<double, 100> threshold_definition(){
		std::random_device rd;
		std::mt19937 gen(rd());
		std::array<double, 100> th_values;

		std::normal_distribution<double> th(0.1f,0.05f);

		for (auto &value : th_values) {
			value = th(gen);
		}
		return th_values;
	}

	/* -*- callbacks -*- */

	void local_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg){
		localpos = *msg;
	}
};
};	// namespace testsetup
