/**
 * @brief Wheel odometry plugin
 * @file wheel_odometry.cpp
 * @author Pavlo Kolomiiets <pkolomiets@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Pavlo Kolomiiets.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/WheelOdomStamped.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Wheel odometry plugin.
 *
 * This plugin allows computing and publishing wheel odometry coming from FCU wheel encoders.
 * Can use either wheel's RPM or WHEEL_DISTANCE messages (the latter gives better accuracy).
 *
 */
class WheelOdometryPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	WheelOdometryPlugin() : PluginBase(),
		wo_nh("~wheel_odometry"),
		count(0),
		odom_mode(OM::NONE),
		raw_send(false),
		twist_send(false),
		tf_send(false),
		yaw_initialized(false),
		rpose(Eigen::Vector3d::Zero()),
		rtwist(Eigen::Vector3d::Zero()),
		rpose_cov(Eigen::Matrix3d::Zero()),
		rtwist_cov(Eigen::Vector3d::Zero())
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// General params
		wo_nh.param("send_raw", raw_send, false);
		// Wheels configuration
		wo_nh.param("count", count, 2);
		count = std::max(1, count); // bound check

		bool use_rpm;
		wo_nh.param("use_rpm", use_rpm, false);
		if (use_rpm)
			odom_mode = OM::RPM;
		else
			odom_mode = OM::DIST;

		// Odometry params
		wo_nh.param("send_twist", twist_send, false);
		wo_nh.param<std::string>("frame_id", frame_id, "odom");
		wo_nh.param<std::string>("child_frame_id", child_frame_id, "base_link");
		wo_nh.param("vel_error", vel_cov, 0.1);
		vel_cov = vel_cov*vel_cov; // std -> cov
		// TF subsection
		wo_nh.param("tf/send", tf_send, false);
		wo_nh.param<std::string>("tf/frame_id", tf_frame_id, "odom");
		wo_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");

		// Read parameters for each wheel.
		{
			int iwheel = 0;
			while (true) {
				// Build the string in the form "wheelX", where X is the wheel number.
				// Check if we have "wheelX" parameter.
				// Indices starts from 0 and should increase without gaps.
				auto name = utils::format("wheel%i", iwheel++);

				// Check if we have "wheelX" parameter
				if (!wo_nh.hasParam(name)) break;

				// Read
				Eigen::Vector2d offset;
				double radius;

				wo_nh.param(name+"/x", offset[0], 0.0);
				wo_nh.param(name+"/y", offset[1], 0.0);
				wo_nh.param(name+"/radius", radius, 0.05);

				wheel_offset.push_back(offset);
				wheel_radius.push_back(radius);
			}

			// Check for all wheels specified
			if (wheel_offset.size() >= count) {
				// Duplicate 1st wheel if only one is available.
				// This generalizes odometry computations for 1- and 2-wheels configurations.
				if (wheel_radius.size() == 1) {
					wheel_offset.resize(2);
					wheel_radius.resize(2);
					wheel_offset[1].x() = wheel_offset[0].x();
					wheel_offset[1].y() = wheel_offset[0].y() + 1.0; // make separation non-zero to avoid div-by-zero
					wheel_radius[1] = wheel_radius[0];
				}

				// Check for non-zero wheel separation (first two wheels)
				double separation = std::abs(wheel_offset[1].y() - wheel_offset[0].y());
				if (separation < 1.e-5) {
					odom_mode = OM::NONE;
					ROS_WARN_NAMED("wo", "WO: Separation between the first two wheels is too small (%f).", separation);
				}

				// Check for reasonable radiuses
				for (int i = 0; i < wheel_radius.size(); i++) {
					if (wheel_radius[i] <= 1.e-5) {
						odom_mode = OM::NONE;
						ROS_WARN_NAMED("wo", "WO: Wheel #%i has incorrect radius (%f).", i, wheel_radius[i]);
					}
				}
			}
			else {
				odom_mode = OM::NONE;
				ROS_WARN_NAMED("wo", "WO: Not all wheels have parameters specified (%lu/%i).", wheel_offset.size(), count);
			}
		}

		// Advertise RPM-s and distance-s
		if (raw_send) {
			rpm_pub = wo_nh.advertise<mavros_msgs::WheelOdomStamped>("rpm", 10);
			dist_pub = wo_nh.advertise<mavros_msgs::WheelOdomStamped>("distance", 10);
		}

		// Advertize topics
		if (odom_mode != OM::NONE) {
			if (twist_send)
				twist_pub = wo_nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity", 10);
			else
				odom_pub = wo_nh.advertise<nav_msgs::Odometry>("odom", 10);
		}
		// No-odometry warning
		else
			ROS_WARN_NAMED("wo", "WO: No odometry computations will be performed.");

	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&WheelOdometryPlugin::handle_rpm),
			make_handler(&WheelOdometryPlugin::handle_wheel_distance)
		};
	}

private:
	ros::NodeHandle wo_nh;

	ros::Publisher rpm_pub;
	ros::Publisher dist_pub;
	ros::Publisher odom_pub;
	ros::Publisher twist_pub;

	/// @brief Odometry computation modes
	enum class OM {
		NONE,	//!< no odometry computation
		RPM,	//!< use wheel's RPM
		DIST	//!< use wheel's cumulative distance
	};
	OM odom_mode; //!< odometry computation mode

	int count;		//!< requested number of wheels to compute odometry
	bool raw_send;		//!< send wheel's RPM and cumulative distance
	std::vector<Eigen::Vector2d> wheel_offset; //!< wheel x,y offsets (m,NED)
	std::vector<double> wheel_radius; //!< wheel radiuses (m)

	bool twist_send;		//!< send geometry_msgs/TwistWithCovarianceStamped instead of nav_msgs/Odometry
	bool tf_send;			//!< send TF
	std::string frame_id;		//!< origin frame for topic headers
	std::string child_frame_id;	//!< body-fixed frame for topic headers
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF and Pose
	double vel_cov;			//!< wheel velocity measurement error 1-var (m/s)

	int count_meas;				//!< number of wheels in measurements
	ros::Time time_prev;			//!< timestamp of previous measurement
	std::vector<double> measurement_prev;	//!< previous measurement

	bool yaw_initialized;			//!< initial yaw initialized (from IMU)

	/// @brief Robot origin 2D-state (SI units)
	Eigen::Vector3d rpose;		//!< pose (x, y, yaw)
	Eigen::Vector3d rtwist;		//!< twist (vx, vy, vyaw)
	Eigen::Matrix3d rpose_cov;	//!< pose error 1-var
	Eigen::Vector3d rtwist_cov;	//!< twist error 1-var (vx_cov, vy_cov, vyaw_cov)

	/**
	 * @brief Publish odometry.
	 * Odometry is computed from the very start but no pose info is published until we have initial orientation (yaw).
	 * Once we get it, the robot's current pose is updated with it and starts to be published.
	 * Twist info doesn't depend on initial orientation so is published from the very start.
	 * @param time		measurement's ROS time stamp
	 */
	void publish_odometry(ros::Time time)
	{
		// Get initial yaw (from IMU)
		// Check that IMU was already initialized
		if (!yaw_initialized && m_uas->get_attitude_imu_enu()) {
			double yaw = ftf::quaternion_get_yaw(ftf::to_eigen(m_uas->get_attitude_orientation_enu()));

			// Rotate current pose by initial yaw
			Eigen::Rotation2Dd rot(yaw);
			rpose.head(2) = rot * rpose.head(2); // x,y
			rpose(2) += yaw; // yaw

			ROS_INFO_NAMED("wo", "WO: Initial yaw (deg): %f", yaw/M_PI*180.0);
			yaw_initialized = true;
		}

		// Orientation (only if we have initial yaw)
		geometry_msgs::Quaternion quat;
		if (yaw_initialized)
			quat = tf2::toMsg(ftf::quaternion_from_rpy(0.0, 0.0, rpose(2)));

		// Twist
		geometry_msgs::TwistWithCovariance twist_cov;
		// linear
		twist_cov.twist.linear.x = rtwist(0);
		twist_cov.twist.linear.y = rtwist(1);
		twist_cov.twist.linear.z = 0.0;
		// angular
		twist_cov.twist.angular.x = 0.0;
		twist_cov.twist.angular.y = 0.0;
		twist_cov.twist.angular.z = rtwist(2);
		// covariance
		ftf::EigenMapCovariance6d twist_cov_map(twist_cov.covariance.data());
		twist_cov_map.setZero();
		twist_cov_map.block<2, 2>(0, 0).diagonal() << rtwist_cov(0), rtwist_cov(1);
		twist_cov_map.block<1, 1>(5, 5).diagonal() << rtwist_cov(2);

		// Publish twist
		if (twist_send) {
			auto twist_cov_t = boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
			// header
			twist_cov_t->header.stamp = time;
			twist_cov_t->header.frame_id = frame_id;
			// twist
			twist_cov_t->twist = twist_cov;
			// publish
			twist_pub.publish(twist_cov_t);
		}
		// Publish odometry (only if we have initial yaw)
		else if (yaw_initialized) {
			auto odom = boost::make_shared<nav_msgs::Odometry>();
			// header
			odom->header.stamp = time;
			odom->header.frame_id = frame_id;
			odom->child_frame_id = child_frame_id;
			// pose
			odom->pose.pose.position.x = rpose(0);
			odom->pose.pose.position.y = rpose(1);
			odom->pose.pose.position.z = 0.0;
			odom->pose.pose.orientation = quat;
			ftf::EigenMapCovariance6d pose_cov_map(odom->pose.covariance.data());
			pose_cov_map.block<2, 2>(0, 0) << rpose_cov.block<2, 2>(0, 0);
			pose_cov_map.block<2, 1>(0, 5) << rpose_cov.block<2, 1>(0, 2);
			pose_cov_map.block<1, 2>(5, 0) << rpose_cov.block<1, 2>(2, 0);
			pose_cov_map.block<1, 1>(5, 5) << rpose_cov.block<1, 1>(2, 2);
			// twist
			odom->twist = twist_cov;
			// publish
			odom_pub.publish(odom);
		}

		// Publish TF (only if we have initial yaw)
		if (tf_send && yaw_initialized) {
			geometry_msgs::TransformStamped transform;
			// header
			transform.header.stamp = time;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;
			// translation
			transform.transform.translation.x = rpose(0);
			transform.transform.translation.y = rpose(1);
			transform.transform.translation.z = 0.0;
			// rotation
			transform.transform.rotation = quat;
			// publish
			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}

	/**
	 * @brief Update odometry for differential drive robot.
	 * Odometry is computed for robot's origin (IMU).
	 * The wheels are assumed to be parallel to the robot's x-direction (forward) and with the same x-offset.
	 * No slip is assumed (Instantaneous Center of Curvature (ICC) along the axis connecting the wheels).
	 * All computations are performed for ROS frame conventions.
	 * The approach is the extended and more accurate version of standard one described in the book
	 * https://github.com/correll/Introduction-to-Autonomous-Robots
	 * and at the page (with some typos fixed)
	 * http://correll.cs.colorado.edu/?p=1307
	 * The extension is that exact pose update is used instead of approximate,
	 * and that the robot's origin can be specified anywhere instead of the middle-point between the wheels.
	 * @param distance	distance traveled by each wheel since last odometry update
	 * @param dt		time elapse since last odometry update (s)
	 */
	void update_odometry_diffdrive(std::vector<double> distance, double dt)
	{
		double y0 = wheel_offset[0](1);
		double y1 = wheel_offset[1](1);
		double a = -wheel_offset[0](0);
		double dy_inv = 1.0 / (y1 - y0);
		double dt_inv = 1.0 / dt;

		// Rotation angle
		double theta = (distance[1] - distance[0]) * dy_inv;
		// Distance traveled by the projection of origin onto the axis connecting the wheels (Op)
		double L = (y1 * distance[0] - y0 * distance[1]) * dy_inv;

		// Instantenous pose update in local (robot) coordinate system (vel*dt)
		Eigen::Vector3d v(L, a*theta, theta);
		// Instantenous local twist
		rtwist = v * dt_inv;

		// Compute local pose update (approximate).
		// In the book a=0 and |y0|=|y1|, additionally.
		// dx = L*cos(theta/2) - a*theta*sin(theta/2)
		// dy = L*sin(theta/2) + a*theta*cos(theta/2)
		// Compute local pose update (exact)
		// dx = a*(cos(theta)-1) + R*sin(theta)
		// dy = a*sin(theta) - R*(cos(theta)-1)
		// where R - rotation radius of Op around ICC (R = L/theta).
		double cos_theta = std::cos(theta);
		double sin_theta = std::sin(theta);
		double p; // sin(theta)/theta
		double q; // (1-cos(theta))/theta
		if (std::abs(theta) > 1.e-5) {
			p = sin_theta / theta;
			q = (1.0 - cos_theta) / theta;
		}
		// Limits for theta -> 0
		else {
			p = 1.0;
			q = 0.0;
		}

		// Local pose update matrix
		Eigen::Matrix3d M;
		M << p, -q,  0,
			 q,  p,  0,
			 0,  0,  1;

		// Local pose update
		Eigen::Vector3d dpose = M * v;

		// Rotation by yaw
		double cy = std::cos(rpose(2));
		double sy = std::sin(rpose(2));
		Eigen::Matrix3d R;
		R << cy, -sy,  0,
			 sy,  cy,  0,
			  0,   0,  1;

		// World pose
		rpose += R * dpose;
		rpose(2) = fmod(rpose(2), 2.0*M_PI); // Clamp to (-2*PI, 2*PI)

		// Twist errors (constant in time)
		if (rtwist_cov(0) == 0.0) {
			rtwist_cov(0) = vel_cov * (y0*y0 + y1*y1) * dy_inv*dy_inv; // vx_cov
			rtwist_cov(1) = vel_cov * a*a * 2.0 * dy_inv*dy_inv + 0.001; // vy_cov (add extra error, otherwise vy_cov= 0 if a=0)
			rtwist_cov(2) = vel_cov * 2.0 * dy_inv*dy_inv; // vyaw_cov
		}

		// Pose errors (accumulated in time).
		// Exact formulations respecting kinematic equations.
		// dR/dYaw
		Eigen::Matrix3d R_yaw;
		R_yaw << -sy, -cy,  0,
				  cy, -sy,  0,
				   0,   0,  0;
		// dYaw/dPose
		Eigen::Vector3d yaw_pose(0, 0, 1);
		// Jacobian by previous pose
		Eigen::Matrix3d J_pose = Eigen::Matrix3d::Identity() + R_yaw * dpose * yaw_pose.transpose();

		// dL,dTheta / dL0,dL1
		double L_L0 = y1 * dy_inv;
		double L_L1 = -y0 * dy_inv;
		double theta_L0 = -dy_inv;
		double theta_L1 = dy_inv;
		// dv/dMeasurement
		Eigen::Matrix<double, 3, 2> v_meas;
		v_meas <<       L_L0,       L_L1,
				  a*theta_L0, a*theta_L1,
					theta_L0,   theta_L1;
		// dTheta/dMeasurement
		Eigen::Vector2d theta_meas(theta_L0, theta_L1);
		// dM/dTheta
		double px; // dP/dTheta
		double qx; // dQ/dTheta
		if (std::abs(theta) > 1.e-5) {
			px = (theta*cos_theta - sin_theta) / (theta*theta);
			qx = (theta*sin_theta - (1-cos_theta)) / (theta*theta);
		}
		// Limits for theta -> 0
		else {
			px = 0;
			qx = 0.5;
		}
		// dM/dTheta
		Eigen::Matrix3d M_theta;
		M_theta << px, -qx,  0,
				   qx,  px,  0,
					0,   0,  0;
		// Jacobian by measurement
		Eigen::Matrix<double, 3, 2> J_meas = R * (M * v_meas + M_theta * v * theta_meas.transpose());

		// Measurement cov
		double L0_cov = vel_cov * dt*dt;
		double L1_cov = vel_cov * dt*dt;
		Eigen::Matrix2d meas_cov;
		meas_cov << L0_cov,     0,
						 0, L1_cov;

		// Update pose cov
		rpose_cov = J_pose * rpose_cov * J_pose.transpose() + J_meas * meas_cov * J_meas.transpose();
	}

	/**
	 * @brief Update odometry (currently, only 2-wheels differential configuration implemented).
	 * Odometry is computed for robot's origin (IMU).
	 * @param distance	distance traveled by each wheel since last odometry update
	 * @param dt		time elapse since last odometry update (s)
	 */
	void update_odometry(std::vector<double> distance, double dt)
	{
		// Currently, only 2-wheels configuration implemented
		int nwheels = std::min(2, (int)distance.size());
		switch (nwheels)
		{
		// Differential drive robot.
		case 2:
			update_odometry_diffdrive(distance, dt);
			break;
		}
	}

	/**
	 * @brief Process wheel measurement.
	 * @param measurement	measurement
	 * @param rpm		whether measurement contains RPM-s or cumulative wheel distances
	 * @param time		measurement's internal time stamp (for accurate dt computations)
	 * @param time_pub	measurement's time stamp for publish
	 */
	void process_measurement(std::vector<double> measurement, bool rpm, ros::Time time, ros::Time time_pub)
	{
		// Initial measurement
		if (time_prev == ros::Time(0)) {
			count_meas = measurement.size();
			measurement_prev.resize(count_meas);
			count = std::min(count, count_meas); // don't try to use more wheels than we have
		}
		// Same time stamp (messages are generated by FCU more often than the wheel state updated)
		else if (time == time_prev) {
			return;
		}
		// # of wheels differs from the initial value
		else if (measurement.size() != count_meas) {
			ROS_WARN_THROTTLE_NAMED(10, "wo", "WO: Number of wheels in measurement (%lu) differs from the initial value (%i).", measurement.size(), count_meas);
			return;
		}
		// Compute odometry
		else {
			double dt = (time - time_prev).toSec(); // Time since previous measurement (s)

			// Distance traveled by each wheel since last measurement.
			// Reserve for at least 2 wheels.
			std::vector<double> distance(std::max(2, count));
			// Compute using RPM-s
			if (rpm) {
				for (int i = 0; i < count; i++) {
					double RPM_2_SPEED = wheel_radius[i] * 2.0 * M_PI / 60.0; // RPM -> speed (m/s)
					double rpm = 0.5 * (measurement[i] + measurement_prev[i]); // Mean RPM during last dt seconds
					distance[i] = rpm * RPM_2_SPEED * dt;
				}
			}
			// Compute using cumulative distances
			else {
				for (int i = 0; i < count; i++)
					distance[i] = measurement[i] - measurement_prev[i];
			}

			// Make distance of the 2nd wheel equal to that of the 1st one if requested or only one is available.
			// This generalizes odometry computations for 1- and 2-wheels configurations.
			if (count == 1)
				distance[1] = distance[0];

			// Update odometry
			update_odometry(distance, dt);

			// Publish odometry
			publish_odometry(time_pub);
		}

		// Time step
		time_prev = time;
		std::copy_n(measurement.begin(), measurement.size(), measurement_prev.begin());
	}

	/* -*- message handlers -*- */

	/**
	 * @brief Handle Ardupilot RPM MAVlink message.
	 * Message specification: http://mavlink.io/en/messages/ardupilotmega.html#RPM
	 * @param msg	Received Mavlink msg
	 * @param rpm	RPM msg
	 */
	void handle_rpm(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::RPM &rpm)
	{
		// Get ROS timestamp of the message
		ros::Time timestamp =  ros::Time::now();

		// Publish RPM-s
		if (raw_send) {
			auto rpm_msg = boost::make_shared<mavros_msgs::WheelOdomStamped>();

			rpm_msg->header.stamp = timestamp;
			rpm_msg->data.resize(2);
			rpm_msg->data[0] = rpm.rpm1;
			rpm_msg->data[1] = rpm.rpm2;

			rpm_pub.publish(rpm_msg);
		}

		// Process measurement
		if (odom_mode == OM::RPM) {
			std::vector<double> measurement{rpm.rpm1, rpm.rpm2};
			process_measurement(measurement, true, timestamp, timestamp);
		}
	}

	/**
	 * @brief Handle WHEEL_DISTANCE MAVlink message.
	 * Message specification: https://mavlink.io/en/messages/common.html#WHEEL_DISTANCE
	 * @param msg	Received Mavlink msg
	 * @param dist	WHEEL_DISTANCE msg
	 */
	void handle_wheel_distance(const mavlink::mavlink_message_t *msg, mavlink::common::msg::WHEEL_DISTANCE &wheel_dist)
	{
		// Check for bad wheels count
		if (wheel_dist.count == 0)
			return;

		// Get ROS timestamp of the message
		ros::Time timestamp = m_uas->synchronise_stamp(wheel_dist.time_usec);
		// Get internal timestamp of the message
		ros::Time timestamp_int = ros::Time(wheel_dist.time_usec / 1000000UL,  1000UL * (wheel_dist.time_usec % 1000000UL));

		// Publish distances
		if (raw_send) {
			auto wheel_dist_msg = boost::make_shared<mavros_msgs::WheelOdomStamped>();

			wheel_dist_msg->header.stamp = timestamp;
			wheel_dist_msg->data.resize(wheel_dist.count);
			std::copy_n(wheel_dist.distance.begin(), wheel_dist.count, wheel_dist_msg->data.begin());

			dist_pub.publish(wheel_dist_msg);
		}

		// Process measurement
		if (odom_mode == OM::DIST) {
			std::vector<double> measurement(wheel_dist.count);
			std::copy_n(wheel_dist.distance.begin(), wheel_dist.count, measurement.begin());
			process_measurement(measurement, false, timestamp_int, timestamp);
		}
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::WheelOdometryPlugin, mavros::plugin::PluginBase)
