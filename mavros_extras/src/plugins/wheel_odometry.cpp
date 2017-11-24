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
#include <mavros_msgs/Float32ArrayStamped.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Ardupilot wheel odometry plugin.
 *
 * This plugin allows computing and publishing wheel odometry coming from Ardupilot FCU wheel encoders.
 * Can use either wheel's RPM or WHEEL_DISTANCE messages (the latter gives better accuracy).
 *
 */
class WheelOdometryPlugin : public plugin::PluginBase {
public:
	WheelOdometryPlugin() : PluginBase(),
		wo_nh("~wheel_odometry"),
		count(0),
		odom_mode(OM::NONE),
		twist_send(false),
		tf_send(false),
		diagnostics(false),
		yaw_initialized(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// General params
		wo_nh.param("diagnostics", diagnostics, false);
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
		wo_nh.param("twist", twist_send, false);
		wo_nh.param<std::string>("frame_id", frame_id, "odom");
		wo_nh.param<std::string>("child_frame_id", child_frame_id, "base_link");
		wo_nh.param("vel_error", vel_cov, 0.1f);
		vel_cov = vel_cov*vel_cov; // std -> cov
		// TF subsection
		wo_nh.param("tf/send", tf_send, false);
		wo_nh.param<std::string>("tf/frame_id", tf_frame_id, "odom");
		wo_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");

		// Read parameters for each wheel.
		{
			int iwheel = 0;
			bool has_param;
			do {
				// Build the string in the form "wheelX", where X is the wheel number.
				// Check if we have "wheelX" parameter.
				// Indices starts from 0 and should increase without gaps.
				std::stringstream ss;
				ss << "wheel" << iwheel++;
				std::string wheel_name = ss.str();
				// Check if we have "wheelX" parameter
				has_param = wo_nh.hasParam(wheel_name);

				if (has_param) {
					Eigen::Vector2f offset;
					float radius;

					wo_nh.param(wheel_name+"/x", offset[0], 0.f);
					wo_nh.param(wheel_name+"/y", offset[1], 0.f);
					wo_nh.param(wheel_name+"/radius", radius, 0.05f);

					wheel_offset.push_back(offset);
					wheel_radius.push_back(radius);
				}
			}
			while (has_param);

			// Check for all wheels specified
			if (wheel_offset.size() < count) {
				odom_mode = OM::NONE;
				ROS_WARN_NAMED("wo", "WO: Not all wheels have parameters specified (%lu/%i).", wheel_offset.size(), count);
			}
			// Duplicate 1st wheel if only one is available.
			// This generalizes odometry computations for 1- and 2-wheels configurations.
			else if (wheel_radius.size() == 1) {
				wheel_offset.resize(2);
				wheel_radius.resize(2);
				wheel_offset[1].x() = wheel_offset[0].x();
				wheel_offset[1].y() = wheel_offset[0].y() + 1.f; // make separation non-zero to avoid div-by-zero
				wheel_radius[1] = wheel_radius[0];
			}

			// Check for non-zero wheel separation (first two wheels)
			float separation = std::abs(wheel_offset[1].y() - wheel_offset[0].y());
			if (separation < 1.e-5f) {
				odom_mode = OM::NONE;
				ROS_WARN_NAMED("wo", "WO: Separation between the first two wheels is too small (%f).", separation);
			}

			// Check for reasonable radiuses
			for (int i = 0; i < wheel_radius.size(); i++) {
				if (wheel_radius[i] <= 1.e-5f) {
					odom_mode = OM::NONE;
					ROS_WARN_NAMED("wo", "WO: Wheel #%i has incorrect radius (%f).", i, wheel_radius[i]);
				}
			}
		}

		// Advertise RPM-s and distance-s
		if (diagnostics) {
			rpm_pub = wo_nh.advertise<mavros_msgs::Float32ArrayStamped>("rpm", 10);
			dist_pub = wo_nh.advertise<mavros_msgs::Float32ArrayStamped>("distance", 10);
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

	Subscriptions get_subscriptions()
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
	tf2_ros::TransformBroadcaster tfBroadcaster;

	/// @brief Odometry computation modes
	enum class OM {
		NONE,	//!< no odometry computation
		RPM,	//!< use wheel's RPM
		DIST	//!< use wheel's cumulative distance
	};
	OM odom_mode; //!< odometry computation mode

	int count;		//!< requested number of wheels to compute odometry
	bool diagnostics;	//!< publish wheel's RPM and cumulative distance
	std::vector<Eigen::Vector2f> wheel_offset; //!< wheel x,y offsets (m,NED)
	std::vector<float> wheel_radius; //!< wheel radiuses (m)

	bool twist_send;		//!< send geometry_msgs/TwistWithCovarianceStamped instead of nav_msgs/Odometry
	bool tf_send;			//!< send TF
	std::string frame_id;		//!< origin frame for topic headers
	std::string child_frame_id;	//!< body-fixed frame for topic headers
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF and Pose
	float vel_cov;			//!< wheel velocity measurement error 1-var (m/s)

	int count_meas;				//!< number of wheels in measurements
	ros::Time time_prev;			//!< timestamp of previous measurement
	std::vector<float> measurement_prev;	//!< previous measurement

	bool yaw_initialized;			//!< initial yaw initialized (from IMU)

	/// @brief Robot origin 2D-state (SI units)
	Eigen::Vector3f rpose {0.f, 0.f, 0.f};		//!< pose (x, y, yaw)
	Eigen::Vector3f rtwist {0.f, 0.f, 0.f};		//!< twist (vx, vy, vyaw)
	Eigen::Vector3f rpose_cov {0.f, 0.f, 0.f};	//!< pose error 1-var (x_cov, y_cov, yaw_cov)
	Eigen::Vector3f rtwist_cov {0.f, 0.f, 0.f};	//!< twist error 1-var (vx_cov, vy_cov, vyaw_cov)

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
			auto quat = m_uas->get_attitude_orientation_enu();
			tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
			double roll, pitch, yaw;
			tf2::Matrix3x3 mat(q);
			mat.getRPY(roll, pitch, yaw);

			// Rotate current pose by initial yaw
			float cosy = std::cos(yaw);
			float siny = std::sin(yaw);
			float x = rpose(0);
			float y = rpose(1);
			rpose(0) = x*cosy - y*siny; // x
			rpose(1) = x*siny + y*cosy; // y
			rpose(2) += yaw; // yaw

			ROS_INFO_NAMED("wo", "WO: Initial yaw (deg) = %f\n", yaw/M_PI*180.0);
			yaw_initialized = true;
		}

		// Orientation (only if we have initial yaw)
		geometry_msgs::Quaternion quat;
		if (yaw_initialized) {
			tf2::Quaternion q;
			q.setRPY(0.0, 0.0, rpose(2));
			quat.x = q.x();
			quat.y = q.y();
			quat.z = q.z();
			quat.w = q.w();
		}

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
		twist_cov_map.block<3, 3>(0, 0).diagonal() << rtwist_cov(0), rtwist_cov(1), -1.0;
		twist_cov_map.block<3, 3>(3, 3).diagonal() << -1.0, -1.0, rtwist_cov(2);

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
			pose_cov_map.setZero();
			pose_cov_map.block<3, 3>(0, 0).diagonal() << rpose_cov(0), rpose_cov(1), -1.0;
			pose_cov_map.block<3, 3>(3, 3).diagonal() << -1.0, -1.0, rpose_cov(2);
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
	 * @brief Update odometry (currently, only 2-wheels differential configuration implemented).
	 * Odometry is computed for robot's origin (IMU).
	 * @param distance	distance traveled by each wheel since last odometry update
	 * @param dt		time elapse since last odometry update (s)
	 */
	void update_odometry(std::vector<float> distance, float dt)
	{
		// Currently, only 2-wheels configuration implemented
		int count = std::min(2, (int)distance.size());
		switch (distance.size())
		{
		// Differential drive robot.
		// The wheels are assumed to be parallel to the robot's x-direction (forward) and with the same x-offset.
		// No slip is assumed (Instantaneous Center of Curvature (ICC) along the axis connecting the wheels).
		// All computations are performed for ROS frame conventions.
		case 2:
			float y0 = wheel_offset[0](1);
			float y1 = wheel_offset[1](1);
			float a = -wheel_offset[0](0);
			float dy_inv = 1.f / (y1 - y0);
			float dt_inv = 1.f / dt;

			// Rotation angle
			float theta = (distance[1] - distance[0]) * dy_inv;
			// Distance traveled by the projection of origin onto the axis connecting the wheels (Op)
			float L = (y1 * distance[1] - y0 * distance[0]) * dy_inv;

			// Robot origin twist
			rtwist(0) = L * dt_inv; // vx
			rtwist(1) = a * theta * dt_inv; // vy
			rtwist(2) = theta * dt_inv; // vyaw

			// Compute displacement in robot origin frame
			// dx = a*(cos(theta)-1) + R*sin(theta)
			// dy = a*sin(theta) - R*(cos(theta)-1)
			// where R - rotation radius of Op around ICC (R = L/theta).
			float cos_theta = std::cos(theta);
			float sin_theta = std::sin(theta);
			float sin_theta_by_theta; // sin(theta)/theta
			float cos_theta_1_by_theta; // (cos(theta)-1)/theta
			// Limits for theta -> 0
			if (std::abs(theta) < 1.e-5f) {
				sin_theta_by_theta = 1.f;
				cos_theta_1_by_theta = 0.f;
			}
			else {
				float theta_inv = 1.f / theta;
				sin_theta_by_theta = sin_theta * theta_inv;
				cos_theta_1_by_theta = (cos_theta-1.f) * theta_inv;
			}
			float dx = a * (cos_theta-1.f) + L*sin_theta_by_theta;
			float dy = a * sin_theta - L*cos_theta_1_by_theta;

			// Pose in world frame
			float cosy = std::cos(rpose(2));
			float siny = std::sin(rpose(2));
			rpose(0) += dx*cosy - dy*siny; // x
			rpose(1) += dx*siny + dy*cosy; // y
			rpose(2) += theta; // yaw


			// Twist errors (constant in time)
			if (rtwist_cov(0) == 0.f) {
				rtwist_cov(0) = vel_cov * (y0*y0 + y1*y1) * dy_inv*dy_inv; // vx_cov
				rtwist_cov(1) = vel_cov * a*a * std::sqrt(2.f) * dy_inv*dy_inv + 0.001f; // vy_cov (add extra error, otherwise vy_cov= 0 if a=0)
				rtwist_cov(2) = vel_cov * 2.f * dy_inv*dy_inv; // vyaw_cov
			}
			// Pose errors (accumulated in time).
			// Simple approximation is used not respecting kinematic equations.
			// TODO: accurate odometry error propagation.
			rpose_cov(0) = rpose_cov(0) + rtwist_cov(0) * dt*dt; // x_cov
			rpose_cov(1) = rpose_cov(0); // y_cov
			rpose_cov(2) = rpose_cov(2) + rtwist_cov(2) * dt*dt; // yaw_cov

			break;
		}
	}

	/**
	 * @brief Process wheel measurement.
	 * @param measurement	measurement
	 * @param rpm		whether measurement contains RPM-s or cumulative wheel distances
	 * @param time		measurement's ROS time stamp
	 */
	void process_measurement(std::vector<float> measurement, bool rpm, ros::Time time)
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
			float dt = (time - time_prev).toSec(); // Time since previous measurement (s)

			// Distance traveled by each wheel since last measurement.
			// Reserve for at least 2 wheels.
			std::vector<float> distance(std::max(2, count));
			// Compute using RPM-s
			if (rpm) {
				for (int i = 0; i < count; i++) {
					float RPM_2_SPEED = wheel_radius[i] * 2.f * (float)M_PI / 60.f; // RPM -> speed (m/s)
					float rpm = 0.5f * (measurement[i] + measurement_prev[i]); // Mean RPM during last dt seconds
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
			publish_odometry(time);
		}

		// Time step
		time_prev = time;
		std::copy_n(measurement.begin(), measurement.size(), measurement_prev.begin());
	}

	/* -*- message handlers -*- */

	/**
	 * @brief Handle RPM MAVlink (Ardupilot) message.
	 * Message specification: http://mavlink.org/messages/ardupilotmega#RPM
	 * @param msg	Received Mavlink msg
	 * @param rpm	RPM msg
	 */
	void handle_rpm(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::RPM &rpm)
	{
		// Get ROS timestamp of the message
		ros::Time timestamp =  ros::Time::now();

		// Publish RPM-s
		if (diagnostics) {
			auto rpm_msg = boost::make_shared<mavros_msgs::Float32ArrayStamped>();

			rpm_msg->header.stamp = timestamp;
			rpm_msg->header.frame_id = "";
			rpm_msg->data.resize(2);
			rpm_msg->data[0] = rpm.rpm1;
			rpm_msg->data[1] = rpm.rpm2;

			rpm_pub.publish(rpm_msg);
		}

		// Process measurement
		if (odom_mode == OM::RPM) {
			std::vector<float> measurement(2);
			measurement[0] = rpm.rpm1;
			measurement[1] = rpm.rpm2;
			process_measurement(measurement, true, timestamp);
		}
	}

	/**
	 * @brief Handle WHEEL_DISTANCE MAVlink (Ardupilot) message.
	 * Message specification: http://mavlink.org/messages/ardupilotmega#WHEEL_DISTANCE
	 * @param msg	Received Mavlink msg
	 * @param dist	WHEEL_DISTANCE msg
	 */
	void handle_wheel_distance(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::WHEEL_DISTANCE &wheel_dist)
	{
		// Check for bad wheels count
		if (wheel_dist.count == 0)
			return;

		// Get ROS timestamp of the message
		ros::Time timestamp =  m_uas->synchronise_stamp(wheel_dist.time_usec);

		// Publish distances
		if (diagnostics) {
			auto wheel_dist_msg = boost::make_shared<mavros_msgs::Float32ArrayStamped>();

			wheel_dist_msg->header.stamp = timestamp;
			wheel_dist_msg->header.frame_id = "";
			wheel_dist_msg->data.resize(wheel_dist.count);
			std::copy_n(wheel_dist.distance.begin(), wheel_dist.count, wheel_dist_msg->data.begin());

			dist_pub.publish(wheel_dist_msg);
		}

		// Process measurement
		if (odom_mode == OM::DIST) {
			std::vector<float> measurement(wheel_dist.count);
			std::copy_n(wheel_dist.distance.begin(), wheel_dist.count, measurement.begin());
			process_measurement(measurement, false, timestamp);
		}
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::WheelOdometryPlugin, mavros::plugin::PluginBase)
