/**
 * @brief Trajectory plugin
 * @file trajectory.cpp
 * @author Martina Rivizzigno <martina@rivizzigno.it>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Martina Rivizzigno.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.mdA
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Path.h>

//! Point array LENGTH
static constexpr int POINT_LEN = 11;

namespace mavros {
namespace extra_plugins {
//! Mavlink MAV_TRAJECTORY_REPRESENTATION enumeration
using mavlink::common::MAV_TRAJECTORY_REPRESENTATION;

/**
 * @brief Trajectory plugin to receive planned path from the FCU and
 * send back to the FCU a corrected path (collision free, smoothed)
 *
 * @see trajectory_cb()
 */
class TrajectoryPlugin : public plugin::PluginBase {
public:
	TrajectoryPlugin() : PluginBase(),
		trajectory_nh("~trajectory")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		trajectory_generated_sub = trajectory_nh.subscribe("generated", 10, &TrajectoryPlugin::trajectory_cb, this);
		path_sub = trajectory_nh.subscribe("path", 10, &TrajectoryPlugin::path_cb, this);
		trajectory_desired_pub = trajectory_nh.advertise<mavros_msgs::Trajectory>("desired", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&TrajectoryPlugin::handle_trajectory)
		};
	}

private:
	ros::NodeHandle trajectory_nh;

	ros::Subscriber trajectory_generated_sub;
	ros::Subscriber path_sub;

	ros::Publisher trajectory_desired_pub;

	/**
	 * @brief Send corrected path to the FCU.
	 *
	 * Message specification: http://mavlink.org/messages/common#TRAJECTORY
	 * @param req	received Trajectory msg
	 */
	void trajectory_cb(const mavros_msgs::Trajectory::ConstPtr &req)
	{
		mavlink::common::msg::TRAJECTORY trajectory {};
		trajectory.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
		trajectory.type = req->type;	//!< trajectory type (waypoints, bezier)

		auto fill_points_position = [] (std::array<float, POINT_LEN> &point, geometry_msgs::Point position) {
			// [[[cog:
			//     cog.outl("auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));")
			//     for index, axis in zip ("012", "xyz"):
			//         cog.outl("point[{index}] = position_ned.{axis}();".format(**locals()))
			// ]]]
			auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));
			point[0] = position_ned.x();
			point[1] = position_ned.y();
			point[2] = position_ned.z();
			// [[[end]]] (checksum: 6b564ac3664cbc831bc9d5d912c976c0)
		};

		auto fill_points_velocity = [] (std::array<float, POINT_LEN> &point, geometry_msgs::Vector3 velocity) {
			// [[[cog:
			//     cog.outl("auto velocity_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(velocity));")
			//     for index, axis in zip ("345", "xyz"):
			//         cog.outl("point[{index}] = velocity_ned.{axis}();".format(**locals()))
			// ]]]
			auto velocity_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(velocity));
			point[3] = velocity_ned.x();
			point[4] = velocity_ned.y();
			point[5] = velocity_ned.z();
			// [[[end]]] (checksum: 0d3c1f396f81c3e3b11981b185ff1cbf)
		};

		auto fill_points_acceleration = [] (std::array<float, POINT_LEN> &point, geometry_msgs::Vector3 acceleration) {
			// [[[cog:
			//     cog.outl("auto acceleration_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(acceleration));")
			//     for index, axis in zip ("678", "xyz"):
			//         cog.outl("point[{index}] = acceleration_ned.{axis}();".format(**locals()))
			// ]]]
			auto acceleration_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(acceleration));
			point[6] = acceleration_ned.x();
			point[7] = acceleration_ned.y();
			point[8] = acceleration_ned.z();
			// [[[end]]] (checksum: 5ae0bb0aeb947ccb16b734ae67953c2e)
		};

		auto fill_points_yaw_wp = [this](std::array<float, POINT_LEN> &point, float yaw) {
			point[9] = wrap_pi(-yaw + (M_PI / 2.0f));
		};

		auto fill_points_yaw_bezier = [this](std::array<float, POINT_LEN> &point, float yaw) {
			point[4] = wrap_pi(-yaw + (M_PI / 2.0f));
		};

		auto fill_points_yaw_speed = [] (std::array<float, POINT_LEN> &point, float yaw_speed) {
			point[10] = yaw_speed;
		};

		auto fill_points_time_horizon = [] (std::array<float, POINT_LEN> &point, float time_horizon) {
			point[3] = time_horizon;
		};

		auto fill_points_unused_bezier = [] (std::array<float, POINT_LEN> &point) {
			std::fill(point.begin() + 5, point.end(), NAN);
		};

		// [[[cog:
		// for p in "12345":
		//     cog.outl("fill_points_position(trajectory.point_{p}, req->point_{p}.position);".format(**locals()))
		// ]]]
		fill_points_position(trajectory.point_1, req->point_1.position);
		fill_points_position(trajectory.point_2, req->point_2.position);
		fill_points_position(trajectory.point_3, req->point_3.position);
		fill_points_position(trajectory.point_4, req->point_4.position);
		fill_points_position(trajectory.point_5, req->point_5.position);
		// [[[end]]] (checksum: e1d89be8f56a63e27215e311cbadf019)

		if (req->type == utils::enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS)) {
			// [[[cog:
			// for p in "12345":
			//     cog.outl("fill_points_velocity(trajectory.point_{p},  req->point_{p}.velocity);".format(**locals()))
			//     cog.outl("fill_points_acceleration(trajectory.point_{p}, req->point_{p}.acceleration_or_force);".format(**locals()))
			//     cog.outl("fill_points_yaw_wp(trajectory.point_{p}, req->point_{p}.yaw);".format(**locals()))
			//     cog.outl("fill_points_yaw_speed(trajectory.point_{p}, req->point_{p}.yaw_rate);\n".format(**locals()))
			// ]]]
			fill_points_velocity(trajectory.point_1,  req->point_1.velocity);
			fill_points_acceleration(trajectory.point_1, req->point_1.acceleration_or_force);
			fill_points_yaw_wp(trajectory.point_1, req->point_1.yaw);
			fill_points_yaw_speed(trajectory.point_1, req->point_1.yaw_rate);

			fill_points_velocity(trajectory.point_2,  req->point_2.velocity);
			fill_points_acceleration(trajectory.point_2, req->point_2.acceleration_or_force);
			fill_points_yaw_wp(trajectory.point_2, req->point_2.yaw);
			fill_points_yaw_speed(trajectory.point_2, req->point_2.yaw_rate);

			fill_points_velocity(trajectory.point_3,  req->point_3.velocity);
			fill_points_acceleration(trajectory.point_3, req->point_3.acceleration_or_force);
			fill_points_yaw_wp(trajectory.point_3, req->point_3.yaw);
			fill_points_yaw_speed(trajectory.point_3, req->point_3.yaw_rate);

			fill_points_velocity(trajectory.point_4,  req->point_4.velocity);
			fill_points_acceleration(trajectory.point_4, req->point_4.acceleration_or_force);
			fill_points_yaw_wp(trajectory.point_4, req->point_4.yaw);
			fill_points_yaw_speed(trajectory.point_4, req->point_4.yaw_rate);

			fill_points_velocity(trajectory.point_5,  req->point_5.velocity);
			fill_points_acceleration(trajectory.point_5, req->point_5.acceleration_or_force);
			fill_points_yaw_wp(trajectory.point_5, req->point_5.yaw);
			fill_points_yaw_speed(trajectory.point_5, req->point_5.yaw_rate);

			// [[[end]]] (checksum: 8672611837052dffae6215e32ec14678)
		} else {
			// [[[cog:
			// for p in "12345":
			//     cog.outl("fill_points_time_horizon(trajectory.point_{p}, req->time_horizon[{p} - 1]);".format(**locals()))
			//     cog.outl("fill_points_yaw_bezier(trajectory.point_{p}, req->point_{p}.yaw);".format(**locals()))
			//     cog.outl("fill_points_unused_bezier(trajectory.point_{p});\n".format(**locals()))
			// ]]]
			fill_points_time_horizon(trajectory.point_1, req->time_horizon[1 - 1]);
			fill_points_yaw_bezier(trajectory.point_1, req->point_1.yaw);
			fill_points_unused_bezier(trajectory.point_1);

			fill_points_time_horizon(trajectory.point_2, req->time_horizon[2 - 1]);
			fill_points_yaw_bezier(trajectory.point_2, req->point_2.yaw);
			fill_points_unused_bezier(trajectory.point_2);

			fill_points_time_horizon(trajectory.point_3, req->time_horizon[3 - 1]);
			fill_points_yaw_bezier(trajectory.point_3, req->point_3.yaw);
			fill_points_unused_bezier(trajectory.point_3);

			fill_points_time_horizon(trajectory.point_4, req->time_horizon[4 - 1]);
			fill_points_yaw_bezier(trajectory.point_4, req->point_4.yaw);
			fill_points_unused_bezier(trajectory.point_4);

			fill_points_time_horizon(trajectory.point_5, req->time_horizon[5 - 1]);
			fill_points_yaw_bezier(trajectory.point_5, req->point_5.yaw);
			fill_points_unused_bezier(trajectory.point_5);

			// [[[end]]] (checksum: 03494272b7395e6a3dc6ca8d10597f39)
		}

		std::copy(req->point_valid.begin(), req->point_valid.end(), trajectory.point_valid.begin());

		UAS_FCU(m_uas)->send_message_ignore_drop(trajectory);
	}


	/**
	 * @brief Send corrected path to the FCU.
	 *
	 * Message specification: http://mavlink.org/messages/common#TRAJECTORY
	 * @param req	received nav_msgs Path msg
	 */
	void path_cb(const nav_msgs::Path::ConstPtr &req)
	{
		mavlink::common::msg::TRAJECTORY trajectory {};
		trajectory.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
		trajectory.type = utils::enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS);
		Eigen::Quaterniond q_enu;
		Eigen::Vector3d position_wp, orientation_wp;
		int conter = 0;

		auto fill_points_position = [] (std::array<float, POINT_LEN> &point, geometry_msgs::Point position) {
			// [[[cog:
			//     cog.outl("auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));")
			//     for index, axis in zip ("012", "xyz"):
			//         cog.outl("point[{index}] = position_ned.{axis}();".format(**locals()))
			// ]]]
			auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));
			point[0] = position_ned.x();
			point[1] = position_ned.y();
			point[2] = position_ned.z();
			// [[[end]]] (checksum: 6b564ac3664cbc831bc9d5d912c976c0)
		};

		auto fill_points_yaw = [this](std::array<float, POINT_LEN> &point, geometry_msgs::Quaternion orientation) {
			Eigen::Quaterniond q_enu;
			tf::quaternionMsgToEigen(orientation, q_enu);
			auto orientation_wp = ftf::quaternion_to_rpy(ftf::transform_orientation_enu_ned(ftf::transform_orientation_baselink_aircraft(q_enu)));
			point[9] = wrap_pi(-orientation_wp(2) + (M_PI / 2.0f));
		};

		auto fill_points_unused_fields = [] (std::array<float, POINT_LEN> &point) {
			std::fill(point.begin() + 3, point.begin() + 8, NAN);
			point[10] = NAN;
		};

		auto fill_points_unused = [] (std::array<float, POINT_LEN> &point) {
			std::fill(point.begin(), point.end(), NAN);
		};

		// [[[cog:
		// for p in "12345":
		//    cog.outl("if (req->poses.size() >= {p}) {{ \n".format(**locals()))
		//    cog.outl("fill_points_position(trajectory.point_{p}, req->poses[{p} - 1].pose.position);".format(**locals()))
		//    cog.outl("fill_points_yaw(trajectory.point_{p}, req->poses[{p} - 1].pose.orientation);".format(**locals()))
		//    cog.outl("fill_points_unused_fields(trajectory.point_{p});".format(**locals()))
		//    cog.outl("} else {")
		//    cog.outl("fill_points_unused(trajectory.point_{p});".format(**locals()))
		//    cog.outl("} \n")
		// ]]]
		if (req->poses.size() >= 1) {
			fill_points_position(trajectory.point_1, req->poses[1 - 1].pose.position);
			fill_points_yaw(trajectory.point_1, req->poses[1 - 1].pose.orientation);
			fill_points_unused_fields(trajectory.point_1);
		} else {
			fill_points_unused(trajectory.point_1);
		}

		if (req->poses.size() >= 2) {
			fill_points_position(trajectory.point_2, req->poses[2 - 1].pose.position);
			fill_points_yaw(trajectory.point_2, req->poses[2 - 1].pose.orientation);
			fill_points_unused_fields(trajectory.point_2);
		} else {
			fill_points_unused(trajectory.point_2);
		}

		if (req->poses.size() >= 3) {
			fill_points_position(trajectory.point_3, req->poses[3 - 1].pose.position);
			fill_points_yaw(trajectory.point_3, req->poses[3 - 1].pose.orientation);
			fill_points_unused_fields(trajectory.point_3);
		} else {
			fill_points_unused(trajectory.point_3);
		}

		if (req->poses.size() >= 4) {
			fill_points_position(trajectory.point_4, req->poses[4 - 1].pose.position);
			fill_points_yaw(trajectory.point_4, req->poses[4 - 1].pose.orientation);
			fill_points_unused_fields(trajectory.point_4);
		} else {
			fill_points_unused(trajectory.point_4);
		}

		if (req->poses.size() >= 5) {
			fill_points_position(trajectory.point_5, req->poses[5 - 1].pose.position);
			fill_points_yaw(trajectory.point_5, req->poses[5 - 1].pose.orientation);
			fill_points_unused_fields(trajectory.point_5);
		} else {
			fill_points_unused(trajectory.point_5);
		}

		// [[[end]]] (checksum: 2f3c59967b0cd4b9167720a16adc1471)


		// check that either x and y are finite or z to set the position waypoint as valid
		if ((std::isfinite(trajectory.point_1[0]) && std::isfinite(trajectory.point_1[1])) || std::isfinite(trajectory.point_1[2])) {
			trajectory.point_valid[0] = true;
		} else {
			trajectory.point_valid[0] = false;
		}

		if ((std::isfinite(trajectory.point_2[0]) && std::isfinite(trajectory.point_2[1])) || std::isfinite(trajectory.point_2[2])) {
			trajectory.point_valid[1] = true;
		} else {
			trajectory.point_valid[1] = false;
		}

		if ((std::isfinite(trajectory.point_3[0]) && std::isfinite(trajectory.point_3[1])) || std::isfinite(trajectory.point_3[2])) {
			trajectory.point_valid[2] = true;
		} else {
			trajectory.point_valid[2] = false;
		}

		if ((std::isfinite(trajectory.point_4[0]) && std::isfinite(trajectory.point_4[1])) || std::isfinite(trajectory.point_4[2])) {
			trajectory.point_valid[3] = true;
		} else {
			trajectory.point_valid[3] = false;
		}

		if ((std::isfinite(trajectory.point_5[0]) && std::isfinite(trajectory.point_5[1])) || std::isfinite(trajectory.point_5[2])) {
			trajectory.point_valid[4] = true;
		} else {
			trajectory.point_valid[4] = false;
		}

		UAS_FCU(m_uas)->send_message_ignore_drop(trajectory);
	}

	void handle_trajectory(const mavlink::mavlink_message_t *msg, mavlink::common::msg::TRAJECTORY &trajectory)
	{
		auto trajectory_desired = boost::make_shared<mavros_msgs::Trajectory>();
		trajectory_desired->header = m_uas->synchronized_header("local_origin", trajectory.time_usec);
		trajectory_desired->type = trajectory.type;						//!< trajectory type (waypoints, bezier)

		auto fill_points_position = [] (geometry_msgs::Point position, std::array<float, POINT_LEN> &point) {
			// [[[cog:
			//     cog.outl("auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[0], point[1], point[2]));".format(**locals()))
			//     for axis in "xyz":
			//         cog.outl("position.{axis} = enu_position.{axis}();".format(**locals()))
			// ]]]
			auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[0], point[1], point[2]));
			position.x = enu_position.x();
			position.y = enu_position.y();
			position.z = enu_position.z();
			// [[[end]]] (checksum: d6bf1867ed4da4c937e609d9b59fb009)
		};

		auto fill_points_velocity = [] (geometry_msgs::Vector3 velocity, std::array<float, POINT_LEN> &point) {
			// [[[cog:
			//     cog.outl("auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[3], point[4], point[5]));".format(**locals()))
			//     for axis in "xyz":
			//         cog.outl("velocity.{axis} = enu_velocity.{axis}();".format(**locals()))
			// ]]]
			auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[3], point[4], point[5]));
			velocity.x = enu_velocity.x();
			velocity.y = enu_velocity.y();
			velocity.z = enu_velocity.z();
			// [[[end]]] (checksum: 6f1a403f651e53e0edd7f440231b9875)
		};

		auto fill_points_acceleration = [] (geometry_msgs::Vector3 acceleration, std::array<float, POINT_LEN> &point) {
			// [[[cog:
			//     cog.outl("auto enu_acceleration = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[6], point[7], point[8]));".format(**locals()))
			//     for axis in "xyz":
			//         cog.outl("acceleration.{axis} = enu_acceleration.{axis}();".format(**locals()))
			// ]]]
			auto enu_acceleration = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[6], point[7], point[8]));
			acceleration.x = enu_acceleration.x();
			acceleration.y = enu_acceleration.y();
			acceleration.z = enu_acceleration.z();
			// [[[end]]] (checksum: 0365bcb69209af709be922d638b7e473)
		};

		auto fill_points_yaw = [this](float &yaw_output, float yaw_input) {
			yaw_output = wrap_pi((M_PI / 2.0f) - yaw_input);
		};

		auto fill_points_yaw_speed = [] (float yaw_speed, std::array<float, POINT_LEN> &point) {
			yaw_speed = point[10];
		};

		auto fill_points_time_horizon = [] (float time_horizon, std::array<float, POINT_LEN> &point){
			time_horizon = point[3];
		};

		auto fill_points_unused_bezier = [] (mavros_msgs::PositionTarget & trajectory){
			// [[[cog:
			// for p in ['velocity', 'acceleration_or_force']:
			//     for i in "xyz":
			//         cog.outl("trajectory.{p}.{i} = NAN;".format(**locals()))
			// cog.outl("trajectory.yaw_rate = NAN;")
			// ]]]
			trajectory.velocity.x = NAN;
			trajectory.velocity.y = NAN;
			trajectory.velocity.z = NAN;
			trajectory.acceleration_or_force.x = NAN;
			trajectory.acceleration_or_force.y = NAN;
			trajectory.acceleration_or_force.z = NAN;
			trajectory.yaw_rate = NAN;
			// [[[end]]] (checksum: 05917efa61b49a05c0180979cb02e28e)
		};

		// [[[cog:
		// for p in "12345":
		//     cog.outl("fill_points_position(trajectory_desired->point_{p}.position, trajectory.point_{p});".format(**locals()))
		// ]]]
		fill_points_position(trajectory_desired->point_1.position, trajectory.point_1);
		fill_points_position(trajectory_desired->point_2.position, trajectory.point_2);
		fill_points_position(trajectory_desired->point_3.position, trajectory.point_3);
		fill_points_position(trajectory_desired->point_4.position, trajectory.point_4);
		fill_points_position(trajectory_desired->point_5.position, trajectory.point_5);
		// [[[end]]] (checksum: 4de7872559e8d3003eecf5058df195a8)


		if (trajectory.type == utils::enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS)) {
			// [[[cog:
			// for p in "12345":
			//    cog.outl("fill_points_velocity(trajectory_desired->point_{p}.velocity, trajectory.point_{p});".format(**locals()))
			//    cog.outl("fill_points_acceleration(trajectory_desired->point_{p}.acceleration_or_force, trajectory.point_{p});".format(**locals()))
			//    cog.outl("fill_points_yaw(trajectory_desired->point_{p}.yaw, trajectory.point_{p}[9]);".format(**locals()))
			//    cog.outl("fill_points_yaw_speed(trajectory_desired->point_{p}.yaw_rate, trajectory.point_{p});\n".format(**locals()))
			// ]]]
			fill_points_velocity(trajectory_desired->point_1.velocity, trajectory.point_1);
			fill_points_acceleration(trajectory_desired->point_1.acceleration_or_force, trajectory.point_1);
			fill_points_yaw(trajectory_desired->point_1.yaw, trajectory.point_1[9]);
			fill_points_yaw_speed(trajectory_desired->point_1.yaw_rate, trajectory.point_1);

			fill_points_velocity(trajectory_desired->point_2.velocity, trajectory.point_2);
			fill_points_acceleration(trajectory_desired->point_2.acceleration_or_force, trajectory.point_2);
			fill_points_yaw(trajectory_desired->point_2.yaw, trajectory.point_2[9]);
			fill_points_yaw_speed(trajectory_desired->point_2.yaw_rate, trajectory.point_2);

			fill_points_velocity(trajectory_desired->point_3.velocity, trajectory.point_3);
			fill_points_acceleration(trajectory_desired->point_3.acceleration_or_force, trajectory.point_3);
			fill_points_yaw(trajectory_desired->point_3.yaw, trajectory.point_3[9]);
			fill_points_yaw_speed(trajectory_desired->point_3.yaw_rate, trajectory.point_3);

			fill_points_velocity(trajectory_desired->point_4.velocity, trajectory.point_4);
			fill_points_acceleration(trajectory_desired->point_4.acceleration_or_force, trajectory.point_4);
			fill_points_yaw(trajectory_desired->point_4.yaw, trajectory.point_4[9]);
			fill_points_yaw_speed(trajectory_desired->point_4.yaw_rate, trajectory.point_4);

			fill_points_velocity(trajectory_desired->point_5.velocity, trajectory.point_5);
			fill_points_acceleration(trajectory_desired->point_5.acceleration_or_force, trajectory.point_5);
			fill_points_yaw(trajectory_desired->point_5.yaw, trajectory.point_5[9]);
			fill_points_yaw_speed(trajectory_desired->point_5.yaw_rate, trajectory.point_5);

			// [[[end]]] (checksum: 645e437fe59172fc58c1efa58af7432f)
		} else {
			// [[[cog:
			// for p in "12345":
			//     cog.outl("fill_points_time_horizon(trajectory_desired->time_horizon[{p} - 1], trajectory.point_{p});".format(**locals()))
			//     cog.outl("fill_points_yaw(trajectory_desired->point_{p}.yaw, trajectory.point_{p}[4]);".format(**locals()))
			//     cog.outl("fill_points_unused_bezier(trajectory_desired->point_{p});\n".format(**locals()))
			// ]]]
			fill_points_time_horizon(trajectory_desired->time_horizon[1 - 1], trajectory.point_1);
			fill_points_yaw(trajectory_desired->point_1.yaw, trajectory.point_1[4]);
			fill_points_unused_bezier(trajectory_desired->point_1);

			fill_points_time_horizon(trajectory_desired->time_horizon[2 - 1], trajectory.point_2);
			fill_points_yaw(trajectory_desired->point_2.yaw, trajectory.point_2[4]);
			fill_points_unused_bezier(trajectory_desired->point_2);

			fill_points_time_horizon(trajectory_desired->time_horizon[3 - 1], trajectory.point_3);
			fill_points_yaw(trajectory_desired->point_3.yaw, trajectory.point_3[4]);
			fill_points_unused_bezier(trajectory_desired->point_3);

			fill_points_time_horizon(trajectory_desired->time_horizon[4 - 1], trajectory.point_4);
			fill_points_yaw(trajectory_desired->point_4.yaw, trajectory.point_4[4]);
			fill_points_unused_bezier(trajectory_desired->point_4);

			fill_points_time_horizon(trajectory_desired->time_horizon[5 - 1], trajectory.point_5);
			fill_points_yaw(trajectory_desired->point_5.yaw, trajectory.point_5[4]);
			fill_points_unused_bezier(trajectory_desired->point_5);

			// [[[end]]] (checksum: 7d946bde5b2e3e1dd663562f7a541d52)
		}

		std::copy(trajectory.point_valid.begin(), trajectory.point_valid.end(), trajectory_desired->point_valid.begin());

		trajectory_desired_pub.publish(trajectory_desired);
	}

	float wrap_pi(float a)
	{
		if (!std::isfinite(a)) {
			return a;
		}

		return fmod(a + M_PI, 2.0f * M_PI) - M_PI;
	}
};
}					// namespace extra_plugins
}				// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TrajectoryPlugin, mavros::plugin::PluginBase)
