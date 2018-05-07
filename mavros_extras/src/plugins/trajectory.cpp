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

namespace mavros {
namespace extra_plugins {
using utils::enum_value;

//! Mavlink MAV_TRAJECTORY_REPRESENTATION enumeration
using mavlink::common::MAV_TRAJECTORY_REPRESENTATION;

//! Point array LENGTH
static constexpr size_t POINT_LEN = 11;

//! Points count in TRAJECTORY message
static constexpr size_t NUM_POINTS = 5;

//! Type matching mavlink::common::msg::TRAJECTORY::point_1 field
using MavPoints = std::array<float, POINT_LEN>;

using RosPoints = mavros_msgs::PositionTarget;

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

	// [[[cog:
	// def outl_fill_points_ned_vector(vec_name, vec_type, point_xyz):
	//     cog.outl(
	//         """void fill_points_{vec_name}(MavPoints &point, const geometry_msgs::{vec_type} &{vec_name})\n"""
	//         """{{\n"""
	//         """\tauto {vec_name}_ned = ftf::transform_frame_enu_ned(ftf::to_eigen({vec_name}));\n"""
	//         .format(**locals())
	//     )
	//
	//     for index, axis in zip(point_xyz, "xyz"):
	//         cog.outl("\tpoint[{index}] = {vec_name}_ned.{axis}();".format(**locals()))
	//
	//     cog.outl("}\n")
	//
	//
	// outl_fill_points_ned_vector('position', 'Point', range(0, 3))
	// outl_fill_points_ned_vector('velocity', 'Vector3', range(3, 6))
	// outl_fill_points_ned_vector('acceleration', 'Vector3', range(6, 9))
	// ]]]
	void fill_points_position(MavPoints &point, const geometry_msgs::Point &position)
	{
		auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));

		point[0] = position_ned.x();
		point[1] = position_ned.y();
		point[2] = position_ned.z();
	}

	void fill_points_velocity(MavPoints &point, const geometry_msgs::Vector3 &velocity)
	{
		auto velocity_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(velocity));

		point[3] = velocity_ned.x();
		point[4] = velocity_ned.y();
		point[5] = velocity_ned.z();
	}

	void fill_points_acceleration(MavPoints &point, const geometry_msgs::Vector3 &acceleration)
	{
		auto acceleration_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(acceleration));

		point[6] = acceleration_ned.x();
		point[7] = acceleration_ned.y();
		point[8] = acceleration_ned.z();
	}

	// [[[end]]] (checksum: a52fa0c2f8d4abd9e49dc437a3f60890)

	void fill_points_yaw_wp(MavPoints &point, const double yaw) {
		point[9] = wrap_pi(-yaw + (M_PI / 2.0f));
	}

	void fill_points_yaw_bezier(MavPoints &point, const double yaw) {
		point[4] = wrap_pi(-yaw + (M_PI / 2.0f));
	}

	void fill_points_yaw_speed(MavPoints &point, const double yaw_speed) {
		point[10] = yaw_speed;
	}

	void fill_points_yaw_q(MavPoints &point, const geometry_msgs::Quaternion &orientation) {
		auto q_wp = ftf::transform_orientation_enu_ned(
				ftf::transform_orientation_baselink_aircraft(
					ftf::to_eigen(orientation)));
		auto yaw_wp = ftf::quaternion_get_yaw(q_wp);

		point[9] = wrap_pi(-yaw_wp + (M_PI / 2.0f));
	}

	void fill_points_time_horizon(MavPoints &point, const float time_horizon) {
		point[3] = time_horizon;
	}

	void fill_points_unused_bezier(MavPoints &point) {
		std::fill(point.begin() + 5, point.end(), NAN);
	}

	auto fill_points_unused_path(MavPoints &point) {
		std::fill(point.begin() + 3, point.begin() + 8, NAN);
		point[10] = NAN;
	}

	void fill_points_all_unused(MavPoints &point) {
		point.fill(NAN);
	}

	/**
	 * @brief Send corrected path to the FCU.
	 *
	 * Message specification: http://mavlink.org/messages/common#TRAJECTORY
	 * @param req	received Trajectory msg
	 */
	void trajectory_cb(const mavros_msgs::Trajectory::ConstPtr &req)
	{
		ROS_ASSERT(NUM_POINTS == req->point_valid.size());

		mavlink::common::msg::TRAJECTORY trajectory {};

		auto fill_point = [&](MavPoints &p, const RosPoints &rp, const size_t i) {
			const auto valid = req->point_valid[i];

			trajectory.point_valid[i] = valid;
			if (!valid) {
				fill_points_all_unused(p);
				return;
			}

			fill_points_position(p, rp.position);

			switch (req->type) {
			case enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS):
				fill_points_velocity(p, rp.velocity);
				fill_points_acceleration(p, rp.acceleration_or_force);
				fill_points_yaw_wp(p, rp.yaw);
				fill_points_yaw_speed(p, rp.yaw_rate);
				break;

			case enum_value(MAV_TRAJECTORY_REPRESENTATION::BEZIER):
				fill_points_time_horizon(p, req->time_horizon[i]);
				fill_points_yaw_bezier(p, rp.yaw);
				fill_points_unused_bezier(p);
				break;

			default:
				ROS_BREAK();
			}
		};

		trajectory.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
		trajectory.type = req->type;	//!< trajectory type (waypoints, bezier)

		// [[[cog:
		// for i in range(5):
		// 	cog.outl(
		// 	    'fill_point(trajectory.point_{i1}, req->point_{i1}, {i0});'
		// 	    .format(i0=i, i1=i+1, )
		// 	)
		// ]]]
		fill_point(trajectory.point_1, req->point_1, 0);
		fill_point(trajectory.point_2, req->point_2, 1);
		fill_point(trajectory.point_3, req->point_3, 2);
		fill_point(trajectory.point_4, req->point_4, 3);
		fill_point(trajectory.point_5, req->point_5, 4);
		// [[[end]]] (checksum: 00b771d18f91f8381cb82232237ccc48)

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

		auto fill_point = [&](MavPoints &p, const size_t i) {
			if (req->poses.size() < i + 1) {
				fill_points_all_unused(p);
			}
			else {
				auto &pose = req->poses[i].pose;

				fill_points_position(p, pose.position);
				fill_points_yaw_q(p, pose.orientation);
				fill_points_unused_path(p);
			}

			// check that either x and y are finite or z to
			// set the position waypoint as valid
			trajectory.point_valid[i] = (std::isfinite(p[0]) && std::isfinite(p[1]))
				|| std::isfinite(p[2]);
		};

		// [[[cog:
		// for i in range(5):
		// 	cog.outl(
		// 	    'fill_point(trajectory.point_{i1}, {i0});'
		// 	    .format(i0=i, i1=i+1, )
		// 	)
		// ]]]
		fill_point(trajectory.point_1, 0);
		fill_point(trajectory.point_2, 1);
		fill_point(trajectory.point_3, 2);
		fill_point(trajectory.point_4, 3);
		fill_point(trajectory.point_5, 4);
		// [[[end]]] (checksum: 4a3db9cd9b44a39a5adeb2f2800ab4f0)

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
