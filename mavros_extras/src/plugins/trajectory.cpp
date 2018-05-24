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

	// [[[cog:
	// def outl_fill_msg_enu_vector(vec_name, vec_type, point_xyz):
	//     paxes = ', '.join('point[%s]' % index for index in point_xyz)
	//
	//     cog.outl(
	//         """void fill_msg_{vec_name}(geometry_msgs::{vec_type} &{vec_name}, const MavPoints &point)\n"""
	//         """{{\n"""
	//         """\tauto {vec_name}_enu = ftf::transform_frame_ned_enu(Eigen::Vector3d({paxes}));\n"""
	//         .format(**locals())
	//     )
	//
	//     for axis in "xyz":
	//         cog.outl("\t{vec_name}.{axis} = {vec_name}_enu.{axis}();".format(**locals()))
	//
	//     cog.outl("}\n")
	//
	//
	// outl_fill_msg_enu_vector('position', 'Point', range(0, 3))
	// outl_fill_msg_enu_vector('velocity', 'Vector3', range(3, 6))
	// outl_fill_msg_enu_vector('acceleration', 'Vector3', range(6, 9))
	// ]]]
	void fill_msg_position(geometry_msgs::Point &position, const MavPoints &point)
	{
		auto position_enu = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[0], point[1], point[2]));

		position.x = position_enu.x();
		position.y = position_enu.y();
		position.z = position_enu.z();
	}

	void fill_msg_velocity(geometry_msgs::Vector3 &velocity, const MavPoints &point)
	{
		auto velocity_enu = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[3], point[4], point[5]));

		velocity.x = velocity_enu.x();
		velocity.y = velocity_enu.y();
		velocity.z = velocity_enu.z();
	}

	void fill_msg_acceleration(geometry_msgs::Vector3 &acceleration, const MavPoints &point)
	{
		auto acceleration_enu = ftf::transform_frame_ned_enu(Eigen::Vector3d(point[6], point[7], point[8]));

		acceleration.x = acceleration_enu.x();
		acceleration.y = acceleration_enu.y();
		acceleration.z = acceleration_enu.z();
	}

	// [[[end]]] (checksum: db51bf161773b227f24f716dbbc51148)

	void fill_msg_unused_bezier(RosPoints &point)
	{
		point.yaw_rate = NAN;

		// [[[cog:
		// for p in ('velocity', 'acceleration_or_force', ):
		//     for i in "xyz":
		//         cog.outl("point.{p}.{i} = NAN;".format(**locals()))
		// ]]]
		point.velocity.x = NAN;
		point.velocity.y = NAN;
		point.velocity.z = NAN;
		point.acceleration_or_force.x = NAN;
		point.acceleration_or_force.y = NAN;
		point.acceleration_or_force.z = NAN;
		// [[[end]]] (checksum: 9a1c840a592e2d44ca758d891734ca21)
	}

	// -*- callbacks -*-

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
		auto tr_desired = boost::make_shared<mavros_msgs::Trajectory>();

		auto fill_msg_point = [&](RosPoints &p, const MavPoints &mp, const size_t i) {
			fill_msg_position(p.position, mp);

			switch (trajectory.type) {
			case enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS):
				fill_msg_velocity(p.velocity, mp);
				fill_msg_acceleration(p.acceleration_or_force, mp);
				p.yaw = wrap_pi((M_PI / 2.0f) - mp[9]);
				p.yaw_rate = mp[10];
				break;

			case enum_value(MAV_TRAJECTORY_REPRESENTATION::BEZIER):
				p.yaw = mp[4];
				tr_desired->time_horizon[i] = mp[3];
				fill_msg_unused_bezier(p);
				break;

			default:
				ROS_BREAK();
			}
		};

		tr_desired->header = m_uas->synchronized_header("local_origin", trajectory.time_usec);
		tr_desired->type = trajectory.type;	//!< trajectory type (waypoints, bezier)
		std::copy(trajectory.point_valid.begin(), trajectory.point_valid.end(), tr_desired->point_valid.begin());

		// [[[cog:
		// for i in range(5):
		//     cog.outl(
		//         "fill_msg_point(tr_desired->point_{i1}, trajectory.point_{i1}, {i0});"
		//         .format(i0=i, i1=i + 1, )
		//     )
		// ]]]
		fill_msg_point(tr_desired->point_1, trajectory.point_1, 0);
		fill_msg_point(tr_desired->point_2, trajectory.point_2, 1);
		fill_msg_point(tr_desired->point_3, trajectory.point_3, 2);
		fill_msg_point(tr_desired->point_4, trajectory.point_4, 3);
		fill_msg_point(tr_desired->point_5, trajectory.point_5, 4);
		// [[[end]]] (checksum: 3f1e46c82879c6445f1c58716dec46f9)

		trajectory_desired_pub.publish(tr_desired);
	}

	float wrap_pi(float a)
	{
		if (!std::isfinite(a)) {
			return a;
		}

		return fmod(a + M_PI, 2.0f * M_PI) - M_PI;
	}
};

}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TrajectoryPlugin, mavros::plugin::PluginBase)
