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

//! Points count in TRAJECTORY message
static constexpr size_t NUM_POINTS = 5;

//! Type matching mavlink::common::msg::TRAJECTORY::TRAJECTORY_REPRESENTATION_WAYPOINTS fields
using MavPoints = std::array<float, NUM_POINTS>;

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

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		trajectory_generated_sub = trajectory_nh.subscribe("generated", 10, &TrajectoryPlugin::trajectory_cb, this);
		path_sub = trajectory_nh.subscribe("path", 10, &TrajectoryPlugin::path_cb, this);
		trajectory_desired_pub = trajectory_nh.advertise<mavros_msgs::Trajectory>("desired", 10);
	}

	Subscriptions get_subscriptions() override
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
	// def outl_fill_points_ned_vector(x, y, z, vec_name, vec_type, point_xyz):
	//     cog.outl(
	//         """void fill_points_{vec_name}(MavPoints &{x}, MavPoints &{y}, MavPoints &{z}, const geometry_msgs::{vec_type} &{vec_name}, const size_t i)\n"""
	//         """{{\n"""
	//         """\tauto {vec_name}_ned = ftf::transform_frame_enu_ned(ftf::to_eigen({vec_name}));\n"""
	//         .format(**locals())
	//     )
	//
	//     for axis in "xyz":
	//         cog.outl("\t{axis}[i] = {vec_name}_ned.{axis}();".format(**locals()))
	//
	//     cog.outl("}\n")
	//
	//
	// outl_fill_points_ned_vector('x', 'y', 'z', 'position', 'Point', range(0, 3))
	// outl_fill_points_ned_vector('x', 'y', 'z', 'velocity', 'Vector3', range(3, 6))
	// outl_fill_points_ned_vector('x', 'y', 'z', 'acceleration', 'Vector3', range(6, 9))
	// ]]]
	void fill_points_position(MavPoints &x, MavPoints &y, MavPoints &z, const geometry_msgs::Point &position, const size_t i)
	{
		auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));

		x[i] = position_ned.x();
		y[i] = position_ned.y();
		z[i] = position_ned.z();
	}

	void fill_points_velocity(MavPoints &x, MavPoints &y, MavPoints &z, const geometry_msgs::Vector3 &velocity, const size_t i)
	{
		auto velocity_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(velocity));

		x[i] = velocity_ned.x();
		y[i] = velocity_ned.y();
		z[i] = velocity_ned.z();
	}

	void fill_points_acceleration(MavPoints &x, MavPoints &y, MavPoints &z, const geometry_msgs::Vector3 &acceleration, const size_t i)
	{
		auto acceleration_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(acceleration));

		x[i] = acceleration_ned.x();
		y[i] = acceleration_ned.y();
		z[i] = acceleration_ned.z();
	}

	// [[[end]]] (checksum: 92ea0f7f329c90c486fdb8b738c0e7c3)


	void fill_points_yaw_wp(MavPoints &y, const double yaw, const size_t i) {
		y[i] = wrap_pi(-yaw + (M_PI / 2.0f));
	}

	void fill_points_yaw_speed(MavPoints &yv, const double yaw_speed, const size_t i) {
		yv[i] = yaw_speed;
	}

	void fill_points_yaw_q(MavPoints &y, const geometry_msgs::Quaternion &orientation, const size_t i) {
		auto q_wp = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(
						ftf::to_eigen(orientation)));
		auto yaw_wp = ftf::quaternion_get_yaw(q_wp);

		y[i] = wrap_pi(-yaw_wp + (M_PI / 2.0f));
	}

	void fill_points_delta(MavPoints &y, const double time_horizon, const size_t i) {
		y[i] = time_horizon;
	}

	auto fill_points_unused_path(mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t, const size_t i) {
		t.vel_x[i] = NAN;
		t.vel_y[i] = NAN;
		t.vel_z[i] = NAN;
		t.acc_x[i] = NAN;
		t.acc_y[i] = NAN;
		t.acc_z[i] = NAN;
		t.vel_yaw[i] = NAN;
	}

	void fill_points_all_unused(mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS &t, const size_t i) {
		t.pos_x[i] = NAN;
		t.pos_y[i] = NAN;
		t.pos_z[i] = NAN;

		t.vel_x[i] = NAN;
		t.vel_y[i] = NAN;
		t.vel_z[i] = NAN;

		t.acc_x[i] = NAN;
		t.acc_y[i] = NAN;
		t.acc_z[i] = NAN;

		t.pos_yaw[i] = NAN;
		t.vel_yaw[i] = NAN;
	}

	void fill_points_all_unused_bezier(mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER &t, const size_t i) {
		t.pos_x[i] = NAN;
		t.pos_y[i] = NAN;
		t.pos_z[i] = NAN;

		t.pos_yaw[i] = NAN;

		t.delta[i] = NAN;
	}

	void fill_msg_position(geometry_msgs::Point &position, const mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS &t, const size_t i)
	{
		auto position_enu = ftf::transform_frame_ned_enu(Eigen::Vector3d(t.pos_x[i], t.pos_y[i], t.pos_z[i]));

		position.x = position_enu.x();
		position.y = position_enu.y();
		position.z = position_enu.z();
	}

	void fill_msg_velocity(geometry_msgs::Vector3 &velocity, const mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS &t, const size_t i)
	{
		auto velocity_enu = ftf::transform_frame_ned_enu(Eigen::Vector3d(t.vel_x[i], t.vel_y[i], t.vel_z[i]));

		velocity.x = velocity_enu.x();
		velocity.y = velocity_enu.y();
		velocity.z = velocity_enu.z();
	}

	void fill_msg_acceleration(geometry_msgs::Vector3 &acceleration, const mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS &t, const size_t i)
	{
		auto acceleration_enu = ftf::transform_frame_ned_enu(Eigen::Vector3d(t.acc_x[i], t.acc_y[i], t.acc_z[i]));

		acceleration.x = acceleration_enu.x();
		acceleration.y = acceleration_enu.y();
		acceleration.z = acceleration_enu.z();
	}


	// -*- callbacks -*-

	/**
	 * @brief Send corrected path to the FCU.
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#TRAJECTORY
	 * @param req	received Trajectory msg
	 */
	void trajectory_cb(const mavros_msgs::Trajectory::ConstPtr &req)
	{
		ROS_ASSERT(NUM_POINTS == req->point_valid.size());

		if (req->type == mavros_msgs::Trajectory::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS) {
			mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS trajectory {};

			auto fill_point_rep_waypoints = [&](mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t, const RosPoints &rp, const size_t i) {
				const auto valid = req->point_valid[i];

				auto valid_so_far = trajectory.valid_points;
				if (!valid) {
					fill_points_all_unused(t, i);
					return;
				}

				trajectory.valid_points = valid_so_far + 1;
				fill_points_position(t.pos_x, t.pos_y, t.pos_z, rp.position, i);
				fill_points_velocity(t.vel_x, t.vel_y, t.vel_z, rp.velocity, i);
				fill_points_acceleration(t.acc_x, t.acc_y, t.acc_z, rp.acceleration_or_force, i);
				fill_points_yaw_wp(t.pos_yaw, rp.yaw, i);
				fill_points_yaw_speed(t.vel_yaw, rp.yaw_rate, i);
				t.command[i] = UINT16_MAX;
			};

			// [[[cog:
			// for i in range(5):
			//      cog.outl(
			//          'fill_point_rep_waypoints(trajectory, req->point_{i1}, {i0});'
			//          .format(i0=i, i1=i+1)
			//      )
			// ]]]
			fill_point_rep_waypoints(trajectory, req->point_1, 0);
			fill_point_rep_waypoints(trajectory, req->point_2, 1);
			fill_point_rep_waypoints(trajectory, req->point_3, 2);
			fill_point_rep_waypoints(trajectory, req->point_4, 3);
			fill_point_rep_waypoints(trajectory, req->point_5, 4);
			// [[[end]]] (checksum: e993aeb535c2df6f07bf7b4f1fcf3d2e)

			trajectory.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
			UAS_FCU(m_uas)->send_message_ignore_drop(trajectory);
		} else {
			mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER trajectory {};
			auto fill_point_rep_bezier = [&](mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER & t, const RosPoints &rp, const size_t i) {
				const auto valid = req->point_valid[i];

				auto valid_so_far = trajectory.valid_points;
				if (!valid) {
					fill_points_all_unused_bezier(t, i);
					return;
				}

				trajectory.valid_points = valid_so_far + 1;
				fill_points_position(t.pos_x, t.pos_y, t.pos_z, rp.position, i);
				fill_points_yaw_wp(t.pos_yaw, rp.yaw, i);
				fill_points_delta(t.delta, req->time_horizon[i], i);
			};
			// [[[cog:
			// for i in range(5):
			//      cog.outl(
			//          'fill_point_rep_bezier(trajectory, req->point_{i1}, {i0});'
			//          .format(i0=i, i1=i+1)
			//      )
			// ]]]
			fill_point_rep_bezier(trajectory, req->point_1, 0);
			fill_point_rep_bezier(trajectory, req->point_2, 1);
			fill_point_rep_bezier(trajectory, req->point_3, 2);
			fill_point_rep_bezier(trajectory, req->point_4, 3);
			fill_point_rep_bezier(trajectory, req->point_5, 4);
			// [[[end]]] (checksum: 3e6da5f06e0b33682c6122c40f05c1f6)

			trajectory.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
			UAS_FCU(m_uas)->send_message_ignore_drop(trajectory);
		}


	}


	/**
	 * @brief Send corrected path to the FCU.
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#TRAJECTORY
	 * @param req	received nav_msgs Path msg
	 */
	void path_cb(const nav_msgs::Path::ConstPtr &req)
	{
		mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS trajectory {};

		trajectory.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
		trajectory.valid_points = std::min(NUM_POINTS, req->poses.size());

		auto fill_point = [&](mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t, const size_t i) {
			t.command[i] = UINT16_MAX;
			if (req->poses.size() < i + 1) {
				fill_points_all_unused(t, i);
			}
			else {
				auto &pose = req->poses[i].pose;

				fill_points_position(t.pos_x, t.pos_y, t.pos_z, pose.position, i);
				fill_points_yaw_q(t.pos_yaw, pose.orientation, i);
				fill_points_unused_path(t, i);
			}
		};

		// [[[cog:
		// for i in range(5):
		//      cog.outl(
		//          'fill_point(trajectory, {i0});'.format(i0=i, i1=i+1))
		// ]]]
		fill_point(trajectory, 0);
		fill_point(trajectory, 1);
		fill_point(trajectory, 2);
		fill_point(trajectory, 3);
		fill_point(trajectory, 4);
		// [[[end]]] (checksum: 267a911c65a3768f04a8230fcb235bca)

		UAS_FCU(m_uas)->send_message_ignore_drop(trajectory);
	}

	void handle_trajectory(const mavlink::mavlink_message_t *msg, mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS &trajectory)
	{
		auto tr_desired = boost::make_shared<mavros_msgs::Trajectory>();

		auto fill_msg_point = [&](RosPoints & p, const mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t, const size_t i) {
			fill_msg_position(p.position, t, i);
			fill_msg_velocity(p.velocity, t, i);
			fill_msg_acceleration(p.acceleration_or_force, t, i);
			p.yaw = wrap_pi((M_PI / 2.0f) - t.pos_yaw[i]);
			p.yaw_rate = t.vel_yaw[i];
			tr_desired->command[i] = t.command[i];
		};

		tr_desired->header = m_uas->synchronized_header("local_origin", trajectory.time_usec);

		for (int i = 0; i < trajectory.valid_points; ++i)
		{
			tr_desired->point_valid[i] = true;
		}

		for (int i = trajectory.valid_points; i < NUM_POINTS; ++i)
		{
			tr_desired->point_valid[i] = false;
		}

		// [[[cog:
		// for i in range(5):
		//     cog.outl(
		//         "fill_msg_point(tr_desired->point_{i1}, trajectory, {i0});"
		//         .format(i0=i, i1=i + 1, )
		//     )
		// ]]]
		fill_msg_point(tr_desired->point_1, trajectory, 0);
		fill_msg_point(tr_desired->point_2, trajectory, 1);
		fill_msg_point(tr_desired->point_3, trajectory, 2);
		fill_msg_point(tr_desired->point_4, trajectory, 3);
		fill_msg_point(tr_desired->point_5, trajectory, 4);
		// [[[end]]] (checksum: b6aa0c7395a7a426c25d726b75b6efea)

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
