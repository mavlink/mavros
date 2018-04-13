/**
 * @brief Trajectory plugin
 * @file trajectory.cpp
 * @author Martina Rivizzigno <martina@rivizzigno.com>
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

		/* Transformation from ENU to NED */
		// [[[cog:
		// for p in "12345":
		//     cog.outl("auto position_point_{p} = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_{p}.position));".format(**locals()))
		//     for index, axis in zip ("012", "xyz"):
		//         cog.outl("trajectory.point_{p}[{index}] = position_point_{p}.{axis}();".format(**locals()))
		// ]]]
		auto position_point_1 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_1.position));
		trajectory.point_1[0] = position_point_1.x();
		trajectory.point_1[1] = position_point_1.y();
		trajectory.point_1[2] = position_point_1.z();
		auto position_point_2 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_2.position));
		trajectory.point_2[0] = position_point_2.x();
		trajectory.point_2[1] = position_point_2.y();
		trajectory.point_2[2] = position_point_2.z();
		auto position_point_3 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_3.position));
		trajectory.point_3[0] = position_point_3.x();
		trajectory.point_3[1] = position_point_3.y();
		trajectory.point_3[2] = position_point_3.z();
		auto position_point_4 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_4.position));
		trajectory.point_4[0] = position_point_4.x();
		trajectory.point_4[1] = position_point_4.y();
		trajectory.point_4[2] = position_point_4.z();
		auto position_point_5 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_5.position));
		trajectory.point_5[0] = position_point_5.x();
		trajectory.point_5[1] = position_point_5.y();
		trajectory.point_5[2] = position_point_5.z();
		// [[[end]]] (checksum: 426c18b59c8fa9b5842fd88741c6df2e)

		if (req->type == utils::enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS)) {
			/* Transformation from ENU to NED */
			//[[[cog:
			//for p in "12345":
			//     cog.outl("auto velocity_point_{p} = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_{p}.velocity));".format(**locals()))
			//     for index, axis in zip ("345", "xyz"):
			//         cog.outl("trajectory.point_{p}[{index}] = velocity_point_{p}.{axis}();".format(**locals()))
			//     cog.outl("auto acceleration_point_{p} = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_{p}.acceleration_or_force));".format(**locals()))
			//     for index, axis in zip ("678", "xyz"):
			//         cog.outl("trajectory.point_{p}[{index}] = acceleration_point_{p}.{axis}();".format(**locals()))
			//     cog.outl("double yaw_ned_point_{p} = wrap_pi(-req->point_{p}.yaw + (M_PI / 2.0f));".format(**locals()))
			//     cog.outl("trajectory.point_{p}[9] = yaw_ned_point_{p};".format(**locals()))
			//     cog.outl("trajectory.point_{p}[10] = req->point_{p}.yaw_rate; \n".format(**locals()))
			// ]]]
			auto velocity_point_1 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_1.velocity));
			trajectory.point_1[3] = velocity_point_1.x();
			trajectory.point_1[4] = velocity_point_1.y();
			trajectory.point_1[5] = velocity_point_1.z();
			auto acceleration_point_1 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_1.acceleration_or_force));
			trajectory.point_1[6] = acceleration_point_1.x();
			trajectory.point_1[7] = acceleration_point_1.y();
			trajectory.point_1[8] = acceleration_point_1.z();
			double yaw_ned_point_1 = wrap_pi(-req->point_1.yaw + (M_PI / 2.0f));
			trajectory.point_1[9] = yaw_ned_point_1;
			trajectory.point_1[10] = req->point_1.yaw_rate;

			auto velocity_point_2 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_2.velocity));
			trajectory.point_2[3] = velocity_point_2.x();
			trajectory.point_2[4] = velocity_point_2.y();
			trajectory.point_2[5] = velocity_point_2.z();
			auto acceleration_point_2 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_2.acceleration_or_force));
			trajectory.point_2[6] = acceleration_point_2.x();
			trajectory.point_2[7] = acceleration_point_2.y();
			trajectory.point_2[8] = acceleration_point_2.z();
			double yaw_ned_point_2 = wrap_pi(-req->point_2.yaw + (M_PI / 2.0f));
			trajectory.point_2[9] = yaw_ned_point_2;
			trajectory.point_2[10] = req->point_2.yaw_rate;

			auto velocity_point_3 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_3.velocity));
			trajectory.point_3[3] = velocity_point_3.x();
			trajectory.point_3[4] = velocity_point_3.y();
			trajectory.point_3[5] = velocity_point_3.z();
			auto acceleration_point_3 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_3.acceleration_or_force));
			trajectory.point_3[6] = acceleration_point_3.x();
			trajectory.point_3[7] = acceleration_point_3.y();
			trajectory.point_3[8] = acceleration_point_3.z();
			double yaw_ned_point_3 = wrap_pi(-req->point_3.yaw + (M_PI / 2.0f));
			trajectory.point_3[9] = yaw_ned_point_3;
			trajectory.point_3[10] = req->point_3.yaw_rate;

			auto velocity_point_4 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_4.velocity));
			trajectory.point_4[3] = velocity_point_4.x();
			trajectory.point_4[4] = velocity_point_4.y();
			trajectory.point_4[5] = velocity_point_4.z();
			auto acceleration_point_4 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_4.acceleration_or_force));
			trajectory.point_4[6] = acceleration_point_4.x();
			trajectory.point_4[7] = acceleration_point_4.y();
			trajectory.point_4[8] = acceleration_point_4.z();
			double yaw_ned_point_4 = wrap_pi(-req->point_4.yaw + (M_PI / 2.0f));
			trajectory.point_4[9] = yaw_ned_point_4;
			trajectory.point_4[10] = req->point_4.yaw_rate;

			auto velocity_point_5 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_5.velocity));
			trajectory.point_5[3] = velocity_point_5.x();
			trajectory.point_5[4] = velocity_point_5.y();
			trajectory.point_5[5] = velocity_point_5.z();
			auto acceleration_point_5 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_5.acceleration_or_force));
			trajectory.point_5[6] = acceleration_point_5.x();
			trajectory.point_5[7] = acceleration_point_5.y();
			trajectory.point_5[8] = acceleration_point_5.z();
			double yaw_ned_point_5 = wrap_pi(-req->point_5.yaw + (M_PI / 2.0f));
			trajectory.point_5[9] = yaw_ned_point_5;
			trajectory.point_5[10] = req->point_5.yaw_rate;

			// [[[end]]] (checksum: c19ffbd7dee05e9242b465c7642c3656)
		} else {
			//[[[cog:
			//for p in "12345":
			//     cog.outl("trajectory.point_{p}[3] = req->point_{p}.velocity.x;".format(**locals()))
			//     cog.outl("double yaw_ned_point_{p} = wrap_pi(-req->point_{p}.yaw + (M_PI / 2.0f)); \n".format(**locals()))
			//     cog.outl("trajectory.point_{p}[4] = yaw_ned_point_{p};".format(**locals()))
			//     cog.outl("trajectory.point_{p}[5] = req->point_{p}.yaw_rate; \n".format(**locals()))
			// ]]]
			trajectory.point_1[3] = req->point_1.velocity.x;
			double yaw_ned_point_1 = wrap_pi(-req->point_1.yaw + (M_PI / 2.0f));

			trajectory.point_1[4] = yaw_ned_point_1;
			trajectory.point_1[5] = req->point_1.yaw_rate;

			trajectory.point_2[3] = req->point_2.velocity.x;
			double yaw_ned_point_2 = wrap_pi(-req->point_2.yaw + (M_PI / 2.0f));

			trajectory.point_2[4] = yaw_ned_point_2;
			trajectory.point_2[5] = req->point_2.yaw_rate;

			trajectory.point_3[3] = req->point_3.velocity.x;
			double yaw_ned_point_3 = wrap_pi(-req->point_3.yaw + (M_PI / 2.0f));

			trajectory.point_3[4] = yaw_ned_point_3;
			trajectory.point_3[5] = req->point_3.yaw_rate;

			trajectory.point_4[3] = req->point_4.velocity.x;
			double yaw_ned_point_4 = wrap_pi(-req->point_4.yaw + (M_PI / 2.0f));

			trajectory.point_4[4] = yaw_ned_point_4;
			trajectory.point_4[5] = req->point_4.yaw_rate;

			trajectory.point_5[3] = req->point_5.velocity.x;
			double yaw_ned_point_5 = wrap_pi(-req->point_5.yaw + (M_PI / 2.0f));

			trajectory.point_5[4] = yaw_ned_point_5;
			trajectory.point_5[5] = req->point_5.yaw_rate;

			// [[[end]]] (checksum: 85dad0e2b627bea8a29668db7eabb610)
		}

		std::copy(req->point_valid.begin(), req->point_valid.end(), trajectory.point_valid.begin());

		UAS_FCU(m_uas)->send_message_ignore_drop(trajectory);
	}



	void handle_trajectory(const mavlink::mavlink_message_t *msg, mavlink::common::msg::TRAJECTORY &trajectory)
	{
		auto trajectory_desired = boost::make_shared<mavros_msgs::Trajectory>();
		trajectory_desired->header = m_uas->synchronized_header("local_origin", trajectory.time_usec);
		trajectory_desired->type = trajectory.type;	//!< trajectory type (waypoints, bezier)


		/* Transformation from NED to ENU */
		// [[[cog:
		// for p in "12345":
		//     cog.outl("auto enu_position_point_{p} = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_{p}[0], trajectory.point_{p}[1], trajectory.point_{p}[2]));".format(**locals()))
		//     cog.outl("auto enu_velocity_point_{p} = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_{p}[3], trajectory.point_{p}[4], trajectory.point_{p}[5]));".format(**locals()))
		//     cog.outl("auto enu_acceleration_point_{p} = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_{p}[6], trajectory.point_{p}[7], trajectory.point_{p}[8]));".format(**locals()))
		//     cog.outl("double yaw_enu_point_{p} = wrap_pi((M_PI / 2.0f) - trajectory.point_{p}[9]); \n".format(**locals()))
		// ]]]
		auto enu_position_point_1 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_1[0], trajectory.point_1[1], trajectory.point_1[2]));
		auto enu_velocity_point_1 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_1[3], trajectory.point_1[4], trajectory.point_1[5]));
		auto enu_acceleration_point_1 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_1[6], trajectory.point_1[7], trajectory.point_1[8]));
		double yaw_enu_point_1 = wrap_pi((M_PI / 2.0f) - trajectory.point_1[9]);

		auto enu_position_point_2 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_2[0], trajectory.point_2[1], trajectory.point_2[2]));
		auto enu_velocity_point_2 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_2[3], trajectory.point_2[4], trajectory.point_2[5]));
		auto enu_acceleration_point_2 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_2[6], trajectory.point_2[7], trajectory.point_2[8]));
		double yaw_enu_point_2 = wrap_pi((M_PI / 2.0f) - trajectory.point_2[9]);

		auto enu_position_point_3 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_3[0], trajectory.point_3[1], trajectory.point_3[2]));
		auto enu_velocity_point_3 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_3[3], trajectory.point_3[4], trajectory.point_3[5]));
		auto enu_acceleration_point_3 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_3[6], trajectory.point_3[7], trajectory.point_3[8]));
		double yaw_enu_point_3 = wrap_pi((M_PI / 2.0f) - trajectory.point_3[9]);

		auto enu_position_point_4 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_4[0], trajectory.point_4[1], trajectory.point_4[2]));
		auto enu_velocity_point_4 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_4[3], trajectory.point_4[4], trajectory.point_4[5]));
		auto enu_acceleration_point_4 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_4[6], trajectory.point_4[7], trajectory.point_4[8]));
		double yaw_enu_point_4 = wrap_pi((M_PI / 2.0f) - trajectory.point_4[9]);

		auto enu_position_point_5 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_5[0], trajectory.point_5[1], trajectory.point_5[2]));
		auto enu_velocity_point_5 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_5[3], trajectory.point_5[4], trajectory.point_5[5]));
		auto enu_acceleration_point_5 = ftf::transform_frame_ned_enu(Eigen::Vector3d(trajectory.point_5[6], trajectory.point_5[7], trajectory.point_5[8]));
		double yaw_enu_point_5 = wrap_pi((M_PI / 2.0f) - trajectory.point_5[9]);

		// [[[end]]] (checksum: 7ea71bce399a8c7cc159a4989eb2a8c4)

		// [[[cog:
		// for p in "12345":
		//     for axis in "xyz":
		//         cog.outl("trajectory_desired->point_{p}.position.{axis} = enu_position_point_{p}.{axis}();".format(**locals()))
		//         cog.outl("trajectory_desired->point_{p}.velocity.{axis} = enu_velocity_point_{p}.{axis}();".format(**locals()))
		//         cog.outl("trajectory_desired->point_{p}.acceleration_or_force.{axis} = enu_acceleration_point_{p}.{axis}();".format(**locals()))
		//     cog.outl("trajectory_desired->point_{p}.yaw = yaw_enu_point_{p};".format(**locals()))
		//     cog.outl("trajectory_desired->point_{p}.yaw_rate = trajectory.point_{p}[10]; \n".format(**locals()))
		// ]]]
		trajectory_desired->point_1.position.x = enu_position_point_1.x();
		trajectory_desired->point_1.velocity.x = enu_velocity_point_1.x();
		trajectory_desired->point_1.acceleration_or_force.x = enu_acceleration_point_1.x();
		trajectory_desired->point_1.position.y = enu_position_point_1.y();
		trajectory_desired->point_1.velocity.y = enu_velocity_point_1.y();
		trajectory_desired->point_1.acceleration_or_force.y = enu_acceleration_point_1.y();
		trajectory_desired->point_1.position.z = enu_position_point_1.z();
		trajectory_desired->point_1.velocity.z = enu_velocity_point_1.z();
		trajectory_desired->point_1.acceleration_or_force.z = enu_acceleration_point_1.z();
		trajectory_desired->point_1.yaw = yaw_enu_point_1;
		trajectory_desired->point_1.yaw_rate = trajectory.point_1[10];

		trajectory_desired->point_2.position.x = enu_position_point_2.x();
		trajectory_desired->point_2.velocity.x = enu_velocity_point_2.x();
		trajectory_desired->point_2.acceleration_or_force.x = enu_acceleration_point_2.x();
		trajectory_desired->point_2.position.y = enu_position_point_2.y();
		trajectory_desired->point_2.velocity.y = enu_velocity_point_2.y();
		trajectory_desired->point_2.acceleration_or_force.y = enu_acceleration_point_2.y();
		trajectory_desired->point_2.position.z = enu_position_point_2.z();
		trajectory_desired->point_2.velocity.z = enu_velocity_point_2.z();
		trajectory_desired->point_2.acceleration_or_force.z = enu_acceleration_point_2.z();
		trajectory_desired->point_2.yaw = yaw_enu_point_2;
		trajectory_desired->point_2.yaw_rate = trajectory.point_2[10];

		trajectory_desired->point_3.position.x = enu_position_point_3.x();
		trajectory_desired->point_3.velocity.x = enu_velocity_point_3.x();
		trajectory_desired->point_3.acceleration_or_force.x = enu_acceleration_point_3.x();
		trajectory_desired->point_3.position.y = enu_position_point_3.y();
		trajectory_desired->point_3.velocity.y = enu_velocity_point_3.y();
		trajectory_desired->point_3.acceleration_or_force.y = enu_acceleration_point_3.y();
		trajectory_desired->point_3.position.z = enu_position_point_3.z();
		trajectory_desired->point_3.velocity.z = enu_velocity_point_3.z();
		trajectory_desired->point_3.acceleration_or_force.z = enu_acceleration_point_3.z();
		trajectory_desired->point_3.yaw = yaw_enu_point_3;
		trajectory_desired->point_3.yaw_rate = trajectory.point_3[10];

		trajectory_desired->point_4.position.x = enu_position_point_4.x();
		trajectory_desired->point_4.velocity.x = enu_velocity_point_4.x();
		trajectory_desired->point_4.acceleration_or_force.x = enu_acceleration_point_4.x();
		trajectory_desired->point_4.position.y = enu_position_point_4.y();
		trajectory_desired->point_4.velocity.y = enu_velocity_point_4.y();
		trajectory_desired->point_4.acceleration_or_force.y = enu_acceleration_point_4.y();
		trajectory_desired->point_4.position.z = enu_position_point_4.z();
		trajectory_desired->point_4.velocity.z = enu_velocity_point_4.z();
		trajectory_desired->point_4.acceleration_or_force.z = enu_acceleration_point_4.z();
		trajectory_desired->point_4.yaw = yaw_enu_point_4;
		trajectory_desired->point_4.yaw_rate = trajectory.point_4[10];

		trajectory_desired->point_5.position.x = enu_position_point_5.x();
		trajectory_desired->point_5.velocity.x = enu_velocity_point_5.x();
		trajectory_desired->point_5.acceleration_or_force.x = enu_acceleration_point_5.x();
		trajectory_desired->point_5.position.y = enu_position_point_5.y();
		trajectory_desired->point_5.velocity.y = enu_velocity_point_5.y();
		trajectory_desired->point_5.acceleration_or_force.y = enu_acceleration_point_5.y();
		trajectory_desired->point_5.position.z = enu_position_point_5.z();
		trajectory_desired->point_5.velocity.z = enu_velocity_point_5.z();
		trajectory_desired->point_5.acceleration_or_force.z = enu_acceleration_point_5.z();
		trajectory_desired->point_5.yaw = yaw_enu_point_5;
		trajectory_desired->point_5.yaw_rate = trajectory.point_5[10];

		// [[[end]]] (checksum: afadac2d81a28d6d85e5d87a5be344bf)

		std::copy(trajectory.point_valid.begin(), trajectory.point_valid.end(), trajectory_desired->point_valid.begin());

		trajectory_desired_pub.publish(trajectory_desired);
	}

	float wrap_pi(float a)
	{
		if (!std::isfinite(a)) {
			return a;
		}

		while (a >= M_PI) {
			a -= (M_PI * 2.0f);
		}

		while (a < -M_PI) {
			a += (M_PI * 2.0f);
		}

		return a;
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TrajectoryPlugin, mavros::plugin::PluginBase)
