/**
 * @brief Obstacle avoidance plugin
 * @file obstacle_avoidance.cpp
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
#include <mavros_msgs/ObstacleAvoidance.h>
#include <mavros_msgs/PositionTarget.h>

namespace mavros {
namespace extra_plugins {
//! Mavlink MAV_TRAJECTORY_REPRESENTATION enumeration
using mavlink::common::MAV_TRAJECTORY_REPRESENTATION;

/**
 * @brief Obstacle avoidance plugin to receive planned path from the FCU and
 * send back to the FCU a collision free path
 *
 * @see avoidance_cb()
 */
class ObstacleAvoidancePlugin : public plugin::PluginBase {
public:
	ObstacleAvoidancePlugin() : PluginBase(),
		avoidance_nh("~avoidance")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		avoidance_outout_sub = avoidance_nh.subscribe("output", 10, &ObstacleAvoidancePlugin::avoidance_cb, this);

		avoidance_input_pub = avoidance_nh.advertise<mavros_msgs::ObstacleAvoidance>("input", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&ObstacleAvoidancePlugin::handle_obstacle_avoidance)
		};
	}

private:
	ros::NodeHandle avoidance_nh;

	ros::Subscriber avoidance_outout_sub;

	ros::Publisher avoidance_input_pub;


	/**
	 * @brief Send collision free path to the FCU.
	 *
	 * Message specification: http://mavlink.org/messages/common#OBSTACLE_AVOIDANCE
	 * @param req	received ObstacleAvoidance msg
	 */
	void avoidance_cb(const mavros_msgs::ObstacleAvoidance::ConstPtr &req)
	{
		mavlink::common::msg::OBSTACLE_AVOIDANCE obstacle_avoidance {};
		obstacle_avoidance.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
		obstacle_avoidance.type = req->type;	//!< trajectory type (waypoints, bezier)

		/* Transformation from ENU to NED */
		// [[[cog:
		// for p in "12345":
		//     cog.outl("auto position_point_{p} = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_{p}.position));".format(**locals()))
		//     cog.outl("auto velocity_point_{p} = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_{p}.velocity));".format(**locals()))
		//     cog.outl("auto acceleration_point_{p} = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_{p}.acceleration_or_force));".format(**locals()))
		//     cog.outl("double yaw_ned_point_{p} = wrap_pi(-req->point_{p}.yaw + (M_PI / 2.0f)); \n".format(**locals()))
		// ]]]
		auto position_point_1 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_1.position));
		auto velocity_point_1 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_1.velocity));
		auto acceleration_point_1 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_1.acceleration_or_force));
		double yaw_ned_point_1 = wrap_pi(-req->point_1.yaw + (M_PI / 2.0f));

		auto position_point_2 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_2.position));
		auto velocity_point_2 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_2.velocity));
		auto acceleration_point_2 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_2.acceleration_or_force));
		double yaw_ned_point_2 = wrap_pi(-req->point_2.yaw + (M_PI / 2.0f));

		auto position_point_3 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_3.position));
		auto velocity_point_3 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_3.velocity));
		auto acceleration_point_3 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_3.acceleration_or_force));
		double yaw_ned_point_3 = wrap_pi(-req->point_3.yaw + (M_PI / 2.0f));

		auto position_point_4 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_4.position));
		auto velocity_point_4 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_4.velocity));
		auto acceleration_point_4 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_4.acceleration_or_force));
		double yaw_ned_point_4 = wrap_pi(-req->point_4.yaw + (M_PI / 2.0f));

		auto position_point_5 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_5.position));
		auto velocity_point_5 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_5.velocity));
		auto acceleration_point_5 = ftf::transform_frame_enu_ned(ftf::to_eigen(req->point_5.acceleration_or_force));
		double yaw_ned_point_5 = wrap_pi(-req->point_5.yaw + (M_PI / 2.0f));

		// [[[end]]] (checksum: 19abc4b41b7ae88227ac08fac3769bb4)


		// [[[cog:
		// for p in "12345":
		//     for index, axis in zip ("012", "xyz"):
		//         cog.outl("obstacle_avoidance.point_{p}[{index}] = position_point_{p}.{axis}();".format(**locals()))
		//     for index, axis in zip ("345", "xyz"):
		//         cog.outl("obstacle_avoidance.point_{p}[{index}] = velocity_point_{p}.{axis}();".format(**locals()))
		//     for index, axis in zip ("678", "xyz"):
		//         cog.outl("obstacle_avoidance.point_{p}[{index}] = acceleration_point_{p}.{axis}();".format(**locals()))
		//     cog.outl("obstacle_avoidance.point_{p}[9] = yaw_ned_point_{p};".format(**locals()))
		//     cog.outl("obstacle_avoidance.point_{p}[10] = req->point_{p}.yaw_rate; \n".format(**locals()))
		// ]]]
		obstacle_avoidance.point_1[0] = position_point_1.x();
		obstacle_avoidance.point_1[1] = position_point_1.y();
		obstacle_avoidance.point_1[2] = position_point_1.z();
		obstacle_avoidance.point_1[3] = velocity_point_1.x();
		obstacle_avoidance.point_1[4] = velocity_point_1.y();
		obstacle_avoidance.point_1[5] = velocity_point_1.z();
		obstacle_avoidance.point_1[6] = acceleration_point_1.x();
		obstacle_avoidance.point_1[7] = acceleration_point_1.y();
		obstacle_avoidance.point_1[8] = acceleration_point_1.z();
		obstacle_avoidance.point_1[9] = yaw_ned_point_1;
		obstacle_avoidance.point_1[10] = req->point_1.yaw_rate;

		obstacle_avoidance.point_2[0] = position_point_2.x();
		obstacle_avoidance.point_2[1] = position_point_2.y();
		obstacle_avoidance.point_2[2] = position_point_2.z();
		obstacle_avoidance.point_2[3] = velocity_point_2.x();
		obstacle_avoidance.point_2[4] = velocity_point_2.y();
		obstacle_avoidance.point_2[5] = velocity_point_2.z();
		obstacle_avoidance.point_2[6] = acceleration_point_2.x();
		obstacle_avoidance.point_2[7] = acceleration_point_2.y();
		obstacle_avoidance.point_2[8] = acceleration_point_2.z();
		obstacle_avoidance.point_2[9] = yaw_ned_point_2;
		obstacle_avoidance.point_2[10] = req->point_2.yaw_rate;

		obstacle_avoidance.point_3[0] = position_point_3.x();
		obstacle_avoidance.point_3[1] = position_point_3.y();
		obstacle_avoidance.point_3[2] = position_point_3.z();
		obstacle_avoidance.point_3[3] = velocity_point_3.x();
		obstacle_avoidance.point_3[4] = velocity_point_3.y();
		obstacle_avoidance.point_3[5] = velocity_point_3.z();
		obstacle_avoidance.point_3[6] = acceleration_point_3.x();
		obstacle_avoidance.point_3[7] = acceleration_point_3.y();
		obstacle_avoidance.point_3[8] = acceleration_point_3.z();
		obstacle_avoidance.point_3[9] = yaw_ned_point_3;
		obstacle_avoidance.point_3[10] = req->point_3.yaw_rate;

		obstacle_avoidance.point_4[0] = position_point_4.x();
		obstacle_avoidance.point_4[1] = position_point_4.y();
		obstacle_avoidance.point_4[2] = position_point_4.z();
		obstacle_avoidance.point_4[3] = velocity_point_4.x();
		obstacle_avoidance.point_4[4] = velocity_point_4.y();
		obstacle_avoidance.point_4[5] = velocity_point_4.z();
		obstacle_avoidance.point_4[6] = acceleration_point_4.x();
		obstacle_avoidance.point_4[7] = acceleration_point_4.y();
		obstacle_avoidance.point_4[8] = acceleration_point_4.z();
		obstacle_avoidance.point_4[9] = yaw_ned_point_4;
		obstacle_avoidance.point_4[10] = req->point_4.yaw_rate;

		obstacle_avoidance.point_5[0] = position_point_5.x();
		obstacle_avoidance.point_5[1] = position_point_5.y();
		obstacle_avoidance.point_5[2] = position_point_5.z();
		obstacle_avoidance.point_5[3] = velocity_point_5.x();
		obstacle_avoidance.point_5[4] = velocity_point_5.y();
		obstacle_avoidance.point_5[5] = velocity_point_5.z();
		obstacle_avoidance.point_5[6] = acceleration_point_5.x();
		obstacle_avoidance.point_5[7] = acceleration_point_5.y();
		obstacle_avoidance.point_5[8] = acceleration_point_5.z();
		obstacle_avoidance.point_5[9] = yaw_ned_point_5;
		obstacle_avoidance.point_5[10] = req->point_5.yaw_rate;

		// [[[end]]] (checksum: e4f9d45114563fb77e5cee8bfd9214c9)


		std::copy(req->point_valid.begin(), req->point_valid.end(), obstacle_avoidance.point_valid.begin());
		std::copy(req->field_of_view.begin(), req->field_of_view.end(), obstacle_avoidance.field_of_view.begin());

		UAS_FCU(m_uas)->send_message_ignore_drop(obstacle_avoidance);
	}

	void handle_obstacle_avoidance(const mavlink::mavlink_message_t *msg, mavlink::common::msg::OBSTACLE_AVOIDANCE &avoid_input)
	{
		auto obstacle_avoidance_input = boost::make_shared<mavros_msgs::ObstacleAvoidance>();
		obstacle_avoidance_input->header = m_uas->synchronized_header("local_origin", avoid_input.time_usec);
		obstacle_avoidance_input->type = avoid_input.type;	//!< trajectory type (waypoints, bezier)


		/* Transformation from NED to ENU */
		// [[[cog:
		// for p in "12345":
		//     cog.outl("auto enu_position_point_{p} = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_{p}[0], avoid_input.point_{p}[1], avoid_input.point_{p}[2]));".format(**locals()))
		//     cog.outl("auto enu_velocity_point_{p} = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_{p}[3], avoid_input.point_{p}[4], avoid_input.point_{p}[5]));".format(**locals()))
		//     cog.outl("auto enu_acceleration_point_{p} = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_{p}[6], avoid_input.point_{p}[7], avoid_input.point_{p}[8]));".format(**locals()))
		//     cog.outl("double yaw_enu_point_{p} = wrap_pi((M_PI / 2.0f) - avoid_input.point_{p}[9]); \n".format(**locals()))
		// ]]]
		auto enu_position_point_1 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_1[0], avoid_input.point_1[1], avoid_input.point_1[2]));
		auto enu_velocity_point_1 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_1[3], avoid_input.point_1[4], avoid_input.point_1[5]));
		auto enu_acceleration_point_1 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_1[6], avoid_input.point_1[7], avoid_input.point_1[8]));
		double yaw_enu_point_1 = wrap_pi((M_PI / 2.0f) - avoid_input.point_1[9]);

		auto enu_position_point_2 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_2[0], avoid_input.point_2[1], avoid_input.point_2[2]));
		auto enu_velocity_point_2 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_2[3], avoid_input.point_2[4], avoid_input.point_2[5]));
		auto enu_acceleration_point_2 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_2[6], avoid_input.point_2[7], avoid_input.point_2[8]));
		double yaw_enu_point_2 = wrap_pi((M_PI / 2.0f) - avoid_input.point_2[9]);

		auto enu_position_point_3 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_3[0], avoid_input.point_3[1], avoid_input.point_3[2]));
		auto enu_velocity_point_3 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_3[3], avoid_input.point_3[4], avoid_input.point_3[5]));
		auto enu_acceleration_point_3 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_3[6], avoid_input.point_3[7], avoid_input.point_3[8]));
		double yaw_enu_point_3 = wrap_pi((M_PI / 2.0f) - avoid_input.point_3[9]);

		auto enu_position_point_4 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_4[0], avoid_input.point_4[1], avoid_input.point_4[2]));
		auto enu_velocity_point_4 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_4[3], avoid_input.point_4[4], avoid_input.point_4[5]));
		auto enu_acceleration_point_4 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_4[6], avoid_input.point_4[7], avoid_input.point_4[8]));
		double yaw_enu_point_4 = wrap_pi((M_PI / 2.0f) - avoid_input.point_4[9]);

		auto enu_position_point_5 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_5[0], avoid_input.point_5[1], avoid_input.point_5[2]));
		auto enu_velocity_point_5 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_5[3], avoid_input.point_5[4], avoid_input.point_5[5]));
		auto enu_acceleration_point_5 = ftf::transform_frame_ned_enu(Eigen::Vector3d(avoid_input.point_5[6], avoid_input.point_5[7], avoid_input.point_5[8]));
		double yaw_enu_point_5 = wrap_pi((M_PI / 2.0f) - avoid_input.point_5[9]);

		// [[[end]]] (checksum: d565266b032616702bd89161253302ed)

		// [[[cog:
		// for p in "12345":
		//     for axis in "xyz":
		//         cog.outl("obstacle_avoidance_input->point_{p}.position.{axis} = enu_position_point_{p}.{axis}();".format(**locals()))
		//         cog.outl("obstacle_avoidance_input->point_{p}.velocity.{axis} = enu_velocity_point_{p}.{axis}();".format(**locals()))
		//         cog.outl("obstacle_avoidance_input->point_{p}.acceleration_or_force.{axis} = enu_acceleration_point_{p}.{axis}();".format(**locals()))
		//     cog.outl("obstacle_avoidance_input->point_{p}.yaw = yaw_enu_point_{p};".format(**locals()))
		//     cog.outl("obstacle_avoidance_input->point_{p}.yaw_rate = avoid_input.point_{p}[10]; \n".format(**locals()))
		// ]]]
		obstacle_avoidance_input->point_1.position.x = enu_position_point_1.x();
		obstacle_avoidance_input->point_1.velocity.x = enu_velocity_point_1.x();
		obstacle_avoidance_input->point_1.acceleration_or_force.x = enu_acceleration_point_1.x();
		obstacle_avoidance_input->point_1.position.y = enu_position_point_1.y();
		obstacle_avoidance_input->point_1.velocity.y = enu_velocity_point_1.y();
		obstacle_avoidance_input->point_1.acceleration_or_force.y = enu_acceleration_point_1.y();
		obstacle_avoidance_input->point_1.position.z = enu_position_point_1.z();
		obstacle_avoidance_input->point_1.velocity.z = enu_velocity_point_1.z();
		obstacle_avoidance_input->point_1.acceleration_or_force.z = enu_acceleration_point_1.z();
		obstacle_avoidance_input->point_1.yaw = yaw_enu_point_1;
		obstacle_avoidance_input->point_1.yaw_rate = avoid_input.point_1[10];

		obstacle_avoidance_input->point_2.position.x = enu_position_point_2.x();
		obstacle_avoidance_input->point_2.velocity.x = enu_velocity_point_2.x();
		obstacle_avoidance_input->point_2.acceleration_or_force.x = enu_acceleration_point_2.x();
		obstacle_avoidance_input->point_2.position.y = enu_position_point_2.y();
		obstacle_avoidance_input->point_2.velocity.y = enu_velocity_point_2.y();
		obstacle_avoidance_input->point_2.acceleration_or_force.y = enu_acceleration_point_2.y();
		obstacle_avoidance_input->point_2.position.z = enu_position_point_2.z();
		obstacle_avoidance_input->point_2.velocity.z = enu_velocity_point_2.z();
		obstacle_avoidance_input->point_2.acceleration_or_force.z = enu_acceleration_point_2.z();
		obstacle_avoidance_input->point_2.yaw = yaw_enu_point_2;
		obstacle_avoidance_input->point_2.yaw_rate = avoid_input.point_2[10];

		obstacle_avoidance_input->point_3.position.x = enu_position_point_3.x();
		obstacle_avoidance_input->point_3.velocity.x = enu_velocity_point_3.x();
		obstacle_avoidance_input->point_3.acceleration_or_force.x = enu_acceleration_point_3.x();
		obstacle_avoidance_input->point_3.position.y = enu_position_point_3.y();
		obstacle_avoidance_input->point_3.velocity.y = enu_velocity_point_3.y();
		obstacle_avoidance_input->point_3.acceleration_or_force.y = enu_acceleration_point_3.y();
		obstacle_avoidance_input->point_3.position.z = enu_position_point_3.z();
		obstacle_avoidance_input->point_3.velocity.z = enu_velocity_point_3.z();
		obstacle_avoidance_input->point_3.acceleration_or_force.z = enu_acceleration_point_3.z();
		obstacle_avoidance_input->point_3.yaw = yaw_enu_point_3;
		obstacle_avoidance_input->point_3.yaw_rate = avoid_input.point_3[10];

		obstacle_avoidance_input->point_4.position.x = enu_position_point_4.x();
		obstacle_avoidance_input->point_4.velocity.x = enu_velocity_point_4.x();
		obstacle_avoidance_input->point_4.acceleration_or_force.x = enu_acceleration_point_4.x();
		obstacle_avoidance_input->point_4.position.y = enu_position_point_4.y();
		obstacle_avoidance_input->point_4.velocity.y = enu_velocity_point_4.y();
		obstacle_avoidance_input->point_4.acceleration_or_force.y = enu_acceleration_point_4.y();
		obstacle_avoidance_input->point_4.position.z = enu_position_point_4.z();
		obstacle_avoidance_input->point_4.velocity.z = enu_velocity_point_4.z();
		obstacle_avoidance_input->point_4.acceleration_or_force.z = enu_acceleration_point_4.z();
		obstacle_avoidance_input->point_4.yaw = yaw_enu_point_4;
		obstacle_avoidance_input->point_4.yaw_rate = avoid_input.point_4[10];

		obstacle_avoidance_input->point_5.position.x = enu_position_point_5.x();
		obstacle_avoidance_input->point_5.velocity.x = enu_velocity_point_5.x();
		obstacle_avoidance_input->point_5.acceleration_or_force.x = enu_acceleration_point_5.x();
		obstacle_avoidance_input->point_5.position.y = enu_position_point_5.y();
		obstacle_avoidance_input->point_5.velocity.y = enu_velocity_point_5.y();
		obstacle_avoidance_input->point_5.acceleration_or_force.y = enu_acceleration_point_5.y();
		obstacle_avoidance_input->point_5.position.z = enu_position_point_5.z();
		obstacle_avoidance_input->point_5.velocity.z = enu_velocity_point_5.z();
		obstacle_avoidance_input->point_5.acceleration_or_force.z = enu_acceleration_point_5.z();
		obstacle_avoidance_input->point_5.yaw = yaw_enu_point_5;
		obstacle_avoidance_input->point_5.yaw_rate = avoid_input.point_5[10];

		// [[[end]]] (checksum: 543833261dcacd7e8194cac017354287)

		std::copy(avoid_input.point_valid.begin(), avoid_input.point_valid.end(), obstacle_avoidance_input->point_valid.begin());
		std::copy(avoid_input.field_of_view.begin(), avoid_input.field_of_view.end(), obstacle_avoidance_input->field_of_view.begin());

		avoidance_input_pub.publish(obstacle_avoidance_input);
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
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ObstacleAvoidancePlugin, mavros::plugin::PluginBase)
