/*
 * Copyright 2018 Martina Rivizzigno.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.mdA
 */
/**
 * @brief Trajectory plugin
 * @file trajectory.cpp
 * @author Martina Rivizzigno <martina@rivizzigno.it>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/trajectory.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT
using utils::enum_value;

//! Points count in TRAJECTORY message
static constexpr size_t NUM_POINTS = 5;

//! Type matching mavlink::common::msg::TRAJECTORY::TRAJECTORY_REPRESENTATION_WAYPOINTS fields
using MavPoints = std::array<float, NUM_POINTS>;

using RosPoints = mavros_msgs::msg::PositionTarget;

/**
 * @brief Trajectory plugin to receive planned path from the FCU and
 * send back to the FCU a corrected path (collision free, smoothed)
 * @plugin trajectory
 *
 * @see trajectory_cb()
 */
class TrajectoryPlugin : public plugin::Plugin
{
public:
  explicit TrajectoryPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "trajectory")
  {
    trajectory_generated_sub = node->create_subscription<mavros_msgs::msg::Trajectory>(
      "~/generated", 10, std::bind(
        &TrajectoryPlugin::trajectory_cb, this, _1));
    path_sub =
      node->create_subscription<nav_msgs::msg::Path>(
      "~/path", 10,
      std::bind(&TrajectoryPlugin::path_cb, this, _1));
    trajectory_desired_pub = node->create_publisher<mavros_msgs::msg::Trajectory>("~/desired", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&TrajectoryPlugin::handle_trajectory)
    };
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::Trajectory>::SharedPtr trajectory_generated_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;

  rclcpp::Publisher<mavros_msgs::msg::Trajectory>::SharedPtr trajectory_desired_pub;

  // [[[cog:
  // def outl_fill_points_ned_vector(x, y, z, vec_name, vec_type, point_xyz):
  //     cog.outl(
  //         f"""void fill_points_{vec_name}(\n"""
  //         f"""  MavPoints & {x}, MavPoints & {y}, MavPoints & {z},\n"""
  //         f"""  const geometry_msgs::msg::{vec_type} & {vec_name}, const size_t i)\n"""
  //         f"""{{\n"""
  //         f"""  auto {vec_name}_ned = ftf::transform_frame_enu_ned("""
  //         f"""ftf::to_eigen({vec_name}));\n"""
  //     )
  //
  //     for axis in "xyz":
  //         cog.outl(f"  {axis}[i] = {vec_name}_ned.{axis}();")
  //
  //     cog.outl("}\n")
  //
  //
  // outl_fill_points_ned_vector('x', 'y', 'z', 'position', 'Point', range(0, 3))
  // outl_fill_points_ned_vector('x', 'y', 'z', 'velocity', 'Vector3', range(3, 6))
  // outl_fill_points_ned_vector('x', 'y', 'z', 'acceleration', 'Vector3', range(6, 9))
  // ]]]
  void fill_points_position(
    MavPoints & x, MavPoints & y, MavPoints & z,
    const geometry_msgs::msg::Point & position, const size_t i)
  {
    auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));

    x[i] = position_ned.x();
    y[i] = position_ned.y();
    z[i] = position_ned.z();
  }

  void fill_points_velocity(
    MavPoints & x, MavPoints & y, MavPoints & z,
    const geometry_msgs::msg::Vector3 & velocity, const size_t i)
  {
    auto velocity_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(velocity));

    x[i] = velocity_ned.x();
    y[i] = velocity_ned.y();
    z[i] = velocity_ned.z();
  }

  void fill_points_acceleration(
    MavPoints & x, MavPoints & y, MavPoints & z,
    const geometry_msgs::msg::Vector3 & acceleration, const size_t i)
  {
    auto acceleration_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(acceleration));

    x[i] = acceleration_ned.x();
    y[i] = acceleration_ned.y();
    z[i] = acceleration_ned.z();
  }

  // [[[end]]] (checksum: a0ed1550494e431a3ba599da8503c8b6)

  void fill_points_yaw_wp(MavPoints & y, const double yaw, const size_t i)
  {
    y[i] = wrap_pi(-yaw + (M_PI / 2.0f));
  }

  void fill_points_yaw_speed(MavPoints & yv, const double yaw_speed, const size_t i)
  {
    yv[i] = yaw_speed;
  }

  void fill_points_yaw_q(
    MavPoints & y, const geometry_msgs::msg::Quaternion & orientation,
    const size_t i)
  {
    auto q_wp = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(
        ftf::to_eigen(orientation)));
    auto yaw_wp = ftf::quaternion_get_yaw(q_wp);

    y[i] = wrap_pi(-yaw_wp + (M_PI / 2.0f));
  }

  void fill_points_delta(MavPoints & y, const double time_horizon, const size_t i)
  {
    y[i] = time_horizon;
  }

  auto fill_points_unused_path(
    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t,
    const size_t i)
  {
    t.vel_x[i] = NAN;
    t.vel_y[i] = NAN;
    t.vel_z[i] = NAN;
    t.acc_x[i] = NAN;
    t.acc_y[i] = NAN;
    t.acc_z[i] = NAN;
    t.vel_yaw[i] = NAN;
  }

  void fill_points_all_unused(
    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t,
    const size_t i)
  {
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

  void fill_points_all_unused_bezier(
    mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER & t,
    const size_t i)
  {
    t.pos_x[i] = NAN;
    t.pos_y[i] = NAN;
    t.pos_z[i] = NAN;

    t.pos_yaw[i] = NAN;

    t.delta[i] = NAN;
  }

  void fill_msg_position(
    geometry_msgs::msg::Point & position,
    const mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t,
    const size_t i)
  {
    auto position_enu =
      ftf::transform_frame_ned_enu(Eigen::Vector3d(t.pos_x[i], t.pos_y[i], t.pos_z[i]));

    position.x = position_enu.x();
    position.y = position_enu.y();
    position.z = position_enu.z();
  }

  void fill_msg_velocity(
    geometry_msgs::msg::Vector3 & velocity,
    const mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t,
    const size_t i)
  {
    auto velocity_enu =
      ftf::transform_frame_ned_enu(Eigen::Vector3d(t.vel_x[i], t.vel_y[i], t.vel_z[i]));

    velocity.x = velocity_enu.x();
    velocity.y = velocity_enu.y();
    velocity.z = velocity_enu.z();
  }

  void fill_msg_acceleration(
    geometry_msgs::msg::Vector3 & acceleration,
    const mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t,
    const size_t i)
  {
    auto acceleration_enu =
      ftf::transform_frame_ned_enu(Eigen::Vector3d(t.acc_x[i], t.acc_y[i], t.acc_z[i]));

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
  void trajectory_cb(const mavros_msgs::msg::Trajectory::SharedPtr req)
  {
    rcpputils::require_true(NUM_POINTS == req->point_valid.size());

    if (req->type == mavros_msgs::msg::Trajectory::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS) {
      mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS trajectory {};

      auto fill_point_rep_waypoints =
        [&](mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t, const RosPoints & rp,
          const size_t i) {
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
      //          f"fill_point_rep_waypoints(trajectory, req->point_{i+1}, {i});"
      //      )
      // ]]]
      fill_point_rep_waypoints(trajectory, req->point_1, 0);
      fill_point_rep_waypoints(trajectory, req->point_2, 1);
      fill_point_rep_waypoints(trajectory, req->point_3, 2);
      fill_point_rep_waypoints(trajectory, req->point_4, 3);
      fill_point_rep_waypoints(trajectory, req->point_5, 4);
      // [[[end]]] (checksum: 3378a593279611a83e25efee67393195)

      trajectory.time_usec = get_time_usec(req->header.stamp);      //!< [milisecs]
      uas->send_message(trajectory);
    } else {
      mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER trajectory {};
      auto fill_point_rep_bezier =
        [&](mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER & t, const RosPoints & rp,
          const size_t i) {
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
      //          f"fill_point_rep_bezier(trajectory, req->point_{i+1}, {i});"
      //      )
      // ]]]
      fill_point_rep_bezier(trajectory, req->point_1, 0);
      fill_point_rep_bezier(trajectory, req->point_2, 1);
      fill_point_rep_bezier(trajectory, req->point_3, 2);
      fill_point_rep_bezier(trajectory, req->point_4, 3);
      fill_point_rep_bezier(trajectory, req->point_5, 4);
      // [[[end]]] (checksum: a12a34d1190be94c777077f2d297918b)

      trajectory.time_usec = get_time_usec(req->header.stamp);      //!< [milisecs]
      uas->send_message(trajectory);
    }
  }

  /**
   * @brief Send corrected path to the FCU.
   *
   * Message specification: https://mavlink.io/en/messages/common.html#TRAJECTORY
   * @param req	received nav_msgs Path msg
   */
  void path_cb(const nav_msgs::msg::Path::SharedPtr req)
  {
    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS trajectory {};

    trajectory.time_usec = get_time_usec(req->header.stamp);        //!< [milisecs]
    trajectory.valid_points = std::min(NUM_POINTS, req->poses.size());

    auto fill_point =
      [&](mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t, const size_t i) {
        t.command[i] = UINT16_MAX;
        if (req->poses.size() < i + 1) {
          fill_points_all_unused(t, i);
        } else {
          auto & pose = req->poses[i].pose;

          fill_points_position(t.pos_x, t.pos_y, t.pos_z, pose.position, i);
          fill_points_yaw_q(t.pos_yaw, pose.orientation, i);
          fill_points_unused_path(t, i);
        }
      };

    // [[[cog:
    // for i in range(5):
    //      cog.outl(f"fill_point(trajectory, {i});")
    // ]]]
    fill_point(trajectory, 0);
    fill_point(trajectory, 1);
    fill_point(trajectory, 2);
    fill_point(trajectory, 3);
    fill_point(trajectory, 4);
    // [[[end]]] (checksum: a63d2682cc16897f19da141e87ab5d60)

    uas->send_message(trajectory);
  }

  void handle_trajectory(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & trajectory,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto tr_desired = mavros_msgs::msg::Trajectory();

    auto fill_msg_point =
      [&](RosPoints & p, const mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS & t,
        const size_t i) {
        fill_msg_position(p.position, t, i);
        fill_msg_velocity(p.velocity, t, i);
        fill_msg_acceleration(p.acceleration_or_force, t, i);
        p.yaw = wrap_pi((M_PI / 2.0f) - t.pos_yaw[i]);
        p.yaw_rate = t.vel_yaw[i];
        tr_desired.command[i] = t.command[i];
      };

    tr_desired.header = uas->synchronized_header("local_origin", trajectory.time_usec);

    if (trajectory.valid_points > tr_desired.point_valid.size()) {
      return;
    }

    for (int i = 0; i < trajectory.valid_points; ++i) {
      tr_desired.point_valid[i] = true;
    }

    for (size_t i = trajectory.valid_points; i < NUM_POINTS; ++i) {
      tr_desired.point_valid[i] = false;
    }

    // [[[cog:
    // for i in range(5):
    //     cog.outl(f"fill_msg_point(tr_desired.point_{i+1}, trajectory, {i});")
    // ]]]
    fill_msg_point(tr_desired.point_1, trajectory, 0);
    fill_msg_point(tr_desired.point_2, trajectory, 1);
    fill_msg_point(tr_desired.point_3, trajectory, 2);
    fill_msg_point(tr_desired.point_4, trajectory, 3);
    fill_msg_point(tr_desired.point_5, trajectory, 4);
    // [[[end]]] (checksum: a1d59b0aa0f24a18ca76f47397bca4ae)

    trajectory_desired_pub->publish(tr_desired);
  }

  float wrap_pi(float a)
  {
    if (!std::isfinite(a)) {
      return a;
    }

    return fmod(a + M_PI, 2.0f * M_PI) - M_PI;
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TrajectoryPlugin)
