/*
 * Copyright 2019 Jaeyoung Lim.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointTRAJECTORY plugin
 * @file setpoint_trajectory.cpp
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <vector>
#include <string>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "nav_msgs/msg/path.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT
using mavlink::common::MAV_FRAME;

/**
 * @brief Setpoint TRAJECTORY plugin
 * @plugin setpoint_trajectory
 *
 * Receive trajectory setpoints and send setpoint_raw setpoints along the trajectory.
 */
class SetpointTrajectoryPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointTrajectoryPlugin>
{
public:
  explicit SetpointTrajectoryPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_trajectory")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "frame_id", "map", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    node_declare_and_watch_parameter(
      "mav_frame", "LOCAL_NED", [&](const rclcpp::Parameter & p) {
        auto mav_frame_str = p.as_string();
        auto new_mav_frame = utils::mav_frame_from_str(mav_frame_str);

        if (new_mav_frame == MAV_FRAME::LOCAL_NED && mav_frame_str != "LOCAL_NED") {
          throw rclcpp::exceptions::InvalidParameterValueException(
            utils::format(
              "unknown MAV_FRAME: %s",
              mav_frame_str.c_str()));
        }
        mav_frame = new_mav_frame;
      });

    auto sensor_qos = rclcpp::SensorDataQoS();

    local_sub = node->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      "~/local",
      sensor_qos, std::bind(
        &SetpointTrajectoryPlugin::local_cb, this,
        _1));
    desired_pub = node->create_publisher<nav_msgs::msg::Path>("~/desired", sensor_qos);

    trajectory_reset_srv =
      node->create_service<std_srvs::srv::Trigger>(
      "~/reset",
      std::bind(&SetpointTrajectoryPlugin::reset_cb, this, _1, _2));

    reset_timer(10ms);
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointTrajectoryPlugin>;
  using lock_guard = std::lock_guard<std::mutex>;
  using V_Point = std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>;

  std::mutex mutex;

  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr local_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trajectory_reset_srv;
  rclcpp::TimerBase::SharedPtr sp_timer;

  trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr trajectory_target_msg;

  V_Point::const_iterator setpoint_target;
  V_Point::const_iterator next_setpoint_target;

  std::string frame_id;
  MAV_FRAME mav_frame;
  ftf::StaticTF transform;

  void reset_timer(const builtin_interfaces::msg::Duration & dur)
  {
    reset_timer(rclcpp::Duration(dur).to_chrono<std::chrono::nanoseconds>());
  }

  void reset_timer(const std::chrono::nanoseconds dur)
  {
    if (sp_timer) {
      sp_timer->cancel();
    }

    sp_timer =
      node->create_wall_timer(dur, std::bind(&SetpointTrajectoryPlugin::reference_cb, this));
  }

  void publish_path(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr req)
  {
    nav_msgs::msg::Path msg;

    msg.header.stamp = node->now();
    msg.header.frame_id = frame_id;
    for (const auto & p : req->points) {
      if (p.transforms.empty()) {
        continue;
      }

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.pose.position.x = p.transforms[0].translation.x;
      pose_msg.pose.position.y = p.transforms[0].translation.y;
      pose_msg.pose.position.z = p.transforms[0].translation.z;
      pose_msg.pose.orientation = p.transforms[0].rotation;
      msg.poses.emplace_back(pose_msg);
    }
    desired_pub->publish(msg);
  }

  /* -*- callbacks -*- */

  void local_cb(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr req)
  {
    lock_guard lock(mutex);

    if (mav_frame == MAV_FRAME::BODY_NED ||
      mav_frame == MAV_FRAME::BODY_OFFSET_NED)
    {
      transform = ftf::StaticTF::BASELINK_TO_AIRCRAFT;
    } else {
      transform = ftf::StaticTF::ENU_TO_NED;
    }

    trajectory_target_msg = req;

    // Read first duration of the setpoint and set the timer
    setpoint_target = req->points.cbegin();
    reset_timer(setpoint_target->time_from_start);
    publish_path(req);
  }

  void reference_cb()
  {
    using mavlink::common::POSITION_TARGET_TYPEMASK;
    lock_guard lock(mutex);

    if (!trajectory_target_msg) {
      return;
    }

    Eigen::Vector3d position, velocity, af;
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
    float yaw_rate = 0;
    uint16_t type_mask = 0;

    if (!setpoint_target->transforms.empty()) {
      position =
        ftf::detail::transform_static_frame(
        ftf::to_eigen(
          setpoint_target->transforms[0].translation), transform);
      attitude =
        ftf::detail::transform_orientation(
        ftf::to_eigen(
          setpoint_target->transforms[0].rotation), transform);

    } else {
      type_mask = type_mask | uint16_t(POSITION_TARGET_TYPEMASK::X_IGNORE) |
        uint16_t(POSITION_TARGET_TYPEMASK::Y_IGNORE) |
        uint16_t(POSITION_TARGET_TYPEMASK::Z_IGNORE) |
        uint16_t(POSITION_TARGET_TYPEMASK::YAW_IGNORE);
    }

    if (!setpoint_target->velocities.empty()) {
      velocity =
        ftf::detail::transform_static_frame(
        ftf::to_eigen(
          setpoint_target->velocities[0].linear), transform);
      yaw_rate = setpoint_target->velocities[0].angular.z;

    } else {
      type_mask = type_mask | uint16_t(POSITION_TARGET_TYPEMASK::VX_IGNORE) |
        uint16_t(POSITION_TARGET_TYPEMASK::VY_IGNORE) |
        uint16_t(POSITION_TARGET_TYPEMASK::VZ_IGNORE) |
        uint16_t(POSITION_TARGET_TYPEMASK::YAW_RATE_IGNORE);
    }

    if (!setpoint_target->accelerations.empty()) {
      af =
        ftf::detail::transform_static_frame(
        ftf::to_eigen(
          setpoint_target->accelerations[0].linear), transform);

    } else {
      type_mask = type_mask | uint16_t(POSITION_TARGET_TYPEMASK::AX_IGNORE) |
        uint16_t(POSITION_TARGET_TYPEMASK::AY_IGNORE) |
        uint16_t(POSITION_TARGET_TYPEMASK::AZ_IGNORE);
    }

    set_position_target_local_ned(
      get_time_boot_ms(),
      utils::enum_value(mav_frame),
      type_mask,
      position,
      velocity,
      af,
      ftf::quaternion_get_yaw(attitude),
      yaw_rate);

    next_setpoint_target = setpoint_target + 1;
    if (next_setpoint_target != trajectory_target_msg->points.cend()) {
      reset_timer(setpoint_target->time_from_start);
      setpoint_target = next_setpoint_target;
    } else {
      trajectory_target_msg.reset();
    }
  }

  void reset_cb(
    std_srvs::srv::Trigger::Request::SharedPtr req [[maybe_unused]],
    std_srvs::srv::Trigger::Response::SharedPtr res [[maybe_unused]])
  {
    lock_guard lock(mutex);

    if (trajectory_target_msg) {
      trajectory_target_msg.reset();
      res->success = true;
    } else {
      res->success = false;
      res->message = "Trajectory reset denied: Empty trajectory";
    }
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointTrajectoryPlugin)
