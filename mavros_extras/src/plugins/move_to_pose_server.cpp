/*
 * Copyright 2014,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointPosition plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <GeographicLib/Geocentric.hpp>
#include <cmath>

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "mavros_msgs/action/move_to_pose.hpp"


namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT
using mavlink::common::MAV_FRAME;
using MoveToPose = mavros_msgs::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using mavlink::common::MAV_FRAME;

/**
 * @brief Setpoint position plugin
 * @plugin setpoint_position
 *
 * Send setpoint positions to FCU controller.
 */
class MoveToPoseServerPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<MoveToPoseServerPlugin>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  explicit MoveToPoseServerPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "move_to_pose_server")
  {
    enable_node_watch_parameters();

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto sensor_qos = rclcpp::SensorDataQoS();

    local_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "local_position/pose", sensor_qos,
      [this](const PoseStamped::SharedPtr msg) {
        current_local_pose = *msg;
      });

    move_to_pose_action_server = rclcpp_action::create_server<MoveToPose>(
      node, "~/set_pose",
      [this](const rclcpp_action::GoalUUID & uuid [[maybe_unused]], std::shared_ptr<const MoveToPose::Goal> goal [[maybe_unused]]) {
        RCLCPP_DEBUG(this->get_logger(), "Received MoveToPose goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose> goal_handle [[maybe_unused]]) {
        RCLCPP_DEBUG(this->get_logger(), "Received request to cancel MoveToPose goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose> goal_handle [[maybe_unused]]) {
        std::thread([this, goal_handle]() {
          execute(goal_handle);
        }).detach();
      }
    );
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<MoveToPoseServerPlugin>;

  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_action_server;
  rclcpp::Subscription<PoseStamped>::SharedPtr local_sub;
  PoseStamped current_local_pose;
  
  // TF2 for transform listening
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /**
   * @brief Send setpoint to FCU position controller.
   *
   * @warning Send only XYZ, Yaw. ENU frame.
   */
  void send_position_target(const Eigen::Affine3d & tr)
  {
    const uint16_t ignore_all_except_xyz_y = (1 << 11) | (7 << 6) | (7 << 3);
    auto p = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
    auto q = ftf::transform_orientation_enu_ned(
          ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

    set_position_target_local_ned(
      get_time_boot_ms(this->get_clock()->now()),
      utils::enum_value(MAV_FRAME::LOCAL_NED),
      ignore_all_except_xyz_y,
      p,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      ftf::quaternion_get_yaw(q), 0.0);
  }

  /* -*- main function for server -*- */

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing MoveToPose goal");

    // Get the goal and create feedback/result messages
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();

    auto parent_frame = goal->parent_frame;
    auto target_frame = goal->target_frame;
    Eigen::Affine3d tr;
    
    if (parent_frame != "" && target_frame != "") {
      try {
        // Look up the transform from parent_frame to target_frame
        TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(parent_frame, target_frame, tf2::TimePointZero);
        
        // Convert TransformStamped to Pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = transform_stamped.transform.translation.x;
        target_pose.position.y = transform_stamped.transform.translation.y;
        target_pose.position.z = transform_stamped.transform.translation.z;
        target_pose.orientation = transform_stamped.transform.rotation;
        
        // Now convert Pose to Eigen::Affine3d
        tf2::fromMsg(target_pose, tr);
        
        RCLCPP_INFO(this->get_logger(), "Using transform from '%s' to '%s'", 
                   parent_frame.c_str(), target_frame.c_str());
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform '%s' to '%s': %s", 
                    parent_frame.c_str(), target_frame.c_str(), ex.what());
        result->error_code = 3; // Transform error
        goal_handle->abort(result);
        return;
      }
    } else {
      tf2::fromMsg(goal->target_pose.pose, tr);
      RCLCPP_INFO(this->get_logger(), "Using target pose from goal message");
    }

    // Tolerance checking
    rclcpp::Time tolerance_start_time; // Track when tolerance conditions first met
    bool tolerance_conditions_met = false;
    double min_success_duration = goal->min_success_duration;
    
    // Send the goal to AP
    auto start_time = this->get_clock()->now();
    send_position_target(tr);

    // Main loop to check for abort, cancel or success
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) 
    {
      loop_rate.sleep();

      // Check if there is a cancel request
      if (goal_handle->is_canceling() || !goal_handle->is_active()) {
        result->error_code = 2;
        stop_at_current_pose();
        goal_handle->canceled(result);
        RCLCPP_DEBUG(this->get_logger(), "MoveToPose goal canceled");
        return;
      } 

      // Check if timeout
      auto elapsed = this->get_clock()->now() - start_time;
      if (goal->timeout > 0.0 && elapsed.seconds() > goal->timeout) {
        result->error_code = 1;
        stop_at_current_pose();
        goal_handle->abort(result);
        RCLCPP_WARN(this->get_logger(), "MoveToPose goal aborted due to timeout");
        return;
      }

      // Check if the goal is reached
      double distance_remaining = (ftf::to_eigen(current_local_pose.pose.position) - Eigen::Vector3d(tr.translation())).norm();
      
      // Calculate heading difference
      double target_yaw = ftf::quaternion_get_yaw(Eigen::Quaterniond(tr.rotation()));
      double current_yaw = ftf::quaternion_get_yaw(ftf::to_eigen(current_local_pose.pose.orientation));
      
      // Normalize yaw difference to [-pi, pi]
      double yaw_diff = target_yaw - current_yaw;
      while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
      while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
      double heading_error = std::abs(yaw_diff);
      
      feedback->distance_remaining = distance_remaining;
      feedback->current_pose = current_local_pose;

      feedback->navigation_time = builtin_interfaces::msg::Duration();
      feedback->navigation_time.sec = static_cast<int32_t>(elapsed.seconds());

      goal_handle->publish_feedback(feedback);
      RCLCPP_DEBUG(this->get_logger(), "Distance to goal: %.2f, Heading error: %.2f rad", distance_remaining, heading_error);

      // Check if both position and heading are within tolerance
      auto position_tolerance = goal->position_tolerance;
      auto heading_tolerance = goal->heading_tolerance;

      bool current_tolerance_met = (distance_remaining < position_tolerance && heading_error < heading_tolerance);
      
      if (current_tolerance_met) {
        if (!tolerance_conditions_met) {
          // First time meeting tolerance conditions
          tolerance_start_time = this->get_clock()->now();
          tolerance_conditions_met = true;
          RCLCPP_DEBUG(this->get_logger(), "Tolerance conditions met - starting %.1f second stability check", min_success_duration);
        } else {
          // Check if we've been within tolerance for minimum duration
          auto tolerance_duration = (this->get_clock()->now() - tolerance_start_time).seconds();
          
          RCLCPP_DEBUG(this->get_logger(), "Within tolerance for %.2f/%.1f seconds", tolerance_duration, min_success_duration);
          
          if (tolerance_duration >= min_success_duration) {
            result->error_code = 0;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "MoveToPose goal succeeded - position and heading stable for %.1f seconds", min_success_duration);
            return;
          }
        }
      } else {
        if (tolerance_conditions_met) {
          // We were within tolerance but now we're not - reset
          tolerance_conditions_met = false;
          RCLCPP_DEBUG(this->get_logger(), "Tolerance conditions lost - resetting stability timer");
        }
      }
    }
  }

  void stop_at_current_pose()
  {
    Eigen::Affine3d current_local_pose_eigen;
    tf2::fromMsg(current_local_pose.pose, current_local_pose_eigen);
    send_position_target(current_local_pose_eigen);

    // Ignore position and accel vectors, yaw.
    // uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);
    // auto mav_frame = MAV_FRAME::BODY_NED;

    // set_position_target_local_ned(
    //   get_time_boot_ms(this->get_clock()->now()),
    //   utils::enum_value(mav_frame),
    //   ignore_all_except_v_xyz_yr,
    //   Eigen::Vector3d::Zero(),
    //   Eigen::Vector3d::Zero(),
    //   Eigen::Vector3d::Zero(),
    //   0.0, 0.0
    // );
  }
};

}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::MoveToPoseServerPlugin)
