/**
 * @author Tien Luc Vu <luc001@e.ntu.edu.sg>
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

#include "mavros_msgs/msg/state.hpp"

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
using MavState = mavros_msgs::msg::State;

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

    local_sub = node->create_subscription<PoseStamped>(
      "local_position/pose", sensor_qos,
      [this](const PoseStamped::SharedPtr msg) {
        current_local_pose = *msg;
      });

    mav_state_sub = node->create_subscription<MavState>(
      "state", sensor_qos,
      [this](const MavState::SharedPtr msg) {
        is_armed = msg->armed;
        is_guided = msg->guided;
      });

    target_pose_pub = node->create_publisher<PoseStamped>("setpoint_position/local", rclcpp::SystemDefaultsQoS());  // Default QoS to make sure that it is received

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

  // Action server for move to pose
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_action_server;
  // Provide feedback
  rclcpp::Subscription<PoseStamped>::SharedPtr local_sub;
  // Publish target pose
  rclcpp::Publisher<PoseStamped>::SharedPtr target_pose_pub;
  // Subscriber to check on state
  rclcpp::Subscription<MavState>::SharedPtr mav_state_sub;
  
  PoseStamped current_local_pose;
  bool is_armed;
  bool is_guided;
  
  // TF2 for transform listening
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /* -*- main function for server -*- */

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    // Get the goal and create feedback/result messages
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();

    if (is_guided && is_armed) {
      RCLCPP_INFO(this->get_logger(), "Executing MoveToPose goal");
    } else {
      RCLCPP_WARN(this->get_logger(), "Vehicle not in GUIDED mode or not ARMED. Aborting MoveToPose goal.");
      result->error_code = 3;
      goal_handle->abort(result);
      return;
    }

    auto parent_frame = goal->parent_frame;
    auto target_frame = goal->target_frame;
    PoseStamped target_pose;
    Eigen::Affine3d tr;
    
    if (parent_frame != "" && target_frame != "") {
      try {
        // Look up the transform from parent_frame to target_frame
        TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(parent_frame, target_frame, tf2::TimePointZero);
        
        // Convert TransformStamped to Pose
        target_pose.pose.position.x = transform_stamped.transform.translation.x;
        target_pose.pose.position.y = transform_stamped.transform.translation.y;
        target_pose.pose.position.z = transform_stamped.transform.translation.z;
        target_pose.pose.orientation = transform_stamped.transform.rotation;

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
      target_pose = goal->target_pose;
      RCLCPP_INFO(this->get_logger(), "Using target pose from goal message");
    }
    // Now convert Pose to Eigen::Affine3d
    tf2::fromMsg(target_pose.pose, tr);

    // Tolerance-related variables
    rclcpp::Time tolerance_start_time; // Track when tolerance conditions first met
    bool tolerance_conditions_met = false;
    double min_success_duration = goal->min_success_duration;
    
    // This is when it starts moving
    target_pose.header.stamp = this->get_clock()->now();
    target_pose_pub->publish(target_pose);
    auto start_time = this->get_clock()->now();

    // Main loop to check for abort, cancel or success
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) 
    {
      loop_rate.sleep();

      // Check if there is a cancel request
      if (goal_handle->is_canceling() || !goal_handle->is_active()) {
        result->error_code = 2;
        target_pose_pub->publish(current_local_pose);
        goal_handle->canceled(result);
        RCLCPP_DEBUG(this->get_logger(), "MoveToPose goal canceled");
        return;
      } 

      // Check if timeout
      auto elapsed = this->get_clock()->now() - start_time;
      if (goal->timeout > 0.0 && elapsed.seconds() > goal->timeout) {
        result->error_code = 1;
        target_pose_pub->publish(current_local_pose);
        goal_handle->abort(result);
        RCLCPP_WARN(this->get_logger(), "MoveToPose goal aborted due to timeout");
        return;
      }

      // Calculate distance remaining
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
      feedback->navigation_time.nanosec = static_cast<uint32_t>((elapsed.seconds() - feedback->navigation_time.sec) * 1e9);

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

};

}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::MoveToPoseServerPlugin)
