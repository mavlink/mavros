/*
 * Copyright 2014 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief SetpointAttitude plugin
 * @file setpoint_attitude.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/thrust.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

using SyncPoseThrustPolicy = message_filters::sync_policies::ApproximateTime<
  geometry_msgs::msg::PoseStamped, mavros_msgs::msg::Thrust>;
using SyncTwistThrustPolicy = message_filters::sync_policies::ApproximateTime<
  geometry_msgs::msg::TwistStamped, mavros_msgs::msg::Thrust>;
using SyncPoseThrust = message_filters::Synchronizer<SyncPoseThrustPolicy>;
using SyncTwistThrust = message_filters::Synchronizer<SyncTwistThrustPolicy>;

/**
 * @brief Setpoint attitude plugin
 * @plugin setpoint_attitude
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointAttitudePlugin : public plugin::Plugin,
  private plugin::SetAttitudeTargetMixin<SetpointAttitudePlugin>
{
public:
  explicit SetpointAttitudePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_attitude"),
    reverse_thrust(false)
  {
    enable_node_watch_parameters();

    auto qos = rclcpp::QoS(10);

    node_declare_and_watch_parameter(
      "reverse_thrust", false, [&](const rclcpp::Parameter & p) {
        reverse_thrust = p.as_bool();
      });

    node_declare_and_watch_parameter(
      "use_quaternion", false, [&](const rclcpp::Parameter & p) {
        auto use_quaternion = p.as_bool();

        pose_sub.unsubscribe();
        twist_sub.unsubscribe();
        sync_pose.reset();
        sync_twist.reset();

        if (use_quaternion) {
          /**
           * @brief Use message_filters to sync attitude and thrust msg coming from different topics
           */
          pose_sub.subscribe(node, "~/attitude", qos.get_rmw_qos_profile());

          sync_pose = std::make_unique<SyncPoseThrust>(SyncPoseThrustPolicy(10), pose_sub, th_sub);
          sync_pose->registerCallback(&SetpointAttitudePlugin::attitude_pose_cb, this);

        } else {
          twist_sub.subscribe(node, "~/cmd_vel", qos.get_rmw_qos_profile());

          sync_twist =
          std::make_unique<SyncTwistThrust>(SyncTwistThrustPolicy(10), twist_sub, th_sub);
          sync_twist->registerCallback(&SetpointAttitudePlugin::attitude_twist_cb, this);
        }
      });

    // thrust msg subscriber to sync
    th_sub.subscribe(node, "~/thrust", qos.get_rmw_qos_profile());
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetAttitudeTargetMixin<SetpointAttitudePlugin>;

  message_filters::Subscriber<mavros_msgs::msg::Thrust> th_sub;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub;
  message_filters::Subscriber<geometry_msgs::msg::TwistStamped> twist_sub;

  std::unique_ptr<SyncPoseThrust> sync_pose;
  std::unique_ptr<SyncTwistThrust> sync_twist;

  bool reverse_thrust;
  float normalized_thrust;

  /**
   * @brief Function to verify if the thrust values are normalized;
   * considers also the reversed trust values
   */
  inline bool is_normalized(float thrust)
  {
    auto lg = get_logger();

    if (reverse_thrust) {
      if (thrust < -1.0) {
        RCLCPP_WARN(lg, "Not normalized reversed thrust! Thd(%f) < Min(%f)", thrust, -1.0);
        return false;
      }
    } else {
      if (thrust < 0.0) {
        RCLCPP_WARN(lg, "Not normalized thrust! Thd(%f) < Min(%f)", thrust, 0.0);
        return false;
      }
    }

    if (thrust > 1.0) {
      RCLCPP_WARN(lg, "Not normalized thrust! Thd(%f) > Max(%f)", thrust, 1.0);
      return false;
    }
    return true;
  }

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send attitude setpoint and thrust to FCU attitude controller
   */
  void send_attitude_quaternion(
    const rclcpp::Time & stamp, const Eigen::Affine3d & tr,
    const float thrust)
  {
    /**
     * @note RPY, also bits numbering started from 1 in docs
     */
    const uint8_t ignore_all_except_q_and_thrust = (7 << 0);

    auto q = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))
    );

    set_attitude_target(
      get_time_boot_ms(stamp),
      ignore_all_except_q_and_thrust,
      q,
      Eigen::Vector3d::Zero(),
      thrust);
  }

  /**
   * @brief Send angular velocity setpoint and thrust to FCU attitude controller
   */
  void send_attitude_ang_velocity(
    const rclcpp::Time & stamp, const Eigen::Vector3d & ang_vel,
    const float thrust)
  {
    /**
     * @note Q, also bits noumbering started from 1 in docs
     */
    const uint8_t ignore_all_except_rpy = (1 << 7);

    auto av = ftf::transform_frame_ned_enu(ang_vel);

    set_attitude_target(
      get_time_boot_ms(stamp),
      ignore_all_except_rpy,
      Eigen::Quaterniond::Identity(),
      av,
      thrust);
  }

  /* -*- callbacks -*- */

  void attitude_pose_cb(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg,
    const mavros_msgs::msg::Thrust::SharedPtr thrust_msg)
  {
    Eigen::Affine3d tr;
    tf2::fromMsg(pose_msg->pose, tr);

    if (is_normalized(thrust_msg->thrust)) {
      send_attitude_quaternion(pose_msg->header.stamp, tr, thrust_msg->thrust);
    }
  }

  void attitude_twist_cb(
    const geometry_msgs::msg::TwistStamped::SharedPtr req,
    const mavros_msgs::msg::Thrust::SharedPtr thrust_msg)
  {
    Eigen::Vector3d ang_vel;
    tf2::fromMsg(req->twist.angular, ang_vel);

    if (is_normalized(thrust_msg->thrust)) {
      send_attitude_ang_velocity(req->header.stamp, ang_vel, thrust_msg->thrust);
    }
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointAttitudePlugin)
