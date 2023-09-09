/*
 * Copyright 2014,2015,2016 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief MocapPoseEstimate plugin
 * @plugin mocap_pose_estimate
 *
 * Sends motion capture data to FCU.
 */
class MocapPoseEstimatePlugin : public plugin::Plugin
{
public:
  explicit MocapPoseEstimatePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "mocap")
  {
    /** @note For VICON ROS package, subscribe to TransformStamped topic */
    mocap_tf_sub = node->create_subscription<geometry_msgs::msg::TransformStamped>(
      "~/tf", 1, std::bind(
        &MocapPoseEstimatePlugin::mocap_tf_cb, this,
        _1));
    /** @note For Optitrack ROS package, subscribe to PoseStamped topic */
    mocap_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/pose", 1, std::bind(
        &MocapPoseEstimatePlugin::mocap_pose_cb, this,
        _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr mocap_tf_sub;

  /* -*- low-level send -*- */
  void mocap_pose_send(
    uint64_t usec,
    Eigen::Quaterniond & q,
    Eigen::Vector3d & v)
  {
    mavlink::common::msg::ATT_POS_MOCAP pos = {};

    pos.time_usec = usec;
    ftf::quaternion_to_mavlink(q, pos.q);
    pos.x = v.x();
    pos.y = v.y();
    pos.z = v.z();

    uas->send_message(pos);
  }

  /* -*- callbacks -*- */

  void mocap_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    Eigen::Quaterniond q_enu;

    tf2::fromMsg(pose->pose.orientation, q_enu);
    auto q = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(q_enu));

    auto position = ftf::transform_frame_enu_ned(
      Eigen::Vector3d(
        pose->pose.position.x,
        pose->pose.position.y,
        pose->pose.position.z));

    mocap_pose_send(
      get_time_usec(pose->header.stamp),
      q,
      position);
  }

  void mocap_tf_cb(const geometry_msgs::msg::TransformStamped::SharedPtr trans)
  {
    Eigen::Quaterniond q_enu;

    tf2::fromMsg(trans->transform.rotation, q_enu);
    auto q = ftf::transform_orientation_enu_ned(
      ftf::transform_orientation_baselink_aircraft(q_enu));

    auto position = ftf::transform_frame_enu_ned(
      Eigen::Vector3d(
        trans->transform.translation.x,
        trans->transform.translation.y,
        trans->transform.translation.z));

    mocap_pose_send(
      get_time_usec(trans->header.stamp),
      q,
      position);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::MocapPoseEstimatePlugin)
