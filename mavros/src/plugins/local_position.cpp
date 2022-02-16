/*
 * Copyright 2014,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup plugin
 * @{
 */

#include <tf2_eigen/tf2_eigen.h>

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Local position plugin.
 * @plugin local_position
 *
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class LocalPositionPlugin : public plugin::Plugin
{
public:
  explicit LocalPositionPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "local_position"),
    tf_send(false),
    has_local_position_ned(false),
    has_local_position_ned_cov(false)
  {
    enable_node_watch_parameters();

    // header frame_id.
    // default to map (world-fixed, ENU as per REP-105).
    node_declare_and_watch_parameter(
      "frame_id", "map", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    // Important tf subsection
    // Report the transform from world to base_link here.
    node_declare_and_watch_parameter(
      "tf.send", false, [&](const rclcpp::Parameter & p) {
        tf_send = p.as_bool();
      });
    node_declare_and_watch_parameter(
      "tf.frame_id", "map", [&](const rclcpp::Parameter & p) {
        tf_frame_id = p.as_string();
      });
    node_declare_and_watch_parameter(
      "tf.child_frame_id", "base_link", [&](const rclcpp::Parameter & p) {
        tf_child_frame_id = p.as_string();
      });

    auto sensor_qos = rclcpp::SensorDataQoS();

    local_position = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", sensor_qos);
    local_position_cov = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/pose_cov", sensor_qos);
    local_velocity_local = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/velocity_local", sensor_qos);
    local_velocity_body = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/velocity_body",
      sensor_qos);
    local_velocity_cov = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "~/velocity_body_cov", sensor_qos);
    local_accel = node->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "~/accel",
      sensor_qos);
    local_odom = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&LocalPositionPlugin::handle_local_position_ned),
      make_handler(&LocalPositionPlugin::handle_local_position_ned_cov)
    };
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr local_position_cov;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr local_velocity_local;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr local_velocity_body;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr local_velocity_cov;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr local_accel;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom;

  std::string frame_id;                 //!< frame for Pose
  std::string tf_frame_id;              //!< origin for TF
  std::string tf_child_frame_id;        //!< frame for TF
  std::atomic<bool> tf_send;
  std::atomic<bool> has_local_position_ned;
  std::atomic<bool> has_local_position_ned_cov;

  void publish_tf(nav_msgs::msg::Odometry & odom)
  {
    if (tf_send) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = odom.header.stamp;
      transform.header.frame_id = tf_frame_id;
      transform.child_frame_id = tf_child_frame_id;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;
      transform.transform.rotation = odom.pose.pose.orientation;
      uas->tf2_broadcaster.sendTransform(transform);
    }
  }

  void handle_local_position_ned(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::LOCAL_POSITION_NED & pos_ned,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    has_local_position_ned = true;

    //--------------- Transform FCU position and Velocity Data ---------------//
    auto enu_position = ftf::transform_frame_ned_enu(
      Eigen::Vector3d(
        pos_ned.x, pos_ned.y,
        pos_ned.z));
    auto enu_velocity =
      ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

    //--------------- Get Odom Information ---------------//
    // Note this orientation describes baselink->ENU transform
    auto enu_orientation_msg = uas->data.get_attitude_orientation_enu();
    auto baselink_angular_msg = uas->data.get_attitude_angular_velocity_enu();
    Eigen::Quaterniond enu_orientation; tf2::fromMsg(enu_orientation_msg, enu_orientation);
    auto baselink_linear =
      ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

    auto odom = nav_msgs::msg::Odometry();
    odom.header = uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
    odom.child_frame_id = tf_child_frame_id;

    odom.pose.pose.position = tf2::toMsg(enu_position);
    odom.pose.pose.orientation = enu_orientation_msg;
    tf2::toMsg(baselink_linear, odom.twist.twist.linear);
    odom.twist.twist.angular = baselink_angular_msg;

    // publish odom if we don't have LOCAL_POSITION_NED_COV
    if (!has_local_position_ned_cov) {
      local_odom->publish(odom);
    }

    // publish pose always
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    local_position->publish(pose);

    // publish velocity always
    // velocity in the body frame
    auto twist_body = geometry_msgs::msg::TwistStamped();
    twist_body.header.stamp = odom.header.stamp;
    twist_body.header.frame_id = tf_child_frame_id;
    twist_body.twist.linear = odom.twist.twist.linear;
    twist_body.twist.angular = baselink_angular_msg;
    local_velocity_body->publish(twist_body);

    // velocity in the local frame
    auto twist_local = geometry_msgs::msg::TwistStamped();
    twist_local.header.stamp = twist_body.header.stamp;
    twist_local.header.frame_id = tf_child_frame_id;
    tf2::toMsg(enu_velocity, twist_local.twist.linear);
    tf2::toMsg(
      ftf::transform_frame_baselink_enu(ftf::to_eigen(baselink_angular_msg), enu_orientation),
      twist_local.twist.angular);

    local_velocity_local->publish(twist_local);

    // publish tf
    publish_tf(odom);
  }

  void handle_local_position_ned_cov(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::LOCAL_POSITION_NED_COV & pos_ned,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    has_local_position_ned_cov = true;

    auto enu_position = ftf::transform_frame_ned_enu(
      Eigen::Vector3d(
        pos_ned.x, pos_ned.y,
        pos_ned.z));
    auto enu_velocity =
      ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

    auto enu_orientation_msg = uas->data.get_attitude_orientation_enu();
    auto baselink_angular_msg = uas->data.get_attitude_angular_velocity_enu();
    Eigen::Quaterniond enu_orientation; tf2::fromMsg(enu_orientation_msg, enu_orientation);
    auto baselink_linear =
      ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

    auto odom = nav_msgs::msg::Odometry();
    odom.header = uas->synchronized_header(frame_id, pos_ned.time_usec);
    odom.child_frame_id = tf_child_frame_id;

    odom.pose.pose.position = tf2::toMsg(enu_position);
    odom.pose.pose.orientation = enu_orientation_msg;
    tf2::toMsg(baselink_linear, odom.twist.twist.linear);
    odom.twist.twist.angular = baselink_angular_msg;

    odom.pose.covariance[0] = pos_ned.covariance[0];                   // x
    odom.pose.covariance[7] = pos_ned.covariance[9];                   // y
    odom.pose.covariance[14] = pos_ned.covariance[17];                 // z

    odom.twist.covariance[0] = pos_ned.covariance[24];                 // vx
    odom.twist.covariance[7] = pos_ned.covariance[30];                 // vy
    odom.twist.covariance[14] = pos_ned.covariance[35];                // vz
    // TODO(vooon): orientation + angular velocity covariances from ATTITUDE_QUATERION_COV

    // publish odom always
    local_odom->publish(odom);

    // publish pose_cov always
    auto pose_cov = geometry_msgs::msg::PoseWithCovarianceStamped();
    pose_cov.header = odom.header;
    pose_cov.pose = odom.pose;
    local_position_cov->publish(pose_cov);

    // publish velocity_cov always
    auto twist_cov = geometry_msgs::msg::TwistWithCovarianceStamped();
    twist_cov.header.stamp = odom.header.stamp;
    twist_cov.header.frame_id = odom.child_frame_id;
    twist_cov.twist = odom.twist;
    local_velocity_cov->publish(twist_cov);

    // publish pose, velocity, tf if we don't have LOCAL_POSITION_NED
    if (!has_local_position_ned) {
      auto pose = geometry_msgs::msg::PoseStamped();
      pose.header = odom.header;
      pose.pose = odom.pose.pose;
      local_position->publish(pose);

      auto twist = geometry_msgs::msg::TwistStamped();
      twist.header.stamp = odom.header.stamp;
      twist.header.frame_id = odom.child_frame_id;
      twist.twist = odom.twist.twist;
      local_velocity_body->publish(twist);

      // publish tf
      publish_tf(odom);
    }

    // publish accelerations
    auto accel = geometry_msgs::msg::AccelWithCovarianceStamped();
    accel.header = odom.header;

    auto enu_accel = ftf::transform_frame_ned_enu(
      Eigen::Vector3d(
        pos_ned.ax, pos_ned.ay,
        pos_ned.az));
    tf2::toMsg(enu_accel, accel.accel.accel.linear);

    accel.accel.covariance[0] = pos_ned.covariance[39];                // ax
    accel.accel.covariance[7] = pos_ned.covariance[42];                // ay
    accel.accel.covariance[14] = pos_ned.covariance[44];               // az

    local_accel->publish(accel);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::LocalPositionPlugin)
