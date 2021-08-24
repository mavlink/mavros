/**
 * @brief VisionPoseEstimate plugin
 * @file vision_pose_estimate.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/utils.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include <tf2_eigen/tf2_eigen.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" //
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/msg/landing_target.hpp"



namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;

/**
 * @brief Vision pose estimate plugin
 * @plugin vision_pose
 *
 * Send pose estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class VisionPoseEstimatePlugin : public plugin::Plugin,
  private plugin::TF2ListenerMixin<VisionPoseEstimatePlugin>
{
public:
  VisionPoseEstimatePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "vision_pose"),
    tf_rate(10.0)
  {
    enable_node_watch_parameters();

    bool tf_listen;

    // tf params
    node_declate_and_watch_parameter(
      "tf/listen", false, [&](const rclcpp::Parameter & p) {
        auto tf_listen = p.as_bool();
        if (tf_listen) {
            RCLCPP_INFO_STREAM(
            get_logger(),
            "Listen to vision transform" << tf_frame_id <<
              " -> " << tf_child_frame_id);
            tf2_start("VisionPoseTF", &VisionPoseEstimatePlugin::transform_cb);
          } else{
            vision_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
              "~/pose", 10, std::bind(
                &VisionPoseEstimatePlugin::vision_cb, this, _1));
            vision_cov_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "~/pose_cov", 10, std::bind(
                &VisionPoseEstimatePlugin::vision_cov_cb, this, _1));
          }
        });

    node_declate_and_watch_parameter(
      "tf/frame_id", "map", [&](const rclcpp::Parameter & p) {
        tf_frame_id = p.as_string();
      });

    node_declate_and_watch_parameter(
      "tf/child_frame_id", "vision_estimate", [&](const rclcpp::Parameter & p) {
        tf_child_frame_id = p.as_string();
      });

    node_declate_and_watch_parameter(
      "tf/rate_limit", 10.0, [&](const rclcpp::Parameter & p) {
        tf_rate = p.as_double();
      });
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class TF2ListenerMixin;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_cov_sub;

  std::string tf_frame_id;
  std::string tf_child_frame_id;

  double tf_rate;

  rclcpp::Time last_transform_stamp;

  /* -*- low-level send -*- */
  /**
   * @brief Send vision estimate transform to FCU position controller
   */
  void send_vision_estimate(void)
  {

  }

  /* -*- callbacks -*- */

  /* common TF listener moved to mixin */

  void transform_cb(const geometry_msgs::msg::TransformStamped & transform)
  {

  }

  void vision_cb(const geometry_msgs::msg::PoseStamped::SharedPtr req)
  {

  }

  void vision_cov_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr req)
  {

  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VisionPoseEstimatePlugin)
