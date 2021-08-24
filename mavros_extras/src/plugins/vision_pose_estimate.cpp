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
    tf_rate(50.0)
  {}


  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  double tf_rate;

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

  void vision_cb(const geometry_msgs::msg::PoseStamped::ConstPtr & req)
  {

  }

  void vision_cov_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr & req)
  {

  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VisionPoseEstimatePlugin)
