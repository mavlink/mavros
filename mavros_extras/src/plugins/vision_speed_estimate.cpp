/*
 * Copyright 2014, 2018 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief VisionSpeedEstimate plugin
 * @file vision_speed.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Vision speed estimate plugin
 * @plugin vision_speed
 *
 * Send velocity estimation from various vision estimators
 * to FCU position and attitude estimators.
 */
class VisionSpeedEstimatePlugin : public plugin::Plugin
{
public:
  explicit VisionSpeedEstimatePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "vision_speed")
  {
    vision_twist_cov_sub =
      node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "~/speed_twist_cov", 10, std::bind(&VisionSpeedEstimatePlugin::twist_cov_cb, this, _1));
    vision_twist_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/speed_twist", 10,
      std::bind(&VisionSpeedEstimatePlugin::twist_cb, this, _1));
    vision_vector_sub = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "~/speed_vector", 10, std::bind(&VisionSpeedEstimatePlugin::vector_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vision_twist_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    vision_twist_cov_sub;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr vision_vector_sub;

  /* -*- low-level send -*- */
  /**
   * @brief Send vision speed estimate on local NED frame to the FCU.
   *
   * Message specification: https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
   * @param usec	Timestamp (microseconds, synced to UNIX time or since system boot) (us)
   * @param v	Velocity/speed vector in the local NED frame (meters)
   * @param cov	Linear velocity covariance matrix (local NED frame)
   */
  void send_vision_speed_estimate(
    const uint64_t usec, const Eigen::Vector3d & v,
    const ftf::Covariance3d & cov)
  {
    mavlink::common::msg::VISION_SPEED_ESTIMATE vs {};

    vs.usec = usec;

    // [[[cog:
    // for f in "xyz":
    //     cog.outl("vs.%s = v.%s();" % (f, f))
    // ]]]
    vs.x = v.x();
    vs.y = v.y();
    vs.z = v.z();
    // [[[end]]] (checksum: c0c3a3d4dea27c5dc44e4d4f982ff1b6)

    ftf::covariance_to_mavlink(cov, vs.covariance);

    uas->send_message(vs);
  }

  /* -*- mid-level helpers -*- */
  /**
   * @brief Convert vector and covariance from local ENU to local NED frame
   *
   * @param stamp		ROS timestamp of the message
   * @param vel_enu	Velocity/speed vector in the ENU frame
   * @param cov_enu	Linear velocity/speed in the ENU frame
   */
  void convert_vision_speed(
    const rclcpp::Time & stamp, const Eigen::Vector3d & vel_enu,
    const ftf::Covariance3d & cov_enu)
  {
    // Send transformed data from local ENU to NED frame
    send_vision_speed_estimate(
      get_time_usec(stamp),
      ftf::transform_frame_enu_ned(vel_enu),
      ftf::transform_frame_enu_ned(cov_enu));
  }

  /* -*- callbacks -*- */
  /**
   * @brief Callback to geometry_msgs/TwistStamped msgs
   *
   * @param req	received geometry_msgs/TwistStamped msg
   */
  void twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr req)
  {
    ftf::Covariance3d cov {};                   // zero initialized

    convert_vision_speed(req->header.stamp, ftf::to_eigen(req->twist.linear), cov);
  }

  /**
   * @brief Callback to geometry_msgs/TwistWithCovarianceStamped msgs
   *
   * @param req	received geometry_msgs/TwistWithCovarianceStamped msg
   */
  void twist_cov_cb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr req)
  {
    ftf::Covariance3d cov3d {};                 // zero initialized

    ftf::EigenMapCovariance3d cov3d_map(cov3d.data());
    ftf::EigenMapConstCovariance6d cov6d_map(req->twist.covariance.data());

    // only the linear velocity will be sent
    cov3d_map = cov6d_map.block<3, 3>(0, 0);

    convert_vision_speed(req->header.stamp, ftf::to_eigen(req->twist.twist.linear), cov3d);
  }

  /**
   * @brief Callback to geometry_msgs/Vector3Stamped msgs
   *
   * @param req	received geometry_msgs/Vector3Stamped msg
   */
  void vector_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr req)
  {
    ftf::Covariance3d cov {};                   // zero initialized

    convert_vision_speed(req->header.stamp, ftf::to_eigen(req->vector), cov);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VisionSpeedEstimatePlugin)
