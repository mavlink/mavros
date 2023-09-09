/*
 * Copyright 2018 Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Wind estimation plugin
 * @file wind_estimation.cpp
 * @author Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */

#include <angles/angles.h>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Wind estimation plugin.
 * @plugin wind_estimation
 */
class WindEstimationPlugin : public plugin::Plugin
{
public:
  explicit WindEstimationPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "wind")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    wind_pub = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "wind_estimation", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&WindEstimationPlugin::handle_apm_wind),
      make_handler(&WindEstimationPlugin::handle_px4_wind),
    };
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr wind_pub;

  /**
   * Handle APM specific wind estimation message
   */
  void handle_apm_wind(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::WIND & wind, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    const double speed = wind.speed;
    // direction "from" -> direction "to"
    const double course = angles::from_degrees(wind.direction) + M_PI;

    auto twist_cov = geometry_msgs::msg::TwistWithCovarianceStamped();
    twist_cov.header.stamp = node->now();
    twist_cov.twist.twist.linear.x = speed * std::sin(course);  // E
    twist_cov.twist.twist.linear.y = speed * std::cos(course);  // N
    twist_cov.twist.twist.linear.z = -wind.speed_z;             // D -> U

    // covariance matrix unknown in APM msg
    ftf::EigenMapCovariance6d cov_map(twist_cov.twist.covariance.data());
    cov_map.setZero();
    cov_map(0, 0) = -1.0;

    wind_pub->publish(twist_cov);
  }

  /**
   * Handle PX4 specific wind estimation message
   */
  void handle_px4_wind(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::WIND_COV & wind, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto twist_cov = geometry_msgs::msg::TwistWithCovarianceStamped();
    twist_cov.header.stamp = uas->synchronise_stamp(wind.time_usec);

    tf2::toMsg(
      ftf::transform_frame_ned_enu(Eigen::Vector3d(wind.wind_x, wind.wind_y, wind.wind_z)),
      twist_cov.twist.twist.linear);

    // fill available covariance elements
    ftf::EigenMapCovariance6d cov_map(twist_cov.twist.covariance.data());
    cov_map.setZero();
    // NOTE: this is a summed covariance for both x and y horizontal wind components
    cov_map(0, 0) = wind.var_horiz;
    cov_map(2, 2) = wind.var_vert;

    wind_pub->publish(twist_cov);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::WindEstimationPlugin)
