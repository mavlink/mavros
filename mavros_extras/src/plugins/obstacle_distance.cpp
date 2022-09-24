/*
 * Copyright 2017 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Obstacle distance plugin
 * @file obstacle_distance.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

//! Radians to degrees
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
//! Mavlink MAV_DISTANCE_SENSOR enumeration
using mavlink::common::MAV_DISTANCE_SENSOR;

/**
 * @brief Obstacle distance plugin
 * @plugin obstacle_distance
 *
 * Publishes obstacle distance array to the FCU, in order to assist in an obstacle
 * avoidance flight.
 * @see obstacle_cb()
 */
class ObstacleDistancePlugin : public plugin::Plugin
{
public:
  explicit ObstacleDistancePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "obstacle")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "mav_frame", "GLOBAL", [&](const rclcpp::Parameter & p) {
        auto mav_frame = p.as_string();
        frame = utils::mav_frame_from_str(mav_frame);
        // MAV_FRAME index based on given frame name (If unknown, defaults to GENERIC)
      });

    obstacle_sub =
      node->create_subscription<sensor_msgs::msg::LaserScan>(
      "~/send", 10,
      std::bind(&ObstacleDistancePlugin::obstacle_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr obstacle_sub;

  mavlink::common::MAV_FRAME frame;

  /**
   * @brief Send obstacle distance array to the FCU.
   *
   * Message specification: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
   * @param req	received ObstacleDistance msg
   */
  void obstacle_cb(const sensor_msgs::msg::LaserScan::SharedPtr req)
  {
    mavlink::common::msg::OBSTACLE_DISTANCE obstacle {};

    if (req->ranges.size() <= obstacle.distances.size()) {
      // all distances from sensor will fit in obstacle distance message
      for (size_t i = 0; i < req->ranges.size(); i++) {
        float distance_cm = req->ranges[i] * 1e2;
        if (std::isnan(distance_cm) || distance_cm >= UINT16_MAX || distance_cm < 0) {
          obstacle.distances[i] = UINT16_MAX;
        } else {
          obstacle.distances[i] = static_cast<uint16_t>(distance_cm);
        }
      }
      std::fill(
        obstacle.distances.begin() + req->ranges.size(),
        obstacle.distances.end(), UINT16_MAX);  //!< fill the rest of the array values as "Unknown"

      const float increment_deg = req->angle_increment * RAD_TO_DEG;
      obstacle.increment = static_cast<uint8_t>(increment_deg + 0.5f);  //!< Round to nearest int
      obstacle.increment_f = increment_deg;
    } else {
      // all distances from sensor will not fit so we combine adjacent
      // distances always taking the shortest distance
      const float scale_factor = static_cast<double>(req->ranges.size()) /
        obstacle.distances.size();
      for (size_t i = 0; i < obstacle.distances.size(); i++) {
        obstacle.distances[i] = UINT16_MAX;
        for (size_t j = 0; j < scale_factor; j++) {
          size_t req_index = floor(i * scale_factor + j);
          float distance_cm = req->ranges[req_index] * 1e2;
          if (!std::isnan(distance_cm) && distance_cm < UINT16_MAX && distance_cm > 0) {
            obstacle.distances[i] =
              std::min(obstacle.distances[i], static_cast<uint16_t>(distance_cm));
          }
        }
      }
      const float increment_deg = req->angle_increment * RAD_TO_DEG * scale_factor;
      obstacle.increment = static_cast<uint8_t>(increment_deg + 0.5f);  //!< Round to nearest int
      obstacle.increment_f = increment_deg;
    }

    obstacle.time_usec = get_time_usec(req->header.stamp);                  //!< [microsecs]
    obstacle.sensor_type = utils::enum_value(MAV_DISTANCE_SENSOR::LASER);   //!< defaults is laser
    obstacle.min_distance = req->range_min * 1e2;                           //!< [centimeters]
    obstacle.max_distance = req->range_max * 1e2;                           //!< [centimeters]
    obstacle.frame = utils::enum_value(frame);
    // Assume angle_increment is positive and incoming message is in a FRD/NED frame
    obstacle.angle_offset = req->angle_min * RAD_TO_DEG;                    //!< [degrees]

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "OBSDIST: sensor type: " <<
        utils::to_string_enum<MAV_DISTANCE_SENSOR>(obstacle.sensor_type) <<
        std::endl << obstacle.to_yaml());

    uas->send_message(obstacle);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::ObstacleDistancePlugin)
