/*
 * Copyright 2014 M.H.Kabir.
 * Copyright 2016,2021 Vladimir Ermakov
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief PX4Flow plugin
 * @file px4flow.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/optical_flow_rad.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief PX4 Optical Flow plugin
 * @plugin px4flow
 *
 * This plugin can publish data from PX4Flow camera to ROS
 */
class PX4FlowPlugin : public plugin::Plugin
{
public:
  explicit PX4FlowPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "px4flow"),
    ranger_fov(0.0),
    ranger_min_range(0.3),
    ranger_max_range(5.0)
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "frame_id", "px4flow", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    /**
     * @note Default rangefinder is Maxbotix HRLV-EZ4
     * This is a narrow beam (60cm wide at 5 meters,
     * but also at 1 meter). 6.8 degrees at 5 meters, 31 degrees
     * at 1 meter
     */
    node_declare_and_watch_parameter(
      "ranger_fov", 0.119428926, [&](const rclcpp::Parameter & p) {
        ranger_fov = p.as_double();
      });

    node_declare_and_watch_parameter(
      "ranger_min_range", 0.3, [&](const rclcpp::Parameter & p) {
        ranger_min_range = p.as_double();
      });

    node_declare_and_watch_parameter(
      "ranger_max_range", 5.0, [&](const rclcpp::Parameter & p) {
        ranger_max_range = p.as_double();
      });

    flow_rad_pub = node->create_publisher<mavros_msgs::msg::OpticalFlowRad>(
      "~/raw/optical_flow_rad", 10);
    range_pub = node->create_publisher<sensor_msgs::msg::Range>("~/ground_distance", 10);
    temp_pub = node->create_publisher<sensor_msgs::msg::Temperature>("~/temperature", 10);

    flow_rad_sub = node->create_subscription<mavros_msgs::msg::OpticalFlowRad>(
      "~/raw/send", 1, std::bind(
        &PX4FlowPlugin::send_cb, this,
        _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&PX4FlowPlugin::handle_optical_flow_rad)
    };
  }

private:
  std::string frame_id;

  double ranger_fov;
  double ranger_min_range;
  double ranger_max_range;

  rclcpp::Publisher<mavros_msgs::msg::OpticalFlowRad>::SharedPtr flow_rad_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
  rclcpp::Subscription<mavros_msgs::msg::OpticalFlowRad>::SharedPtr flow_rad_sub;

  void handle_optical_flow_rad(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::OPTICAL_FLOW_RAD & flow_rad,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto header = uas->synchronized_header(frame_id, flow_rad.time_usec);

    /**
     * Raw message with axes mapped to ROS conventions and temp in degrees celsius.
     *
     * The optical flow camera is essentially an angular sensor, so conversion is like
     * gyroscope. (aircraft -> baselink)
     */
    auto int_xy = ftf::transform_frame_aircraft_baselink(
      Eigen::Vector3d(
        flow_rad.integrated_x,
        flow_rad.integrated_y,
        0.0));
    auto int_gyro = ftf::transform_frame_aircraft_baselink(
      Eigen::Vector3d(
        flow_rad.integrated_xgyro,
        flow_rad.integrated_ygyro,
        flow_rad.integrated_zgyro));

    auto flow_rad_msg = mavros_msgs::msg::OpticalFlowRad();

    flow_rad_msg.header = header;
    flow_rad_msg.integration_time_us = flow_rad.integration_time_us;

    flow_rad_msg.integrated_x = int_xy.x();
    flow_rad_msg.integrated_y = int_xy.y();

    flow_rad_msg.integrated_xgyro = int_gyro.x();
    flow_rad_msg.integrated_ygyro = int_gyro.y();
    flow_rad_msg.integrated_zgyro = int_gyro.z();

    flow_rad_msg.temperature = flow_rad.temperature / 100.0f;   // in degrees celsius
    flow_rad_msg.time_delta_distance_us = flow_rad.time_delta_distance_us;
    flow_rad_msg.distance = flow_rad.distance;
    flow_rad_msg.quality = flow_rad.quality;

    flow_rad_pub->publish(flow_rad_msg);

    // Temperature
    auto temp_msg = sensor_msgs::msg::Temperature();

    temp_msg.header = header;
    temp_msg.temperature = flow_rad_msg.temperature;

    temp_pub->publish(temp_msg);

    // Rangefinder
    /**
     * @todo: use distance_sensor plugin only to publish this data
     * (which receives DISTANCE_SENSOR msg with multiple rangefinder
     * sensors data)
     *
     * @todo: suggest modification on MAVLink OPTICAL_FLOW_RAD msg
     * which removes sonar data fields from it
     */
    auto range_msg = sensor_msgs::msg::Range();

    range_msg.header = header;

    range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range_msg.field_of_view = ranger_fov;
    range_msg.min_range = ranger_min_range;
    range_msg.max_range = ranger_max_range;
    range_msg.range = flow_rad.distance;

    range_pub->publish(range_msg);
  }

  void send_cb(const mavros_msgs::msg::OpticalFlowRad::SharedPtr msg)
  {
    mavlink::common::msg::OPTICAL_FLOW_RAD flow_rad_msg = {};

    auto int_xy = ftf::transform_frame_baselink_aircraft(
      Eigen::Vector3d(
        msg->integrated_x,
        msg->integrated_y,
        0.0));
    auto int_gyro = ftf::transform_frame_baselink_aircraft(
      Eigen::Vector3d(
        msg->integrated_xgyro,
        msg->integrated_ygyro,
        msg->integrated_zgyro));

    flow_rad_msg.time_usec = get_time_usec(msg->header.stamp);
    flow_rad_msg.sensor_id = 0;
    flow_rad_msg.integration_time_us = msg->integration_time_us;
    flow_rad_msg.integrated_x = int_xy.x();
    flow_rad_msg.integrated_y = int_xy.y();
    flow_rad_msg.integrated_xgyro = int_gyro.x();
    flow_rad_msg.integrated_ygyro = int_gyro.y();
    flow_rad_msg.integrated_zgyro = int_gyro.z();
    flow_rad_msg.temperature = msg->temperature * 100.0f;   // temperature in centi-degrees Celsius
    flow_rad_msg.quality = msg->quality;
    flow_rad_msg.time_delta_distance_us = msg->time_delta_distance_us;
    flow_rad_msg.distance = msg->distance;

    uas->send_message(flow_rad_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::PX4FlowPlugin)
