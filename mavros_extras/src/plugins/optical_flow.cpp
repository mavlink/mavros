/*
 * Copyright 2023 Vladislav Chuvarjov
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief OpticalFlow plugin
 * @file optical_flow.cpp
 * @author Vladislav Chuvarjov <chuvarevanv@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/optical_flow.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Optical Flow custom plugin
 * @plugin optical_flow
 *
 * This plugin can publish data from OpticalFlow camera to ROS
 */
class OpticalFlowPlugin : public plugin::Plugin
{
public:
  explicit OpticalFlowPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "optical_flow"),
    ranger_fov(0.0),
    ranger_min_range(0.3),
    ranger_max_range(5.0)
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "frame_id", "optical_flow", [&](const rclcpp::Parameter & p) {
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

    flow_pub = node->create_publisher<mavros_msgs::msg::OpticalFlow>(
      "~/raw/optical_flow", 10);
    range_pub = node->create_publisher<sensor_msgs::msg::Range>("~/ground_distance", 10);

    flow_sub = node->create_subscription<mavros_msgs::msg::OpticalFlow>(
      "~/raw/send", 1, std::bind(
        &OpticalFlowPlugin::send_cb, this,
        _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&OpticalFlowPlugin::handle_optical_flow)
    };
  }

private:
  std::string frame_id;

  double ranger_fov;
  double ranger_min_range;
  double ranger_max_range;

  rclcpp::Publisher<mavros_msgs::msg::OpticalFlow>::SharedPtr flow_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub;
  rclcpp::Subscription<mavros_msgs::msg::OpticalFlow>::SharedPtr flow_sub;

  void handle_optical_flow(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::OPTICAL_FLOW & flow,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto header = uas->synchronized_header(frame_id, flow.time_usec);

    /**
     * Raw message with axes mapped to ROS conventions and temp in degrees celsius.
     *
     * The optical flow camera is essentially an angular sensor, so conversion is like
     * gyroscope. (aircraft -> baselink)
     */
    auto int_flow = ftf::transform_frame_aircraft_baselink(
      Eigen::Vector3d(
        flow.flow_x,
        flow.flow_y,
        0.0));

    auto int_flow_comp_m_xy = ftf::transform_frame_aircraft_baselink(
      Eigen::Vector3d(
        flow.flow_comp_m_x,
        flow.flow_comp_m_y,
        0.0));

    auto int_flow_rate_xy = ftf::transform_frame_aircraft_baselink(
      Eigen::Vector3d(
        flow.flow_rate_x,
        flow.flow_rate_y,
        0.0));

    auto flow_msg = mavros_msgs::msg::OpticalFlow();

    flow_msg.header = header;

    flow_msg.flow.x = int_flow.x();
    flow_msg.flow.y = int_flow.y();
    flow_msg.flow_comp_m.x = int_flow_comp_m_xy.x();
    flow_msg.flow_comp_m.y = int_flow_comp_m_xy.y();
    flow_msg.flow_rate.x = int_flow_rate_xy.x();
    flow_msg.flow_rate.y = int_flow_rate_xy.y();
    flow_msg.ground_distance = flow.ground_distance;
    flow_msg.quality = flow.quality;

    flow_pub->publish(flow_msg);

    // Rangefinder
    /**
     * @todo: use distance_sensor plugin only to publish this data
     * (which receives DISTANCE_SENSOR msg with multiple rangefinder
     * sensors data)
     *
     */
    auto range_msg = sensor_msgs::msg::Range();

    range_msg.header = header;

    range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range_msg.field_of_view = ranger_fov;
    range_msg.min_range = ranger_min_range;
    range_msg.max_range = ranger_max_range;
    range_msg.range = flow.ground_distance;

    range_pub->publish(range_msg);
  }

  void send_cb(const mavros_msgs::msg::OpticalFlow::SharedPtr msg)
  {
    mavlink::common::msg::OPTICAL_FLOW flow_msg = {};

    auto int_flow =
      ftf::transform_frame_baselink_aircraft(mavros::ftf::to_eigen(msg->flow));
    auto int_flow_comp_m_xy =
      ftf::transform_frame_baselink_aircraft(mavros::ftf::to_eigen(msg->flow_comp_m));
    auto int_flow_rate_xy =
      ftf::transform_frame_baselink_aircraft(mavros::ftf::to_eigen(msg->flow_rate));

    flow_msg.time_usec = get_time_usec(msg->header.stamp);
    flow_msg.sensor_id = 0;
    flow_msg.flow_x = int_flow.x();
    flow_msg.flow_y = int_flow.y();
    flow_msg.flow_comp_m_x = int_flow_comp_m_xy.x();
    flow_msg.flow_comp_m_y = int_flow_comp_m_xy.y();
    flow_msg.quality = msg->quality;
    flow_msg.ground_distance = msg->ground_distance;   // temperature in centi-degrees Celsius
    flow_msg.flow_rate_x = int_flow_rate_xy.x();
    flow_msg.flow_rate_y = int_flow_rate_xy.y();

    uas->send_message(flow_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::OpticalFlowPlugin)
