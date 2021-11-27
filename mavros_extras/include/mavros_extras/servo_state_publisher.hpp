/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Publish servo states as JointState message
 * @file
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#pragma once

#ifndef MAVROS_EXTRAS__SERVO_STATE_PUBLISHER_HPP_
#define MAVROS_EXTRAS__SERVO_STATE_PUBLISHER_HPP_

#include <yaml-cpp/yaml.h>
#include <urdf/model.h>

#include <algorithm>
#include <memory>
#include <string>
#include <list>
#include <shared_mutex>  // NOLINT

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace mavros
{
namespace extras
{

/**
 * ServoDescription captures configuration for one joint.
 */
class ServoDescription
{
public:
  std::string joint_name;
  float joint_lower;
  float joint_upper;

  size_t rc_channel;

  uint16_t rc_min;
  uint16_t rc_max;
  uint16_t rc_trim;
  uint16_t rc_dz;
  bool rc_rev;

  explicit ServoDescription(std::string joint_name_ = {})
  : joint_name(joint_name_),
    joint_lower(-M_PI / 4),
    joint_upper(M_PI / 4),
    rc_channel(0),
    rc_min(1000),
    rc_max(2000),
    rc_trim(1500),
    rc_dz(0),
    rc_rev(false)
  {}

  ServoDescription(urdf::Model & model, std::string joint_name_, YAML::Node config);

  /**
   * Normalization code taken from PX4 Firmware
   * src/modules/sensors/sensors.cpp Sensors::rc_poll() line 1966
   */
  inline float normalize(uint16_t pwm)
  {
    // 1) fix bounds
    pwm = std::max(pwm, rc_min);
    pwm = std::min(pwm, rc_max);

    // 2) scale around mid point
    float chan;
    if (pwm > (rc_trim + rc_dz)) {
      chan = (pwm - rc_trim - rc_dz) / static_cast<float>(rc_max - rc_trim - rc_dz);
    } else if (pwm < (rc_trim - rc_dz)) {
      chan = (pwm - rc_trim + rc_dz) / static_cast<float>(rc_trim - rc_min - rc_dz);
    } else {
      chan = 0.0;
    }

    if (rc_rev) {
      chan *= -1;
    }

    if (!std::isfinite(chan)) {
      chan = 0.0;
    }

    return chan;
  }

  inline float calculate_position(uint16_t pwm)
  {
    float channel = normalize(pwm);

    // not sure should i differently map -1..0 and 0..1
    // for now there arduino map() (explicit)
    float position = (channel + 1.0) * (joint_upper - joint_lower) / (1.0 + 1.0) + joint_lower;

    return position;
  }
};

/**
 * ServoStatePublisher class implements servo_state_publisher node
 *
 * That node translates RC Servo outputs to URDF Joint states
 */
class ServoStatePublisher : public rclcpp::Node
{
public:
  explicit ServoStatePublisher(const std::string & node_name = "servo_state_publisher")
  : ServoStatePublisher(rclcpp::NodeOptions(), node_name) {}

  explicit ServoStatePublisher(
    const rclcpp::NodeOptions & options,
    const std::string & node_name = "servo_state_publisher");

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub;
  rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr rc_out_sub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

  std::shared_mutex mutex;
  std::list<ServoDescription> servos;

  void robot_description_cb(const std_msgs::msg::String::SharedPtr msg);
  void rc_out_cb(const mavros_msgs::msg::RCOut::SharedPtr msg);
};

}   // namespace extras
}   // namespace mavros

#endif  // MAVROS_EXTRAS__SERVO_STATE_PUBLISHER_HPP_
