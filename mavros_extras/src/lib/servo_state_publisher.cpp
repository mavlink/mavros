/*
 * Copyright 2015,2021 Vladimir Ermakov
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

#include <string>

#include "mavros_extras/servo_state_publisher.hpp"

using namespace mavros::extras;     // NOLINT
using namespace std::placeholders;  // NOLINT
using rclcpp::QoS;

ServoDescription::ServoDescription(urdf::Model & model, std::string joint_name_, YAML::Node config)
: ServoDescription(joint_name_)
{
  if (!config["rc_channel"]) {
    throw std::invalid_argument("`rc_channel` field required");
  }

  rc_channel = config["rc_channel"].as<int>();
  rc_min = config["rc_min"].as<int>(1000);
  rc_max = config["rc_max"].as<int>(2000);

  if (auto rc_trim_n = config["rc_trim"]; !rc_trim_n) {
    rc_trim = rc_min + (rc_max - rc_min) / 2;
  } else {
    rc_trim = rc_trim_n.as<int>();
  }

  rc_dz = config["rc_dz"].as<int>(0);
  rc_rev = config["rc_rev"].as<bool>(false);

  auto joint = model.getJoint(joint_name);
  if (!joint) {
    throw std::runtime_error("Joint " + joint_name + " is not found in URDF");
  }
  if (!joint->limits) {
    throw std::runtime_error("URDF for joint " + joint_name + " must provide <limit>");
  }

  joint_lower = joint->limits->lower;
  joint_upper = joint->limits->upper;
}

ServoStatePublisher::ServoStatePublisher(
  const rclcpp::NodeOptions & options,
  const std::string & node_name)
: rclcpp::Node(node_name, options)
{
  // Declare configuration parameter
  this->declare_parameter("config", "");

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto description_qos = QoS(1).transient_local();

  // robot_state_publisher sends URDF in that topic
  robot_description_sub = this->create_subscription<std_msgs::msg::String>(
    "robot_description",
    description_qos, std::bind(
      &ServoStatePublisher::robot_description_cb, this,
      _1));

  // Create topics
  rc_out_sub =
    this->create_subscription<mavros_msgs::msg::RCOut>(
    "rc_out", sensor_qos,
    std::bind(&ServoStatePublisher::rc_out_cb, this, _1));
  joint_states_pub =
    this->create_publisher<sensor_msgs::msg::JointState>("joint_states", sensor_qos);
}

void ServoStatePublisher::robot_description_cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::unique_lock lock(mutex);

  servos.clear();

  // 1. Load model
  urdf::Model model;
  if (!model.initString(msg->data)) {
    throw std::runtime_error("Unable to initialize urdf::Model from robot description");
  }

  // 2. Load mapping config
  YAML::Node root_node;
  {
    std::string configYaml; this->get_parameter("config", configYaml);
    root_node = YAML::Load(configYaml);
  }
  if (!root_node.IsMap()) {
    throw std::runtime_error("Mapping config must be a map");
  }

  // 3. Load servos
  RCLCPP_INFO(get_logger(), "SSP: URDF robot: %s", model.getName().c_str());
  for (auto it = root_node.begin(); it != root_node.end(); ++it) {
    auto joint_name = it->first.as<std::string>();
    RCLCPP_INFO_STREAM(get_logger(), "SSP: " << joint_name << ": Loading joint: " << it->second);

    try {
      auto joint = servos.emplace_back(model, joint_name, it->second);
      RCLCPP_INFO(
        get_logger(), "SSP: joint '%s' (RC%zu) loaded",
        joint_name.c_str(), joint.rc_channel);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "SSP: " << joint_name << ": Failed to load mapping: " << ex.what());
    }
  }
}

void ServoStatePublisher::rc_out_cb(const mavros_msgs::msg::RCOut::SharedPtr msg)
{
  std::shared_lock lock(mutex);

  if (msg->channels.empty()) {
    return;         // nothing to do
  }

  auto states = sensor_msgs::msg::JointState();
  states.header.stamp = msg->header.stamp;

  for (auto & desc : servos) {
    if (!(desc.rc_channel != 0 && desc.rc_channel <= msg->channels.size())) {
      continue;     // prevent crash on servos not in that message
    }
    uint16_t pwm = msg->channels[desc.rc_channel - 1];
    if (pwm == 0 || pwm == UINT16_MAX) {
      continue;     // exclude unset channels
    }
    states.name.emplace_back(desc.joint_name);
    states.position.emplace_back(desc.calculate_position(pwm));
  }

  joint_states_pub->publish(states);
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(mavros::extras::ServoStatePublisher)
