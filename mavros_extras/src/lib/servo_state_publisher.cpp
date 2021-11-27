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


#include "mavros/servo_state_publisher.hpp"

using namespace mavros::extras;     // NOLINT
using namespace std::placeholders;  // NOLINT
using rclcpp::QoS;


ServoDescription::ServoDescription(std::string joint_name_, YAML::Node config)
{
}

ServoStatePublisher::ServoStatePublisher(
  const rclcpp::NodeOptions & options,
  const std::string & node_name = "servo_state_publisher")
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
      &ServoStatePublisher::robot_description_sub, this,
      _1));

  // Create topics
  rc_out_sub =
    this->create_subscription<mavros_msgs::msg::RCOut>(
    "rc_out", sensor_qos,
    std::bind(&ServoStatePublisher::rc_out_cb, this, _1));
  joint_states_pub =
    this->create_publisher<sensor_msgs::msg::JointState>("joint_states", sensor_qos);
}


void ServoDescriptionr::robot_description_sub(const std_msgs::msg::String::SharedPtr msg)
{
  std::unique_lock lock(mutex);

  servos.clear();

  urdf::Model model;

#if 0
  urdf::Model model;
  ROS_INFO("SSP: URDF robot: %s", model.getName().c_str());

  for (auto & pair : param_dict) {
    ROS_DEBUG("SSP: Loading joint: %s", pair.first.c_str());

    // inefficient, but easier to program
    ros::NodeHandle pnh(priv_nh, pair.first);

    bool rc_rev;
    int rc_channel, rc_min, rc_max, rc_trim, rc_dz;

    if (!pnh.getParam("rc_channel", rc_channel)) {
      ROS_ERROR("SSP: '%s' should provice rc_channel", pair.first.c_str());
      continue;
    }

    pnh.param("rc_min", rc_min, 1000);
    pnh.param("rc_max", rc_max, 2000);
    if (!pnh.getParam("rc_trim", rc_trim)) {
      rc_trim = rc_min + (rc_max - rc_min) / 2;
    }

    pnh.param("rc_dz", rc_dz, 0);
    pnh.param("rc_rev", rc_rev, false);

    auto joint = model.getJoint(pair.first);
    if (!joint) {
      ROS_ERROR("SSP: URDF: there no joint '%s'", pair.first.c_str());
      continue;
    }
    if (!joint->limits) {
      ROS_ERROR("SSP: URDF: joint '%s' should provide <limit>", pair.first.c_str());
      continue;
    }

    double lower = joint->limits->lower;
    double upper = joint->limits->upper;

    servos.emplace_back(
      pair.first, lower, upper, rc_channel, rc_min, rc_max, rc_trim, rc_dz,
      rc_rev);
    ROS_INFO("SSP: joint '%s' (RC%d) loaded", pair.first.c_str(), rc_channel);
  }
#endif
}

void ServoStatePublisher::rc_out_cb(const mavros_msgs::msg::RCOut::SharedPtr msg)
{
  if (msg->channels.empty()) {
    return;                             // nothing to do

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
