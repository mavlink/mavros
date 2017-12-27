/**
 * @brief Command Ack plugin
 * @file command_ack.cpp
 * @author Pavlo Kolomiiets <pkolomiets@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Pavlo Kolomiiets
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CommandAck.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief Command Ack plugin.
 *
 * Publish COMMAND_ACK message data from FCU to ROS.
 */
class CommandAckPlugin : public plugin::PluginBase {
public:
  CommandAckPlugin() : PluginBase(),
    nh("~")
  { }

  void initialize(UAS &uas_)
  {
    PluginBase::initialize(uas_);

    command_ack_pub = nh.advertise<mavros_msgs::CommandAck>("cmd_ack", 10, true);
  }

  Subscriptions get_subscriptions()
  {
    return {
      make_handler(&CommandAckPlugin::handle_command_ack)
    };
  }

private:
  ros::NodeHandle nh;
  ros::Publisher command_ack_pub;

  /* -*- message handlers -*- */

  void handle_command_ack(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_ACK &ack)
  {
    auto ack_msg = boost::make_shared<mavros_msgs::CommandAck>();
    ack_msg->header.stamp = ros::Time::now();

    ack_msg->command = ack.command;
    ack_msg->result = ack.result;
    ack_msg->progress = ack.progress;
    ack_msg->result_param2 = ack.result_param2;

    command_ack_pub.publish(ack_msg);
  }
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CommandAckPlugin, mavros::plugin::PluginBase)
