/*
 * Copyright 2019 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Mount Control plugin
 * @file mount_control.cpp
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <tf2_eigen/tf2_eigen.h>

#include <memory>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/msg/mount_control.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/srv/mount_configure.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

//! Mavlink enumerations
using mavlink::common::MAV_MOUNT_MODE;
using mavlink::common::MAV_CMD;
using utils::enum_value;

/**
 * @brief Mount Control plugin
 * @plugin mount_control
 *
 * Publishes Mission commands to control the camera or antenna mount.
 * @see command_cb()
 */
class MountControlPlugin : public plugin::Plugin
{
public:
  explicit MountControlPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "mount_control")
  {
    command_sub = node->create_subscription<mavros_msgs::msg::MountControl>(
      "~/command", 10, std::bind(
        &MountControlPlugin::command_cb, this,
        _1));

    mount_orientation_pub = node->create_publisher<geometry_msgs::msg::Quaternion>(
      "~/orientation",
      10);
    mount_status_pub = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("~/status", 10);

    configure_srv = node->create_service<mavros_msgs::srv::MountConfigure>(
      "~/configure", std::bind(
        &MountControlPlugin::mount_configure_cb,
        this, _1, _2));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&MountControlPlugin::handle_mount_orientation),
      make_handler(&MountControlPlugin::handle_mount_status)
    };
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::MountControl>::SharedPtr command_sub;

  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr mount_orientation_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mount_status_pub;

  rclcpp::Service<mavros_msgs::srv::MountConfigure>::SharedPtr configure_srv;

  /**
   * @brief Publish the mount orientation
   *
   * Message specification: https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION
   * @param msg   the mavlink message
   * @param mo	received MountOrientation msg
   */
  void handle_mount_orientation(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::MOUNT_ORIENTATION & mo,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto q = ftf::quaternion_from_rpy(Eigen::Vector3d(mo.roll, mo.pitch, mo.yaw) * M_PI / 180.0);

    geometry_msgs::msg::Quaternion quaternion_msg = tf2::toMsg(q);

    mount_orientation_pub->publish(quaternion_msg);
  }

  /**
   * @brief Publish the mount status
   *
   * @param msg   the mavlink message
   * @param ms	received MountStatus msg
   */
  void handle_mount_status(
    const mavlink::mavlink_message_t * mmsg [[maybe_unused]],
    mavlink::ardupilotmega::msg::MOUNT_STATUS & ms,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    geometry_msgs::msg::Vector3Stamped publish_msg;

    publish_msg.header.stamp = node->now();
    publish_msg.header.frame_id = std::to_string(ms.target_component);

    auto vec = Eigen::Vector3d(ms.pointing_b, ms.pointing_a, ms.pointing_c) * M_PI / 18000.0;
    tf2::toMsg(vec, publish_msg.vector);

    mount_status_pub->publish(publish_msg);

    // pointing_X is cdeg
    auto q = ftf::quaternion_from_rpy(
      Eigen::Vector3d(
        ms.pointing_b, ms.pointing_a,
        ms.pointing_c) * M_PI / 18000.0);
    geometry_msgs::msg::Quaternion quaternion_msg = tf2::toMsg(q);

    mount_orientation_pub->publish(quaternion_msg);
  }

  /**
   * @brief Send mount control commands to vehicle
   *
   * Message specification: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL
   * @param req	received MountControl msg
   */
  void command_cb(const mavros_msgs::msg::MountControl::SharedPtr req)
  {
    mavlink::common::msg::COMMAND_LONG cmd {};

    uas->msg_set_target(cmd);
    cmd.command = enum_value(MAV_CMD::DO_MOUNT_CONTROL);
    cmd.param1 = req->pitch;
    cmd.param2 = req->roll;
    cmd.param3 = req->yaw;
    cmd.param4 = req->altitude;         //
    cmd.param5 = req->latitude;         // latitude in degrees * 1E7
    cmd.param6 = req->longitude;        // longitude in degrees * 1E7
    cmd.param7 = req->mode;             // MAV_MOUNT_MODE

    uas->send_message(cmd);
  }

  void mount_configure_cb(
    mavros_msgs::srv::MountConfigure::Request::SharedPtr req,
    mavros_msgs::srv::MountConfigure::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;

    try {
      auto client = node->create_client<mavros_msgs::srv::CommandLong>("cmd/command");

      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();

      cmdrq->broadcast = false;
      cmdrq->command = enum_value(MAV_CMD::DO_MOUNT_CONFIGURE);
      cmdrq->confirmation = false;
      cmdrq->param1 = req->mode;
      cmdrq->param2 = req->stabilize_roll;
      cmdrq->param3 = req->stabilize_pitch;
      cmdrq->param4 = req->stabilize_yaw;
      cmdrq->param5 = req->roll_input;
      cmdrq->param6 = req->pitch_input;
      cmdrq->param7 = req->yaw_input;

      RCLCPP_DEBUG(get_logger(), "MountConfigure: Request mode %u ", req->mode);
      auto future = client->async_send_request(cmdrq);
      auto response = future.get();
      res->success = response->success;
    } catch (std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "MountConfigure: %s", ex.what());
    }

    RCLCPP_ERROR_EXPRESSION(
      get_logger(), !res->success, "MountConfigure: command plugin service call failed!");
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::MountControlPlugin)
