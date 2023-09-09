/*
 * Copyright 2019 Jaeyoung Lim.
 * Copyright 2021 Dr.-Ing. Amilcar do Carmo Lucas <amilcar.lucas@iav.de>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Mount Control plugin
 * @file mount_control.cpp
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Dr.-Ing. Amilcar do Carmo Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */

#include <memory>

#include "tf2_eigen/tf2_eigen.hpp"
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
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT

//! Mavlink enumerations
using mavlink::common::MAV_MOUNT_MODE;
using mavlink::common::MAV_CMD;
using utils::enum_value;

/**
 * @brief Mount diagnostic updater
 */
class MountStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
  explicit MountStatusDiag(const std::string & name)
  : diagnostic_updater::DiagnosticTask(name),
    _last_orientation_update(0, 0),
    _debounce_s(NAN),
    _roll_deg(NAN),
    _pitch_deg(NAN),
    _yaw_deg(NAN),
    _setpoint_roll_deg(NAN),
    _setpoint_pitch_deg(NAN),
    _setpoint_yaw_deg(NAN),
    _err_threshold_deg(NAN),
    _error_detected(false),
    _mode(255)
  {}

  void set_err_threshold_deg(float threshold_deg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    _err_threshold_deg = threshold_deg;
  }

  void set_debounce_s(double debounce_s)
  {
    std::lock_guard<std::mutex> lock(mutex);
    _debounce_s = debounce_s;
  }

  void set_status(float roll_deg, float pitch_deg, float yaw_deg, rclcpp::Time timestamp)
  {
    std::lock_guard<std::mutex> lock(mutex);
    _roll_deg = roll_deg;
    _pitch_deg = pitch_deg;
    _yaw_deg = yaw_deg;
    _last_orientation_update = timestamp;
  }

  void set_setpoint(float roll_deg, float pitch_deg, float yaw_deg, uint8_t mode)
  {
    std::lock_guard<std::mutex> lock(mutex);
    _setpoint_roll_deg = roll_deg;
    _setpoint_pitch_deg = pitch_deg;
    _setpoint_yaw_deg = yaw_deg;
    _mode = mode;
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    float roll_err_deg;
    float pitch_err_deg;
    float yaw_err_deg;
    bool error_detected = false;
    bool stale = false;

    if (_mode != mavros_msgs::msg::MountControl::MAV_MOUNT_MODE_MAVLINK_TARGETING) {
      // Can only directly compare the MAV_CMD_DO_MOUNT_CONTROL angles with
      // the MOUNT_ORIENTATION angles when in MAVLINK_TARGETING mode
      stat.summary(DiagnosticStatus::WARN, "Can not diagnose in this targeting mode");
      stat.addf("Mode", "%d", _mode);
      return;
    }

    const rclcpp::Time now = clock.now();
    {
      std::lock_guard<std::mutex> lock(mutex);
      roll_err_deg = _setpoint_roll_deg - _roll_deg;
      pitch_err_deg = _setpoint_pitch_deg - _pitch_deg;
      yaw_err_deg = _setpoint_yaw_deg - _yaw_deg;

      // detect errors (setpoint != current angle)
      if (fabs(roll_err_deg) > _err_threshold_deg) {
        error_detected = true;
      }
      if (fabs(pitch_err_deg) > _err_threshold_deg) {
        error_detected = true;
      }
      if (fabs(yaw_err_deg) > _err_threshold_deg) {
        error_detected = true;
      }
      if (now - _last_orientation_update > rclcpp::Duration(5s)) {
        stale = true;
      }
      // accessing the _debounce_s variable should be done inside this mutex,
      // but we can treat it as an atomic variable, and save the trouble
    }

    // detect error state changes
    if (!_error_detected && error_detected) {
      _error_started = now;
      _error_detected = true;
    }
    if (_error_detected && !error_detected) {
      _error_detected = false;
    }

    // debounce errors
    // *INDENT-OFF*
    if (stale) {
      stat.summary(DiagnosticStatus::STALE, "No MOUNT_ORIENTATION received in the last 5 s");
    } else if (_error_detected &&
      (now - _error_started > rclcpp::Duration(std::chrono::duration<double>(_debounce_s)))) {
      stat.summary(DiagnosticStatus::ERROR, "angle error too high");
    } else {
      stat.summary(DiagnosticStatus::OK, "Normal");
    }
    // *INDENT-ON*

    stat.addf("Roll err (deg)", "%.1f", roll_err_deg);
    stat.addf("Pitch err (deg)", "%.1f", pitch_err_deg);
    stat.addf("Yaw err (deg)", "%.1f", yaw_err_deg);
  }

private:
  std::mutex mutex;
  rclcpp::Clock clock;
  rclcpp::Time _error_started;
  rclcpp::Time _last_orientation_update;
  double _debounce_s;
  float _roll_deg;
  float _pitch_deg;
  float _yaw_deg;
  float _setpoint_roll_deg;
  float _setpoint_pitch_deg;
  float _setpoint_yaw_deg;
  float _err_threshold_deg;
  bool _error_detected;
  uint8_t _mode;
};

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
  : Plugin(uas_, "mount_control"),
    mount_diag("Mount")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "negate_measured_roll", false, [&](const rclcpp::Parameter & p) {
        negate_measured_roll = p.as_bool();
      });
    node_declare_and_watch_parameter(
      "negate_measured_pitch", false, [&](const rclcpp::Parameter & p) {
        negate_measured_pitch = p.as_bool();
      });
    node_declare_and_watch_parameter(
      "negate_measured_yaw", false, [&](const rclcpp::Parameter & p) {
        negate_measured_yaw = p.as_bool();
      });

    node_declare_and_watch_parameter(
      "debounce_s", 4.0, [&](const rclcpp::Parameter & p) {
        auto debounce_s = p.as_double();
        mount_diag.set_debounce_s(debounce_s);
      });
    node_declare_and_watch_parameter(
      "err_threshold_deg", 10.0, [&](const rclcpp::Parameter & p) {
        auto err_threshold_deg = p.as_double();
        mount_diag.set_err_threshold_deg(err_threshold_deg);
      });
    node_declare_and_watch_parameter(
      "disable_diag", false, [&](const rclcpp::Parameter & p) {
        auto disable_diag = p.as_bool();

        if (!disable_diag) {
          uas->diagnostic_updater.add(mount_diag);
        } else {
          uas->diagnostic_updater.removeByName(mount_diag.getName());
        }
      });

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

  MountStatusDiag mount_diag;
  bool negate_measured_roll;
  bool negate_measured_pitch;
  bool negate_measured_yaw;

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
    const auto timestamp = node->now();
    // some gimbals send negated/inverted angle measurements,
    // correct that to obey the MAVLink frame convention
    if (negate_measured_roll) {
      mo.roll = -mo.roll;
    }
    if (negate_measured_pitch) {
      mo.pitch = -mo.pitch;
    }
    if (negate_measured_yaw) {
      mo.yaw = -mo.yaw;
      mo.yaw_absolute = -mo.yaw_absolute;
    }

    auto q = ftf::quaternion_from_rpy(Eigen::Vector3d(mo.roll, mo.pitch, mo.yaw) * M_PI / 180.0);

    geometry_msgs::msg::Quaternion quaternion_msg = tf2::toMsg(q);

    mount_orientation_pub->publish(quaternion_msg);
    mount_diag.set_status(mo.roll, mo.pitch, mo.yaw_absolute, timestamp);
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

    mount_diag.set_setpoint(req->roll * 0.01f, req->pitch * 0.01f, req->yaw * 0.01f, req->mode);
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
