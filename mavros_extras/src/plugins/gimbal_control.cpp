/*
 * Copyright 2023 Adinkra Inc.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Gimbal Control Plugin
 * @file gimbal_control.cpp
 * @author Mark Beaty <mark.beaty@adinkratech.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <tf2_eigen/tf2_eigen.hpp>

#include <memory>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/srv/command_long.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "mavros_msgs/msg/gimbal_device_attitude_status.hpp"
#include "mavros_msgs/msg/gimbal_device_set_attitude.hpp"
#include "mavros_msgs/msg/gimbal_device_information.hpp"
#include "mavros_msgs/msg/gimbal_manager_set_attitude.hpp"
#include "mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp"
#include "mavros_msgs/msg/gimbal_manager_status.hpp"
#include "mavros_msgs/msg/gimbal_manager_information.hpp"

#include "mavros_msgs/srv/gimbal_get_information.hpp"
#include "mavros_msgs/srv/gimbal_manager_configure.hpp"
#include "mavros_msgs/srv/gimbal_manager_pitchyaw.hpp"
#include "mavros_msgs/srv/gimbal_manager_set_roi.hpp"
#include "mavros_msgs/srv/gimbal_manager_camera_track.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

// Mavlink enumerations
using mavlink::common::GIMBAL_MANAGER_FLAGS;
using mavlink::common::GIMBAL_MANAGER_CAP_FLAGS;
using mavlink::common::GIMBAL_DEVICE_CAP_FLAGS;
using mavlink::common::GIMBAL_DEVICE_FLAGS;
using mavlink::common::GIMBAL_DEVICE_ERROR_FLAGS;
using mavlink::common::MAV_CMD;
using utils::enum_value;
using uas::s_shared_lock;

/**
 * @brief Gimbal Control Plugin
 * @plugin gimbal_control
 *
 * Adds support for Mavlink Gimbal Protocol v2.
 * Also publishes gimbal pose to TF when parameter tf_send==true
 */
class GimbalControlPlugin : public plugin::Plugin
{
public:
  explicit GimbalControlPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "gimbal_control"),
    tf_send(false)
  {
    /**
     * Sample service calls for interfacing with a Gimbal Protocol v2 gimbal:
     * This service must be called first to take control of the gimbal:
     * ros2 service call /mavros/gimbal_control/manager/configure mavros_msgs/srv/GimbalManagerConfigure "{sysid_primary: -2, compid_primary: 191, sysid_secondary: -1, compid_secondary: -1, gimbal_device_id: 0}"
     * Set pitch and yaw:
     * ros2 service call /mavros/gimbal_control/manager/pitchyaw mavros_msgs/srv/GimbalManagerPitchyaw "{pitch: -45, yaw: 90, pitch_rate: -0.2, yaw_rate: -0.2, flags: 0, gimbal_device_id: 0}"
     * Set region of intrest for tracking:
     * ros2 service call /mavros/gimbal_control/manager/set_roi mavros_msgs/srv/GimbalManagerSetRoi "{mode: 0, gimbal_device_id: 0, latitude: x, longitude: y, altitude: z}"
    */
    // Callback group for supporting nested service calls
    cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Parameter for msg header frame
    enable_node_watch_parameters();
    node_declare_and_watch_parameter(
      "frame_id", "base_link_frd", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    // Important tf subsection
    // Report the transform from base_link to gimbal here.
    node_declare_and_watch_parameter(
      "tf.send", false, [&](const rclcpp::Parameter & p) {
        tf_send = p.as_bool();
      });
    node_declare_and_watch_parameter(
      "tf.frame_id", "base_link_frd", [&](const rclcpp::Parameter & p) {
        tf_frame_id = p.as_string();
      });

    // Subscribers
    // --Not successfully validated--
    set_device_attitude_sub = node->create_subscription<mavros_msgs::msg::GimbalDeviceSetAttitude>(
      "~/device/set_attitude", 10, std::bind(
        &GimbalControlPlugin::device_set_attitude_cb, this,
        _1));

    // --Not successfully validated--
    set_manager_attitude_sub =
      node->create_subscription<mavros_msgs::msg::GimbalManagerSetAttitude>(
      "~/manager/set_attitude", 10, std::bind(
        &GimbalControlPlugin::manager_set_attitude_cb, this,
        _1));

    // --Not successfully validated--
    set_manager_pitchyaw_sub =
      node->create_subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
      "~/manager/set_pitchyaw", 10, std::bind(
        &GimbalControlPlugin::manager_set_pitchyaw_cb, this,
        _1));

    // --Not successfully validated-- also note that the message is the same as pitchyaw and will likely change
    set_manager_manual_control_sub = node->create_subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
      "~/manager/set_manual_control", 10, std::bind(
        &GimbalControlPlugin::manager_set_manual_control_cb, this,
        _1));


    // Publishers
    gimbal_attitude_status_pub = node->create_publisher<mavros_msgs::msg::GimbalDeviceAttitudeStatus>(
      "~/device/attitude_status",
      10);

    gimbal_manager_status_pub = node->create_publisher<mavros_msgs::msg::GimbalManagerStatus>(
      "~/manager/status",
      10);

    gimbal_manager_info_pub = node->create_publisher<mavros_msgs::msg::GimbalManagerInformation>(
      "~/manager/info",
      10);

    // --Not successfully validated--
    gimbal_device_info_pub = node->create_publisher<mavros_msgs::msg::GimbalDeviceInformation>(
      "~/device/info",
      10);


    // Services
    // --Not successfully validated--
    gimbal_device_info_srv = node->create_service<mavros_msgs::srv::GimbalGetInformation>(
      "~/device/get_info", std::bind(
        &GimbalControlPlugin::device_get_info_cb,
        this, _1, _2));

    gimbal_manager_info_srv = node->create_service<mavros_msgs::srv::GimbalGetInformation>(
      "~/manager/get_info", std::bind(
        &GimbalControlPlugin::manager_get_info_cb,
        this, _1, _2));

    gimbal_manager_configure_srv = node->create_service<mavros_msgs::srv::GimbalManagerConfigure>(
      "~/manager/configure", std::bind(
        &GimbalControlPlugin::manager_configure_cb,
        this, _1, _2));

    gimbal_manager_pitchyaw_srv = node->create_service<mavros_msgs::srv::GimbalManagerPitchyaw>(
      "~/manager/pitchyaw", std::bind(
        &GimbalControlPlugin::manager_pitchyaw_cb,
        this, _1, _2));

    gimbal_manager_set_roi_srv = node->create_service<mavros_msgs::srv::GimbalManagerSetRoi>(
      "~/manager/set_roi", std::bind(
        &GimbalControlPlugin::manager_set_roi_cb,
        this, _1, _2));

    // --Not successfully validated--
    gimbal_manager_camera_track = node->create_service<mavros_msgs::srv::GimbalManagerCameraTrack>(
      "~/manager/camera_track", std::bind(
        &GimbalControlPlugin::manager_camera_track,
        this, _1, _2));

  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&GimbalControlPlugin::handle_gimbal_attitude_status),
      make_handler(&GimbalControlPlugin::handle_manager_status),
      make_handler(&GimbalControlPlugin::handle_device_information),
      make_handler(&GimbalControlPlugin::handle_manager_information),
    };
  }

private:
  // Callback Group
  rclcpp::CallbackGroup::SharedPtr cb_group;
  // Subscribers
  rclcpp::Subscription<mavros_msgs::msg::GimbalDeviceSetAttitude>::SharedPtr set_device_attitude_sub;
  rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetAttitude>::SharedPtr
    set_manager_attitude_sub;
  rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr
    set_manager_pitchyaw_sub;
  rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr
    set_manager_manual_control_sub;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::GimbalDeviceAttitudeStatus>::SharedPtr
    gimbal_attitude_status_pub;
  rclcpp::Publisher<mavros_msgs::msg::GimbalManagerStatus>::SharedPtr gimbal_manager_status_pub;
  rclcpp::Publisher<mavros_msgs::msg::GimbalManagerInformation>::SharedPtr gimbal_manager_info_pub;
  rclcpp::Publisher<mavros_msgs::msg::GimbalDeviceInformation>::SharedPtr gimbal_device_info_pub;

  // Services
  rclcpp::Service<mavros_msgs::srv::GimbalGetInformation>::SharedPtr gimbal_device_info_srv;
  rclcpp::Service<mavros_msgs::srv::GimbalGetInformation>::SharedPtr gimbal_manager_info_srv;
  rclcpp::Service<mavros_msgs::srv::GimbalManagerConfigure>::SharedPtr gimbal_manager_configure_srv;
  rclcpp::Service<mavros_msgs::srv::GimbalManagerPitchyaw>::SharedPtr gimbal_manager_pitchyaw_srv;
  rclcpp::Service<mavros_msgs::srv::GimbalManagerSetRoi>::SharedPtr gimbal_manager_set_roi_srv;
  rclcpp::Service<mavros_msgs::srv::GimbalManagerCameraTrack>::SharedPtr gimbal_manager_camera_track;

  // Clients
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_cli;
  std::shared_timed_mutex mu;

  std::string frame_id;       // origin frame for topic headers
  std::string tf_frame_id;    // origin frame for TF
  std::atomic<bool> tf_send;  // parameter for enabling TF publishing

  // Client used by all services for sending mavros/cmd/command service calls, on a separate callback group to support nested service calls
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr get_cmd_cli()
  {
    s_shared_lock lock(mu);

    if (cmd_cli) {
      return cmd_cli;
    }

#ifdef USE_OLD_RMW_QOS
    auto services_qos = rmw_qos_profile_services_default;
#else
    auto services_qos = rclcpp::ServicesQoS();
#endif

    cmd_cli = node->create_client<mavros_msgs::srv::CommandLong>("cmd/command", services_qos,
          cb_group);
    while (!cmd_cli->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node->get_logger(),
            "GimbalControl: mavros/cmd/command service not available after waiting");
      cmd_cli.reset();
      throw std::logic_error("client not connected");
    }

    return cmd_cli;
  }

  // Transform Publisher
  void publish_tf(mavros_msgs::msg::GimbalDeviceAttitudeStatus & gimbal_attitude_msg)
  {
    if (tf_send) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = gimbal_attitude_msg.header.stamp;
      transform.header.frame_id = tf_frame_id;
      // TF child_frame_id with format "gimbal_<component_id>" where the component_id comes from the gimbal_attitude_msg
      transform.child_frame_id = "gimbal_" + std::to_string(gimbal_attitude_msg.target_component);
      transform.transform.rotation = gimbal_attitude_msg.q;
      uas->tf2_broadcaster.sendTransform(transform);
    }
  }

  // Mavlink subscriber callbacks
  /**
   * @brief Publish the gimbal orientation
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS
   * @param msg - the mavlink message
   * @param mo - received GimbalDeviceAttitudeStatus msg
   */
  void handle_gimbal_attitude_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS & mo,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    mavros_msgs::msg::GimbalDeviceAttitudeStatus gimbal_attitude_msg;
    gimbal_attitude_msg.header = uas->synchronized_header(frame_id, mo.time_boot_ms);
    gimbal_attitude_msg.target_system = mo.target_system;
    gimbal_attitude_msg.target_component = mo.target_component;
    gimbal_attitude_msg.flags = mo.flags;
    auto q = mavros::ftf::mavlink_to_quaternion(mo.q);
    gimbal_attitude_msg.q = tf2::toMsg(q);
    gimbal_attitude_msg.angular_velocity_x = mo.angular_velocity_x;
    gimbal_attitude_msg.angular_velocity_y = mo.angular_velocity_y;
    gimbal_attitude_msg.angular_velocity_z = mo.angular_velocity_z;
    gimbal_attitude_msg.failure_flags = mo.failure_flags;

    gimbal_attitude_status_pub->publish(gimbal_attitude_msg);

    // publish tf
    publish_tf(gimbal_attitude_msg);
  }

  /**
   * @brief Publish gimbal manager status
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS
   * @param msg - the mavlink message
   * @param ms - received ManagerStatus msg
   */
  void handle_manager_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GIMBAL_MANAGER_STATUS & ms,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    mavros_msgs::msg::GimbalManagerStatus gimbal_manager_status_msg;
    gimbal_manager_status_msg.header = uas->synchronized_header(frame_id, ms.time_boot_ms);
    gimbal_manager_status_msg.flags = ms.flags;
    gimbal_manager_status_msg.gimbal_device_id = ms.gimbal_device_id;
    gimbal_manager_status_msg.sysid_primary = ms.primary_control_sysid;
    gimbal_manager_status_msg.compid_primary = ms.primary_control_compid;
    gimbal_manager_status_msg.sysid_secondary = ms.secondary_control_sysid;
    gimbal_manager_status_msg.compid_secondary = ms.secondary_control_compid;

    gimbal_manager_status_pub->publish(gimbal_manager_status_msg);
  }

  /**
   * @brief Publish gimbal device information - Note: this message is only published on request by default (see device_get_info_cb)
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION
   * @param msg - the mavlink message
   * @param di - received GimbalDeviceInformation msg
   */
  void handle_device_information(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GIMBAL_DEVICE_INFORMATION & di,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    mavros_msgs::msg::GimbalDeviceInformation gimbal_device_information_msg;
    gimbal_device_information_msg.header = uas->synchronized_header(frame_id, di.time_boot_ms);
    gimbal_device_information_msg.vendor_name = mavlink::to_string(di.vendor_name);
    gimbal_device_information_msg.model_name = mavlink::to_string(di.model_name);
    gimbal_device_information_msg.custom_name = mavlink::to_string(di.custom_name);
    gimbal_device_information_msg.firmware_version = di.firmware_version;
    gimbal_device_information_msg.hardware_version = di.hardware_version;
    gimbal_device_information_msg.uid = di.uid;
    gimbal_device_information_msg.cap_flags = di.cap_flags;
    gimbal_device_information_msg.custom_cap_flags = di.custom_cap_flags;
    gimbal_device_information_msg.roll_min = di.roll_min;
    gimbal_device_information_msg.roll_max = di.roll_max;
    gimbal_device_information_msg.pitch_min = di.pitch_min;
    gimbal_device_information_msg.pitch_max = di.pitch_max;
    gimbal_device_information_msg.yaw_min = di.yaw_min;
    gimbal_device_information_msg.yaw_max = di.yaw_max;

    gimbal_device_info_pub->publish(gimbal_device_information_msg);
  }

  /**
   * @brief Publish gimbal manager information - Note: this message is only published on request by default (see manager_get_info_cb)
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS
   * @param msg - the mavlink message
   * @param mi - received GimbalManagerInformation msg
   */
  void handle_manager_information(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::GIMBAL_MANAGER_INFORMATION & mi,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    mavros_msgs::msg::GimbalManagerInformation gimbal_manager_information_msg;
    gimbal_manager_information_msg.header = uas->synchronized_header(frame_id, mi.time_boot_ms);
    gimbal_manager_information_msg.cap_flags = mi.cap_flags;
    gimbal_manager_information_msg.gimbal_device_id = mi.gimbal_device_id;
    gimbal_manager_information_msg.roll_min = mi.roll_min;
    gimbal_manager_information_msg.roll_max = mi.roll_max;
    gimbal_manager_information_msg.pitch_min = mi.pitch_min;
    gimbal_manager_information_msg.pitch_max = mi.pitch_max;
    gimbal_manager_information_msg.yaw_min = mi.yaw_min;
    gimbal_manager_information_msg.yaw_max = mi.yaw_max;

    gimbal_manager_info_pub->publish(gimbal_manager_information_msg);
  }

  // ROS subscriber callbacks
  /**
   * @brief Send attitude control commands to gimbal device
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE
   * @param req	- received GimbalControl msg
   */
  void device_set_attitude_cb(const mavros_msgs::msg::GimbalDeviceSetAttitude::SharedPtr req)
  {
    mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE msg;
    std::array<float, 4UL> new_q;
    new_q[0] = req->q.w;
    new_q[1] = req->q.x;
    new_q[2] = req->q.y;
    new_q[3] = req->q.z;
    uas->msg_set_target(msg);
    msg.target_system = req->target_system;
    msg.target_component = req->target_component;
    msg.flags = req->flags;
    msg.q = new_q;
    msg.angular_velocity_x = req->angular_velocity_x;
    msg.angular_velocity_y = req->angular_velocity_y;
    msg.angular_velocity_z = req->angular_velocity_z;

    uas->send_message(msg);
  }

  /**
   * @brief Send attitude control commands to gimbal manager
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_ATTITUDE
   * @param req	- received GimbalControl msg
   */
  void manager_set_attitude_cb(const mavros_msgs::msg::GimbalManagerSetAttitude::SharedPtr req)
  {
    mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE msg {};
    std::array<float, 4UL> new_q;
    new_q[0] = req->q.w;
    new_q[1] = req->q.x;
    new_q[2] = req->q.y;
    new_q[3] = req->q.z;
    uas->msg_set_target(msg);
    msg.target_system = req->target_system;
    msg.target_component = req->target_component;
    msg.flags = req->flags;
    msg.gimbal_device_id = req->gimbal_device_id;
    msg.q = new_q;
    msg.angular_velocity_x = req->angular_velocity_x;
    msg.angular_velocity_y = req->angular_velocity_y;
    msg.angular_velocity_z = req->angular_velocity_z;

    uas->send_message(msg);
  }

  /**
   * @brief Send pitchyaw control commands to gimbal manager
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW
   * @param req	- received GimbalControl msg
   */
  void manager_set_pitchyaw_cb(const mavros_msgs::msg::GimbalManagerSetPitchyaw::SharedPtr req)
  {
    mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW msg {};
    uas->msg_set_target(msg);
    msg.target_system = req->target_system;
    msg.target_component = req->target_component;
    msg.flags = req->flags;
    msg.gimbal_device_id = req->gimbal_device_id;
    msg.pitch = req->pitch;
    msg.yaw = req->yaw;
    msg.pitch_rate = req->pitch_rate;
    msg.yaw_rate = req->yaw_rate;

    uas->send_message(msg);
  }

  /**
   * @brief Send manual control commands to gimbal manager
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL
   * Note that message contents is identical to that of pitchyaw, so the message type is re-used here
   * @param req	- received GimbalControl msg
   */
  void manager_set_manual_control_cb(
    const mavros_msgs::msg::GimbalManagerSetPitchyaw::SharedPtr req)
  {
    mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW msg {};
    uas->msg_set_target(msg);
    msg.target_system = req->target_system;
    msg.target_component = req->target_component;
    msg.flags = req->flags;
    msg.gimbal_device_id = req->gimbal_device_id;
    msg.pitch = req->pitch;
    msg.yaw = req->yaw;
    msg.pitch_rate = req->pitch_rate;
    msg.yaw_rate = req->yaw_rate;

    uas->send_message(msg);
  }

  // Service Callbacks
  /**
   * @brief Request GIMBAL_DEVICE_INFORMATION msg be broadcast through Mavlink (see handle_device_information)
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION
   * @param req	- received GimbalGetInformation msg
   */
  void device_get_info_cb(
    const mavros_msgs::srv::GimbalGetInformation::Request::SharedPtr req [[maybe_unused]],
    mavros_msgs::srv::GimbalGetInformation::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    try {
      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      cmdrq->command = enum_value(MAV_CMD::REQUEST_MESSAGE);
      cmdrq->param1 = mavlink::common::msg::GIMBAL_DEVICE_INFORMATION::MSG_ID;
      auto future = get_cmd_cli()->async_send_request(cmdrq);
      auto response = future.get();
      res->success = response->success;
      res->result = response->result;
    } catch (std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "GimbalControl: %s", ex.what());
    }
    RCLCPP_ERROR_EXPRESSION(
      get_logger(), !res->success, "GimbalControl: plugin service call failed!");
  }

  void device_get_info_response(
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture response)
  {
    auto result = response.get();

  }

  /**
   * @brief Request GIMBAL_MANAGER_INFORMATION msg be broadcast through Mavlink (see handle_manager_information)
   *
   * Message specification: https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION
   * @param req	- received GimbalControl msg
   */
  void manager_get_info_cb(
    const mavros_msgs::srv::GimbalGetInformation::Request::SharedPtr req [[maybe_unused]],
    mavros_msgs::srv::GimbalGetInformation::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;
    try {
      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      cmdrq->command = enum_value(MAV_CMD::REQUEST_MESSAGE);
      cmdrq->param1 = mavlink::common::msg::GIMBAL_MANAGER_INFORMATION::MSG_ID;
      auto future = get_cmd_cli()->async_send_request(cmdrq);
      auto response = future.get();
      res->success = response->success;
      res->result = response->result;
    } catch (std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "GimbalControl: %s", ex.what());
    }
    RCLCPP_ERROR_EXPRESSION(
      get_logger(), !res->success, "GimbalControl: plugin service call failed!");
  }

  /**
   * @brief Configure gimbal manager
   *
   * Message specification: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
   * @param req	- received GimbalControl msg
   */
  void manager_configure_cb(
    mavros_msgs::srv::GimbalManagerConfigure::Request::SharedPtr req,
    mavros_msgs::srv::GimbalManagerConfigure::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;

    try {
      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      cmdrq->command = enum_value(MAV_CMD::DO_GIMBAL_MANAGER_CONFIGURE);
      cmdrq->param1 = req->sysid_primary;
      cmdrq->param2 = req->compid_primary;
      cmdrq->param3 = req->sysid_secondary;
      cmdrq->param4 = req->compid_secondary;
      cmdrq->param7 = req->gimbal_device_id;
      auto future = get_cmd_cli()->async_send_request(cmdrq);
      auto response = future.get();
      res->success = response->success;
      res->result = response->result;
    } catch (std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "GimbalControl: %s", ex.what());
    }
    RCLCPP_ERROR_EXPRESSION(
      get_logger(), !res->success, "GimbalControl: plugin service call failed!");
  }

  /**
   * @brief Send pitch/yaw command to gimbal manager
   *
   * Message specification: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
   * @param req	- received GimbalControl msg
   */
  void manager_pitchyaw_cb(
    mavros_msgs::srv::GimbalManagerPitchyaw::Request::SharedPtr req,
    mavros_msgs::srv::GimbalManagerPitchyaw::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;

    try {
      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      cmdrq->command = enum_value(MAV_CMD::DO_GIMBAL_MANAGER_PITCHYAW);
      cmdrq->param1 = req->pitch;
      cmdrq->param2 = req->yaw;
      cmdrq->param3 = req->pitch_rate;
      cmdrq->param4 = req->yaw_rate;
      cmdrq->param5 = req->flags;
      cmdrq->param7 = req->gimbal_device_id;
      auto future = get_cmd_cli()->async_send_request(cmdrq);
      auto response = future.get();
      res->success = response->success;
      res->result = response->result;
    } catch (std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "GimbalControl: %s", ex.what());
    }
    RCLCPP_ERROR_EXPRESSION(
      get_logger(), !res->success, "GimbalControl: plugin service call failed!");
  }

  /**
   * @brief Set Gimbal ROI mode and parameters
   *
   * Message specifications:
   * https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION
   * https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET
   * https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID
   * https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE
   * @param req	- received GimbalControl msg
   */
  void manager_set_roi_cb(
    mavros_msgs::srv::GimbalManagerSetRoi::Request::SharedPtr req,
    mavros_msgs::srv::GimbalManagerSetRoi::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;

    try {
      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      if (req->mode == req->ROI_MODE_LOCATION) {
        cmdrq->command = enum_value(MAV_CMD::DO_SET_ROI_LOCATION);
        cmdrq->param1 = req->gimbal_device_id;
        cmdrq->param5 = req->latitude;
        cmdrq->param6 = req->longitude;
        cmdrq->param7 = req->altitude;
      } else if (req->mode == req->ROI_MODE_WP_NEXT_OFFSET) {
        cmdrq->command = enum_value(MAV_CMD::DO_SET_ROI_WPNEXT_OFFSET);
        cmdrq->param1 = req->gimbal_device_id;
        cmdrq->param5 = req->pitch_offset;
        cmdrq->param6 = req->roll_offset;
        cmdrq->param7 = req->yaw_offset;
      } else if (req->mode == req->ROI_MODE_SYSID) {
        cmdrq->command = enum_value(MAV_CMD::DO_SET_ROI_SYSID);
        cmdrq->param1 = req->sysid;
        cmdrq->param2 = req->gimbal_device_id;
      } else if (req->mode == req->ROI_MODE_NONE) {
        cmdrq->command = enum_value(MAV_CMD::DO_SET_ROI_NONE);
        cmdrq->param1 = req->gimbal_device_id;
      } else {
        res->success = false;
        res->result = 2; // MAV_RESULT_DENIED - Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work.
        return;
      }

      // RCLCPP_DEBUG(get_logger(), "GimbalManagerSetRoi for gimbal id: %u ", req->gimbal_device_id);
      auto future = get_cmd_cli()->async_send_request(cmdrq);
      auto response = future.get();
      res->success = response->success;
      res->result = response->result;
    } catch (std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "GimbalManagerSetRoi: %s", ex.what());
    }

    RCLCPP_ERROR_EXPRESSION(
      get_logger(), !res->success, "GimbalManager - set roi: plugin service call failed!");
  }

  /**
   * @brief Set camera tracking mode and parameters
   *
   * Message specifications:
   * https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT
   * https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE
   * https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING
   * @param req	- received GimbalControl msg
   */
  void manager_camera_track(
    mavros_msgs::srv::GimbalManagerCameraTrack::Request::SharedPtr req,
    mavros_msgs::srv::GimbalManagerCameraTrack::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;

    try {
      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      if (req->mode == req->CAMERA_TRACK_MODE_POINT) {
        cmdrq->command = enum_value(MAV_CMD::CAMERA_TRACK_POINT);
        cmdrq->param1 = req->x;
        cmdrq->param2 = req->y;
        cmdrq->param3 = req->radius;
      } else if (req->mode == req->CAMERA_TRACK_MODE_RECTANGLE) {
        cmdrq->command = enum_value(MAV_CMD::CAMERA_TRACK_RECTANGLE);
        cmdrq->param1 = req->top_left_x;
        cmdrq->param2 = req->top_left_y;
        cmdrq->param3 = req->bottom_right_x;
        cmdrq->param4 = req->bottom_right_y;
      } else if (req->mode == req->CAMERA_TRACK_MODE_STOP_TRACKING) {
        cmdrq->command = enum_value(MAV_CMD::CAMERA_STOP_TRACKING);
      } else {
        res->success = false;
        res->result = 2; // MAV_RESULT_DENIED - Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work.
        return;
      }

      auto future = get_cmd_cli()->async_send_request(cmdrq);
      auto response = future.get();
      res->success = response->success;
      res->result = response->result;
    } catch (std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "GimbalManagerCameraTrack: %s", ex.what());
    }

    RCLCPP_ERROR_EXPRESSION(
      get_logger(), !res->success, "GimbalManager - camera track: plugin service call failed!");
  }
};

}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::GimbalControlPlugin)
