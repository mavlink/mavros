/*
 * Copyright 2021 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Camera plugin
 * @file camera.cpp
 * @author Jaeyoung Lim <jalim@ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/camera_image_captured.hpp"

namespace mavros
{
namespace extra_plugins
{

//! Mavlink enumerations
using mavlink::common::MAV_CMD;
using utils::enum_value;

/**
 * @brief Camera plugin plugin
 * @plugin camera
 *
 * Plugin for interfacing on the mavlink camera protocol
 * @see command_cb()
 */
class CameraPlugin : public plugin::Plugin
{
public:
  explicit CameraPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "camera")
  {
    camera_image_captured_pub = node->create_publisher<mavros_msgs::msg::CameraImageCaptured>(
      "~/image_captured", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&CameraPlugin::handle_camera_image_captured)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::CameraImageCaptured>::SharedPtr camera_image_captured_pub;

  /**
   * @brief Publish camera image capture information
   *
   * Message specification: https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED
   * @param msg   the mavlink message
   * @param mo	received CAMERA_IMAGE_CAPTURED msg
   */
  void handle_camera_image_captured(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::CAMERA_IMAGE_CAPTURED & mo,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto ic = mavros_msgs::msg::CameraImageCaptured();

    ic.header.stamp = uas->synchronise_stamp(mo.time_boot_ms);
    ic.geo.latitude = mo.lat / 1E7;
    ic.geo.longitude = mo.lon / 1E7;    // deg
    ic.geo.altitude = mo.alt / 1E3 + uas->data.geoid_to_ellipsoid_height(&ic.geo);  // in meters
    ic.relative_alt = mo.relative_alt / 1E3;
    ic.orientation = tf2::toMsg(ftf::mavlink_to_quaternion(mo.q));
    ic.file_url = mavlink::to_string(mo.file_url);

    camera_image_captured_pub->publish(ic);
  }
};

}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::CameraPlugin)
