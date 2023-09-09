/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Vibration plugin
 * @file vibration.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/vibration.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Vibration plugin
 * @plugin vibration
 *
 * This plugin is intended to publish MAV vibration levels and accelerometer clipping from FCU.
 */
class VibrationPlugin : public plugin::Plugin
{
public:
  explicit VibrationPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "vibration")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "frame_id", "base_link", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    vibration_pub = node->create_publisher<mavros_msgs::msg::Vibration>("~/raw/vibration", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&VibrationPlugin::handle_vibration)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::Vibration>::SharedPtr vibration_pub;

  std::string frame_id;

  void handle_vibration(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::VIBRATION & vibration,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto vibe_msg = mavros_msgs::msg::Vibration();

    vibe_msg.header = uas->synchronized_header(frame_id, vibration.time_usec);

    Eigen::Vector3d vib_enu = {vibration.vibration_x, vibration.vibration_y, vibration.vibration_z};
    tf2::toMsg(ftf::transform_frame_ned_enu(vib_enu), vibe_msg.vibration);

    vibe_msg.clipping[0] = vibration.clipping_0;
    vibe_msg.clipping[1] = vibration.clipping_1;
    vibe_msg.clipping[2] = vibration.clipping_2;

    vibration_pub->publish(vibe_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VibrationPlugin)
