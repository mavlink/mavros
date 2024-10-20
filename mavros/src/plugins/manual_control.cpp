/*
 * Copyright 2015 Matias Nitsche.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief ManualControls plugin
 * @file manual_controls.cpp
 * @author Matias Nitsche <mnitsche@dc.uba.ar>
 *
 * @addtogroup plugin
 * @{
 */

#include <rcpputils/asserts.hpp>
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <mavros_msgs/msg/manual_control.hpp>

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Manual Control plugin
 * @plugin manual_control
 */
class ManualControlPlugin : public plugin::Plugin
{
public:
  explicit ManualControlPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "manual_control")
  {
    control_pub = node->create_publisher<mavros_msgs::msg::ManualControl>("~/control", 10);
    send_sub =
      node->create_subscription<mavros_msgs::msg::ManualControl>(
      "~/send", 10,
      std::bind(&ManualControlPlugin::send_cb, this, _1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ManualControlPlugin::handle_manual_control),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr control_pub;
  rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr send_sub;

  /* -*- rx handlers -*- */

  void handle_manual_control(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::MANUAL_CONTROL & manual_control,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto manual_control_msg = mavros_msgs::msg::ManualControl();

    manual_control_msg.header.stamp = node->now();
    manual_control_msg.x = (manual_control.x / 1000.0);
    manual_control_msg.y = (manual_control.y / 1000.0);
    manual_control_msg.z = (manual_control.z / 1000.0);
    manual_control_msg.r = (manual_control.r / 1000.0);
    manual_control_msg.buttons = manual_control.buttons;

    manual_control_msg.buttons2 = manual_control.buttons2;
    manual_control_msg.enabled_extensions = manual_control.enabled_extensions;
    manual_control_msg.s = (manual_control.s / 1000.0);
    manual_control_msg.t = (manual_control.t / 1000.0);
    manual_control_msg.aux1 = (manual_control.aux1 / 1000.0);
    manual_control_msg.aux2 = (manual_control.aux2 / 1000.0);
    manual_control_msg.aux3 = (manual_control.aux3 / 1000.0);
    manual_control_msg.aux4 = (manual_control.aux4 / 1000.0);
    manual_control_msg.aux5 = (manual_control.aux5 / 1000.0);
    manual_control_msg.aux6 = (manual_control.aux6 / 1000.0);

    control_pub->publish(manual_control_msg);
  }

  /* -*- callbacks -*- */

  void send_cb(const mavros_msgs::msg::ManualControl::SharedPtr req)
  {
    mavlink::common::msg::MANUAL_CONTROL msg = {};
    msg.target = uas->get_tgt_system();
    msg.x = req->x;
    msg.y = req->y;
    msg.z = req->z;
    msg.r = req->r;
    msg.buttons = req->buttons;

    msg.buttons2 = req->buttons2;
    msg.enabled_extensions = req->enabled_extensions;
    msg.s = req->s;
    msg.t = req->t;
    msg.aux1 = req->aux1;
    msg.aux2 = req->aux2;
    msg.aux3 = req->aux3;
    msg.aux4 = req->aux4;
    msg.aux5 = req->aux5;
    msg.aux6 = req->aux6;

    uas->send_message(msg);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ManualControlPlugin)
