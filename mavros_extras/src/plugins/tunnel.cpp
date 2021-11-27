/*
 * Copyright 2021 Morten Fyhn Amundsen <morten.fyhn.amundsen@gmail.com>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Tunnel plugin
 * @file tunnel.cpp
 * @author Morten Fyhn Amundsen <morten.fyhn.amundsen@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>
#include <stdexcept>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/tunnel.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Tunnel plugin
 * @plugin tunnel
 */
class TunnelPlugin : public plugin::Plugin
{
public:
  explicit TunnelPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "tunnel")
  {
    sub_ =
      node->create_subscription<mavros_msgs::msg::Tunnel>(
      "~/in", 10,
      std::bind(&TunnelPlugin::ros_callback, this, _1));
    pub_ = node->create_publisher<mavros_msgs::msg::Tunnel>("~/out", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&TunnelPlugin::mav_callback),
    };
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::Tunnel>::SharedPtr sub_;
  rclcpp::Publisher<mavros_msgs::msg::Tunnel>::SharedPtr pub_;

  void ros_callback(const mavros_msgs::msg::Tunnel::SharedPtr ros_tunnel)
  {
    try {
      const auto mav_tunnel =
        copy_tunnel<mavros_msgs::msg::Tunnel, mavlink::common::msg::TUNNEL>(
        *ros_tunnel);

      uas->send_message(mav_tunnel);
    } catch (std::overflow_error & ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "in error: " << ex.what());
    }
  }

  void mav_callback(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::TUNNEL & mav_tunnel,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    try {
      const auto ros_tunnel =
        copy_tunnel<mavlink::common::msg::TUNNEL, mavros_msgs::msg::Tunnel>(
        mav_tunnel);

      pub_->publish(ros_tunnel);
    } catch (std::overflow_error & ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "out error: " << ex.what());
    }
  }

  template<typename From, typename To>
  static To copy_tunnel(const From & from) noexcept(false)
  {
    static const auto max_payload_length = mavlink::common::msg::TUNNEL().payload.max_size();

    if (from.payload_length > max_payload_length) {
      throw std::overflow_error("too long payload length");
    }

    auto to = To{};

    to.target_system = from.target_system;
    to.target_component = from.target_component;
    to.payload_type = from.payload_type;
    to.payload_length = from.payload_length;
    std::copy(
      from.payload.begin(),
      from.payload.begin() + from.payload_length,
      to.payload.begin());

    return to;
  }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TunnelPlugin)
