/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Optional PX4 event plugin skeleton
 * @file px4_events.cpp
 * @author Sean Hickey <seanhickey.svh@gmail.com>
 */

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"

namespace mavros
{
namespace px4_events
{

/**
 * @brief Optional PX4 event decoding plugin.
 * @plugin px4_events
 *
 * Package skeleton; event handling and metadata acquisition are not implemented yet.
 */
class PX4EventsPlugin : public plugin::Plugin
{
public:
  explicit PX4EventsPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "px4_events")
  {
    RCLCPP_INFO(get_logger(), "PX4 events plugin skeleton loaded");
  }

  Subscriptions get_subscriptions() override
  {
    return {};
  }
};

}  // namespace px4_events
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::px4_events::PX4EventsPlugin)
