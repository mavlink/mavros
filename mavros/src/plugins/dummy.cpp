/*
 * Copyright 2013,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Dummy plugin
 * @file dummy.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @example dummy.cpp
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

namespace mavros
{
namespace std_plugins
{

/**
 * @brief Dummy plugin.
 * @plugin dummy
 * @example_plugin
 *
 * Example and "how to" for users.
 */
class DummyPlugin : public plugin::Plugin
{
public:
  /**
   * Plugin constructor.
   * It must first call initialization of base class.
   *
   * When second argument is passed, it's used as a sub-node name.
   * `node` class member will be initialized with it.
   *
   * You can use `node->create_xyz()` functions to make any node interfaces.
   */
  explicit DummyPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "dummy")
  {
  }

  /**
   * This function returns message subscriptions.
   *
   * Each subscription made by PluginBase::make_handler() template.
   * Two variations:
   *  - With automatic decoding and framing error filtering (see handle_heartbeat)
   *  - Raw message with framig status (see handle_systemtext)
   */
  Subscriptions get_subscriptions() override
  {
    return {
      /* automatic message deduction by second argument */
      make_handler(&DummyPlugin::handle_heartbeat),
      make_handler(&DummyPlugin::handle_sys_status),
      /* handle raw message, check framing! */
      make_handler(mavlink::common::msg::STATUSTEXT::MSG_ID, &DummyPlugin::handle_statustext_raw),
      make_handler(&DummyPlugin::handle_statustext),
    };
  }

private:
  /**
   * This function will be called to handle HEARTBEAT from any source
   * when framing is ok.
   *
   * @param[in] msg     raw message frame
   * @param[in] hb      decoded message (require a type from mavlink c++11 library)
   * @param[in] filter  an instance of that filter will determine conditions to call that function
   */
  void handle_heartbeat(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::minimal::msg::HEARTBEAT & hb,
    plugin::filter::AnyOk filter [[maybe_unused]])
  {
    RCLCPP_INFO_STREAM(get_logger(), "Dummy::handle_heartbeat: " << hb.to_yaml());
  }

  /**
   * This function subscribes for SYS_STATUS message.
   * It will be only called when message came from our target system and framing is ok.
   *
   * @p mavros::plugin::filter::SystemAndOk
   */
  void handle_sys_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SYS_STATUS & st,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    RCLCPP_INFO_STREAM(get_logger(), "Dummy::handle_sys_status: " << st.to_yaml());
  }

  /**
   * This function subscribes for STATUSTEXT message.
   * It will be only called when message came from out target system & component and framing is ok.
   *
   * @p mavros::plugin::filter::ComponentAndOk
   */
  void handle_statustext(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::STATUSTEXT & st,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    RCLCPP_INFO_STREAM(get_logger(), "Dummy::handle_statustext: " << st.to_yaml());
  }

  /**
   * This function will be called for any message with same ID passed to make_handler(),
   * and with any framing status (not only ok).
   */
  void handle_statustext_raw(const mavlink::mavlink_message_t * msg, const mavconn::Framing f)
  {
    RCLCPP_INFO(
      get_logger(),
      "Dummy::handle_statustext_raw(%p, %d) from %u.%u", static_cast<const void *>(msg),
      utils::enum_value(f), msg->sysid, msg->compid);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::DummyPlugin)
