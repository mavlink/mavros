/**
 * @brief MAVROS UAS Node class
 * @file mavros_uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013,2014,2015,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <fnmatch.h>

#include <rcpputils/asserts.hpp>
#include <mavros/mavros_uas.hpp>
#include <mavros/utils.hpp>

using namespace mavros;
using namespace mavros::uas;
using mavconn::MAVConnInterface;
using mavconn::Framing;
using mavlink::mavlink_message_t;
using plugin::Plugin;
using plugin::PluginFactory;
using utils::enum_value;

UAS::UAS(
  const rclcpp::NodeOptions & options_,
  const std::string & name_,
  const std::string & uas_url_, uint8_t target_system_,
  uint8_t target_component_)
: rclcpp::Node(name_, rclcpp::NodeOptions(options_).use_intra_process_comms(true)),
  uas_url(uas_url_),
  source_system(1),
  source_component(MAV_COMP_ID_ONBOARD_COMPUTER),
  target_system(target_system_),
  target_component(target_component_),
  diagnostic_updater(this, 1.0),
  plugin_factory_loader("mavros", "mavros::plugin::PluginFactory"),
  loaded_plugins{},
  plugin_subscriptions{},
  tf2_listener(tf2_buffer, true),
  type(enum_value(MAV_TYPE::GENERIC)),
  autopilot(enum_value(MAV_AUTOPILOT::GENERIC)),
  base_mode(0),
  connected(false),
  time_offset(0),
  tsync_mode(UAS::timesync_mode::NONE),
  fcu_caps_known(false),
  fcu_capabilities(0)
{
  int tgt_system, tgt_component;
  std::string fcu_protocol = "v2.0";

  // XXX TODO(vooon): should i use LifecycleNode?


  this->declare_parameter("uas_url", uas_url);
  this->declare_parameter("fcu_protocol", fcu_protocol);
  this->declare_parameter("system_id", source_system);
  this->declare_parameter("component_id", source_component);
  this->declare_parameter("target_system_id", target_system);
  this->declare_parameter("target_component_id", target_component)

  this->declare_parameter("plugin_allowlist", plugin_allowlist);
  this->declare_parameter("plugin_denylist", plugin_denylist);


  this->get_parameter("uas_url", uas_url);
  this->get_parameter("fcu_protocol", fcu_protocol);
  this->get_parameter("system_id", source_system);
  this->get_parameter("component_id", source_component);
  this->get_parameter("target_system_id", tgt_system);
  this->get_parameter("target_component_id", tgt_component);
  this->get_parameter("plugin_allowlist", plugin_allowlist);
  this->get_parameter("plugin_denylist", plugin_denylist);

  // setup diag
  diagnostic_updater.setHardwareID(utils::format("uas://%s", uas_url.c_str()));
  diagnostic_updater.add("MAVROS UAS", this, &UAS::diag_run);

  // setup uas link
  if (fcu_protocol == "v1.0") {
    set_protocol_version(mavconn::Protocol::V10);
  } else if (fcu_protocol == "v2.0") {
    set_protocol_version(mavconn::Protocol::V20);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Unknown FCU protocol: \"%s\", should be: \"v1.0\" or \"v2.0\". Used default v1.0.",
      fcu_protocol.c_str());
    set_protocol_version(mavconn::Protocol::V10);
  }

  // setup source and target
  set_tgt(tgt_system, tgt_component);

  add_connection_change_handler(
    std::bind(
      &UAS::log_connect_change, this,
      std::placeholders::_1));

  // prepare plugin lists
  // issue #257 2: assume that all plugins blacklisted
  if (plugin_denylist.empty() and !plugin_allowlist.empty()) {
    plugin_denylist.emplace_back("*");
  }

  for (auto & name : plugin_factory_loader.getDeclaredClasses()) {
    add_plugin(name);
  }


  std::stringstream ss;
  for (auto & s : mavconn::MAVConnInterface::get_known_dialects()) {
    ss << " " << s;
  }

  RCLCPP_INFO(get_logger(), "Built-in SIMD instructions: %s", Eigen::SimdInstructionSetsInUse());
  RCLCPP_INFO(get_logger(), "Built-in MAVLink package version: %s", mavlink::version);
  RCLCPP_INFO(get_logger(), "Known MAVLink dialects:%s", ss.str().c_str());
  RCLCPP_INFO(
    get_logger(), "MAVROS UAS started. MY ID %u.%u, TARGET ID %u.%u",
    source_system, source_component,
    target_system, target_component);
}


void UAS::plugin_route_cb(const mavlink_message_t * mmsg, const Framing framing)
{
  auto it = plugin_subscriptions.find(mmsg->msgid);
  if (it == plugin_subscriptions.end()) {
    return;
  }

  for (auto & info : it->second) {
    std::get<3>(info)(mmsg, framing);
  }
}

static bool pattern_match(std::string & pattern, std::string & pl_name)
{
  int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
  rcpputils::assert_true(cmp != FNM_NOMATCH, "fnmatch(pattern, pl_name, FNM_CASEFOLD) error");

  return cmp == 0;
}

/**
 * @brief Checks that plugin blacklisted
 *
 * Operation algo:
 *
 *  1. if blacklist and whitelist is empty: load all
 *  2. if blacklist is empty and whitelist non empty: assume blacklist is ["*"]
 *  3. if blacklist non empty: usual blacklist behavior
 *  4. if whitelist non empty: override blacklist
 *
 * @note Issue #257.
 */
bool UAS::is_plugin_allowed(
  std::string & pl_name)
{
  for (auto & bl_pattern : plugin_denylist) {
    if (pattern_match(bl_pattern, pl_name)) {
      for (auto & wl_pattern : plugin_allowlist) {
        if (pattern_match(wl_pattern, pl_name)) {
          return false;
        }
      }

      return true;
    }
  }

  return false;
}

inline bool is_mavlink_message_t(const size_t rt)
{
  static const auto h = typeid(mavlink_message_t).hash_code();
  return h == rt;
}

/**
 * @brief Loads plugin (if not blacklisted)
 */
void UAS::add_plugin(std::string & pl_name)
{
  auto lg = get_logger();

  if (is_plugin_allowed(pl_name)) {
    RCLCPP_INFO_STREAM(lg, "Plugin " << pl_name << " ignored");
    return;
  }

  try {
    auto plugin_factory = plugin_factory_loader.createSharedInstance(pl_name);
    auto plugin = plugin_factory->create_plugin_instance(shared_from_this());

    RCLCPP_INFO_STREAM(lg, "Plugin " << pl_name << " created");

    for (auto & info : plugin->get_subscriptions()) {
      auto msgid = std::get<0>(info);
      auto msgname = std::get<1>(info);
      auto type_hash_ = std::get<2>(info);

      std::string log_msgname;

      if (is_mavlink_message_t(type_hash_)) {
        log_msgname = utils::format("MSG-ID (%u) <%zu>", msgid, type_hash_);
      } else {
        log_msgname = utils::format("%s (%u) <%zu>", msgname, msgid, type_hash_);
      }

      RCLCPP_DEBUG_STREAM(lg, "Route " << log_msgname << " to " << pl_name);

      auto it = plugin_subscriptions.find(msgid);
      if (it == plugin_subscriptions.end()) {
        // new entry

        RCLCPP_DEBUG_STREAM(lg, log_msgname << " - new element");
        plugin_subscriptions[msgid] = Plugin::Subscriptions{{info}};
      } else {
        // existing: check handler message type

        bool append_allowed = is_mavlink_message_t(type_hash_);
        if (!append_allowed) {
          append_allowed = true;
          for (auto & e : it->second) {
            auto t2 = std::get<2>(e);
            if (!is_mavlink_message_t(t2) && t2 != type_hash_) {
              RCLCPP_ERROR_STREAM(
                lg,
                log_msgname << " routed to different message type (hash: " << t2 << ")");
              append_allowed = false;
            }
          }
        }

        if (append_allowed) {
          RCLCPP_DEBUG_STREAM(lg, log_msgname << " - emplace");
          it->second.emplace_back(info);
        } else {
          RCLCPP_ERROR_STREAM(
            lg,
            log_msgname << " handler dropped because this ID are used for another message type");
        }
      }
    }

    loaded_plugins.push_back(plugin);

    RCLCPP_INFO_STREAM(lg, "Plugin " << pl_name << " initialized");
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR_STREAM(lg, "Plugin " << pl_name << " load exception: " << ex.what());
  }
}


void UAS::log_connect_change(bool connected)
{
  auto ap = utils::to_string(mav_uas.get_autopilot());

  /* note: sys_status plugin required */
  if (connected) {
    RCLCPP_INFO(get_logger(), "CON: Got HEARTBEAT, connected. FCU: %s", ap.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "CON: Lost connection, HEARTBEAT timed out.");
  }
}


#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(mavros::uas::UAS)
