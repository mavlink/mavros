/*
 * Copyright 2013,2014,2015,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS UAS Node class
 * @file mavros_uas.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <fnmatch.h>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Eigen>  // NOLINT

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/utils.hpp"

using namespace mavros;                 // NOLINT
using namespace mavros::uas;            // NOLINT
using namespace std::chrono_literals;   // NOLINT
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
: rclcpp::Node(name_, options_ /* rclcpp::NodeOptions(options_).use_intra_process_comms(true) */),
  diagnostic_updater(this, 1.0),
  data(),
  tf2_buffer(get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME)),
  tf2_listener(tf2_buffer, true),
  tf2_broadcaster(this),
  tf2_static_broadcaster(this),
  source_system(target_system_),
  source_component(MAV_COMP_ID_ONBOARD_COMPUTER),
  target_system(target_system_),
  target_component(target_component_),
  uas_url(uas_url_),
  plugin_factory_loader("mavros", "mavros::plugin::PluginFactory"),
  loaded_plugins{},
  plugin_subscriptions{},
  type(enum_value(MAV_TYPE::GENERIC)),
  autopilot(enum_value(MAV_AUTOPILOT::GENERIC)),
  base_mode(0),
  fcu_caps_known(false),
  fcu_capabilities(0),
  connected(false),
  time_offset(0),
  tsync_mode(timesync_mode::NONE),
  mavlink_status({})
{
  // XXX TODO(vooon): should i use LifecycleNode?

  set_parameters_handle_ptr =
    this->add_on_set_parameters_callback(
    std::bind(
      &UAS::on_set_parameters_cb, this,
      std::placeholders::_1));

  this->declare_parameter("uas_url", uas_url);
  this->declare_parameter("fcu_protocol", "v2.0");
  this->declare_parameter("system_id", source_system);
  this->declare_parameter("component_id", source_component);
  this->declare_parameter("target_system_id", target_system);
  this->declare_parameter("target_component_id", target_component);
  this->declare_parameter("plugin_allowlist", plugin_allowlist);
  this->declare_parameter("plugin_denylist", plugin_denylist);

  // NOTE(vooon): we couldn't add_plugin() in constructor because it needs shared_from_this()
  startup_delay_timer = this->create_wall_timer(
    10ms, [this]() {
      startup_delay_timer->cancel();

      std::string fcu_protocol;
      int tgt_system, tgt_component;
      this->get_parameter("uas_url", uas_url);
      this->get_parameter("fcu_protocol", fcu_protocol);
      this->get_parameter("system_id", source_system);
      this->get_parameter("component_id", source_component);
      this->get_parameter("target_system_id", tgt_system);
      this->get_parameter("target_component_id", tgt_component);
      this->get_parameter("plugin_allowlist", plugin_allowlist);
      this->get_parameter("plugin_denylist", plugin_denylist);

      exec_spin_thd = thread_ptr(
        new std::thread(
          [this]() {
            utils::set_this_thread_name("uas-exec/%d.%d", source_system, source_component);
            auto lg = this->get_logger();

            RCLCPP_INFO(
              lg, "UAS Executor started, threads: %zu",
              this->exec.get_number_of_threads());
            this->exec.spin();
            RCLCPP_WARN(lg, "UAS Executor terminated");
          }),
        [this](std::thread * t) {
          this->exec.cancel();
          t->join();
          delete t;
        });

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
          "Unknown FCU protocol: \"%s\", should be: \"v1.0\" or \"v2.0\". Used default v2.0.",
          fcu_protocol.c_str());
        set_protocol_version(mavconn::Protocol::V20);
      }

      // setup source and target
      set_tgt(tgt_system, tgt_component);

      add_connection_change_handler(
        std::bind(
          &UAS::log_connect_change, this,
          std::placeholders::_1));

      // prepare plugin lists
      // issue #257 2: assume that all plugins blacklisted
      if (plugin_denylist.empty() && !plugin_allowlist.empty()) {
        plugin_denylist.emplace_back("*");
      }

      for (auto & name : plugin_factory_loader.getDeclaredClasses()) {
        add_plugin(name);
      }

      connect_to_router();

      // Publish helper TFs used for frame transformation in the odometry plugin
      {
        std::vector<geometry_msgs::msg::TransformStamped> transform_vector;
        add_static_transform(
          "map", "map_ned", Eigen::Affine3d(
            ftf::quaternion_from_rpy(
              M_PI, 0,
              M_PI_2)),
          transform_vector);
        add_static_transform(
          "odom", "odom_ned", Eigen::Affine3d(
            ftf::quaternion_from_rpy(
              M_PI, 0,
              M_PI_2)),
          transform_vector);
        add_static_transform(
          "base_link", "base_link_frd",
          Eigen::Affine3d(ftf::quaternion_from_rpy(M_PI, 0, 0)), transform_vector);

        tf2_static_broadcaster.sendTransform(transform_vector);
      }

      std::stringstream ss;
      for (auto & s : mavconn::MAVConnInterface::get_known_dialects()) {
        ss << " " << s;
      }

      RCLCPP_INFO(
        get_logger(), "Built-in SIMD instructions: %s",
        Eigen::SimdInstructionSetsInUse());
      RCLCPP_INFO(get_logger(), "Built-in MAVLink package version: %s", mavlink::version);
      RCLCPP_INFO(get_logger(), "Known MAVLink dialects:%s", ss.str().c_str());
      RCLCPP_INFO(
        get_logger(), "MAVROS UAS via %s started. MY ID %u.%u, TARGET ID %u.%u",
        uas_url.c_str(),
        source_system, source_component,
        target_system, target_component);
    });
}

void UAS::plugin_route(const mavlink_message_t * mmsg, const Framing framing)
{
  auto it = plugin_subscriptions.find(mmsg->msgid);
  if (it == plugin_subscriptions.end()) {
    return;
  }

  for (auto & info : it->second) {
    std::get<3>(info)(mmsg, framing);
  }
}

static bool pattern_match(const std::string & pattern, const std::string & pl_name)
{
  int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
  rcpputils::require_true(
    cmp == 0 || cmp == FNM_NOMATCH,
    "fnmatch(pattern, pl_name, FNM_CASEFOLD) error");

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
  const std::string & pl_name)
{
  for (auto & bl_pattern : plugin_denylist) {
    if (pattern_match(bl_pattern, pl_name)) {
      for (auto & wl_pattern : plugin_allowlist) {
        if (pattern_match(wl_pattern, pl_name)) {
          return true;
        }
      }

      return false;
    }
  }

  return true;
}

inline bool is_mavlink_message_t(const size_t rt)
{
  static const auto h = typeid(mavlink_message_t).hash_code();
  return h == rt;
}

plugin::Plugin::SharedPtr UAS::create_plugin_instance(const std::string & pl_name)
{
  auto plugin_factory = plugin_factory_loader.createSharedInstance(pl_name);

  return
    plugin_factory->create_plugin_instance(std::static_pointer_cast<UAS>(shared_from_this()));
}

void UAS::add_plugin(const std::string & pl_name)
{
  auto lg = get_logger();

  if (!is_plugin_allowed(pl_name)) {
    RCLCPP_INFO_STREAM(lg, "Plugin " << pl_name << " ignored");
    return;
  }

  try {
    auto plugin = create_plugin_instance(pl_name);

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

    auto pl_node = plugin->get_node();
    if (pl_node && pl_node.get() != this) {
      RCLCPP_DEBUG_STREAM(lg, "Plugin " << pl_name << " added to executor");
      exec.add_node(pl_node);
    }

    RCLCPP_INFO_STREAM(lg, "Plugin " << pl_name << " initialized");
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR_STREAM(lg, "Plugin " << pl_name << " load exception: " << ex.what());
  }
}

rcl_interfaces::msg::SetParametersResult UAS::on_set_parameters_cb(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto lg = get_logger();
  rcl_interfaces::msg::SetParametersResult result{};

  RCLCPP_DEBUG(lg, "params callback");

  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto name = parameter.get_name();
    if (true) {
      // XXX TODO
    } else {
      result.successful = false;
      result.reason = "unknown parameter";
    }
  }

  return result;
}

void UAS::connect_to_router()
{
  auto qos = rclcpp::QoS(
    1000).best_effort().durability_volatile();

  this->sink =
    this->create_publisher<mavros_msgs::msg::Mavlink>(
    utils::format(
      "%s/%s", this->uas_url.c_str(),
      "mavlink_sink"), qos);

  this->source = this->create_subscription<mavros_msgs::msg::Mavlink>(
    utils::format("%s/%s", this->uas_url.c_str(), "mavlink_source"), qos,
    std::bind(&UAS::recv_message, this, std::placeholders::_1));
}

void UAS::recv_message(const mavros_msgs::msg::Mavlink::SharedPtr rmsg)
{
  mavlink::mavlink_message_t msg;

  auto ok = mavros_msgs::mavlink::convert(*rmsg, msg);
  rcpputils::assert_true(ok, "conversion error");

  if (ok) {
    plugin_route(&msg, static_cast<mavconn::Framing>(rmsg->framing_status));
  }
}

void UAS::send_message(const mavlink::Message & obj, const uint8_t src_compid)
{
  mavlink::mavlink_message_t msg;
  mavlink::MsgMap map(msg);

  auto mi = obj.get_message_info();

  obj.serialize(map);
  mavlink::mavlink_finalize_message_buffer(
    &msg, source_system, src_compid, &mavlink_status, mi.min_length, mi.length,
    mi.crc_extra);

  mavros_msgs::msg::Mavlink rmsg{};
  auto ok = mavros_msgs::mavlink::convert(msg, rmsg);

  rmsg.header.stamp = this->now();
  rmsg.header.frame_id = this->get_name();

  if (this->sink && ok) {
    this->sink->publish(rmsg);
  }
}

void UAS::set_protocol_version(mavconn::Protocol pver)
{
  if (pver == mavconn::Protocol::V10) {
    mavlink_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  } else {
    mavlink_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
  }
}

void UAS::log_connect_change(bool connected)
{
  auto ap = utils::to_string(get_autopilot());

  /* note: sys_status plugin required */
  if (connected) {
    RCLCPP_INFO(get_logger(), "CON: Got HEARTBEAT, connected. FCU: %s", ap.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "CON: Lost connection, HEARTBEAT timed out.");
  }
}

void UAS::diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // TODO(vooon): add some fields

  if (connected) {
    stat.summary(0, "connected");
  } else {
    stat.summary(2, "disconnected");
  }
}


#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(mavros::uas::UAS)
