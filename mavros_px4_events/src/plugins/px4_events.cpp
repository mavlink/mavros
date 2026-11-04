/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Optional PX4 event decoding plugin
 * @file px4_events.cpp
 * @author Sean Hickey <seanhickey.svh@gmail.com>
 */

#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <string_view>

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "px4_events/cache.hpp"
#include "px4_events/decoder.hpp"
#include "px4_events/discovery.hpp"

namespace mavros
{
namespace px4_events
{

/**
 * @brief Optional PX4 event decoding plugin.
 * @plugin px4_events
 *
 * Decode PX4 EVENT messages using local firmware-matching metadata.
 * Core sys_status raw event logging and publication remain unchanged.
 * Discover automatic metadata locations after PX4 connection.
 * Automatic file acquisition is not wired yet.
 */
class PX4EventsPlugin : public plugin::Plugin
{
public:
  explicit PX4EventsPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "px4_events"),
    event_decoder(std::make_shared<Decoder>(
        [logger = get_logger()](uint32_t id, const char * reason) {
          RCLCPP_WARN_ONCE(logger,
                "PX4 event decoding failed for ID %u: %s; retaining core raw event logging",
            static_cast<unsigned int>(id), reason);
        }))
  {
    rcl_interfaces::msg::ParameterDescriptor configuration;
    configuration.read_only = true;
    configuration.description = "PX4 event decoding configuration; changes require restart.";

    //! PX4 event logging: auto (default), local, or disabled; changes require restart.
    const auto mode = node->declare_parameter<std::string>(
      "event_decoding_mode", "auto", configuration);
    //! Firmware-matching local JSON or JSON.xz definitions; valid only in local mode.
    const auto file = node->declare_parameter<std::string>(
      "event_metadata_file", "", configuration);
    //! Automatic metadata download delay from FCU connection, in seconds.
    const auto download_delay = node->declare_parameter<double>(
      "event_metadata_download_delay", 30.0, configuration);
    //! Reuse validated persistent definitions in automatic mode.
    const auto cache_enabled = node->declare_parameter<bool>(
      "event_metadata_cache_enabled", true, configuration);
    //! Empty selects $HOME/.cache/mavros/px4_events; otherwise use this directory.
    node->declare_parameter<std::string>("event_metadata_cache_dir", "", configuration);
    //! Best-effort finalized cache size limit; nonpositive values disable persistence.
    const auto cache_max_bytes = node->declare_parameter<int64_t>(
      "event_metadata_cache_max_bytes", MetadataCache::DEFAULT_MAX_BYTES, configuration);

    auto error = event_decoder->configure(mode, file);
    if (error.empty() && mode == "auto" &&
      (!std::isfinite(download_delay) || download_delay < 0))
    {
      error = "event_metadata_download_delay must be finite and nonnegative";
    }
    if (!error.empty()) {
      event_decoder.reset();
      RCLCPP_ERROR(get_logger(),
            "PX4 event decoding inactive: %s; retaining core raw event logging", error.c_str());
    } else if (mode == "auto" && cache_enabled && cache_max_bytes <= 0) {
      RCLCPP_ERROR(get_logger(),
            "PX4 metadata cache disabled: event_metadata_cache_max_bytes must be positive; "
            "continuing automatic decoding");
    }
    if (mode == "auto" && error.empty()) {
      try {
        event_discovery = Discovery::create(node);
        enable_connection_cb();
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(),
              "PX4 event metadata setup failed: %s; retaining core raw event logging", ex.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(),
              "PX4 event metadata setup failed; retaining core raw event logging");
      }
    }
    // Auto mode has no definitions until acquisition wiring is added.
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&PX4EventsPlugin::handle_event),
      make_handler(&PX4EventsPlugin::handle_component_metadata),
      make_handler(&PX4EventsPlugin::handle_component_information),
    };
  }

private:
  friend class PX4EventsPluginTestPeer;
  std::shared_ptr<Decoder> event_decoder;
  std::shared_ptr<Discovery> event_discovery;

  void connection_cb(bool connected) override
  {
    if (!event_discovery) {
      return;
    }
    try {
      event_discovery->connection(
        connected && uas->is_px4(), uas->get_tgt_system(), uas->get_tgt_component());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(),
            "PX4 event metadata connection update failed: %s; retaining core raw event logging",
        ex.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(),
            "PX4 event metadata connection update failed; retaining core raw event logging");
    }
  }

  void handle_component_metadata(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::COMPONENT_METADATA & metadata,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    if (event_discovery && uas->is_connected() && uas->is_px4()) {
      event_discovery->advertisement(
        msg->sysid, msg->compid, std::string_view(metadata.uri.data(), metadata.uri.size()),
        metadata.file_crc);
    }
  }

  void handle_component_information(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::COMPONENT_INFORMATION & metadata,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    if (event_discovery && uas->is_connected() && uas->is_px4()) {
      event_discovery->advertisement(
        msg->sysid, msg->compid,
        std::string_view(metadata.general_metadata_uri.data(), metadata.general_metadata_uri.size()),
        metadata.general_metadata_file_crc);
    }
  }

  void handle_event(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::EVENT & event,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (!uas->is_px4() || !event_decoder) {
      return;
    }
    const auto decoded = event_decoder->decode(event.id, event.log_levels, event.arguments);
    if (!decoded) {
      return;  // sys_status owns the existing raw log and status_event publication.
    }
    if (decoded->severity <= 3) {
      RCLCPP_ERROR(get_logger(), "FCU: %s", decoded->text.c_str());
    } else if (decoded->severity <= 5) {
      RCLCPP_WARN(get_logger(), "FCU: %s", decoded->text.c_str());
    } else if (decoded->severity == 6) {
      RCLCPP_INFO(get_logger(), "FCU: %s", decoded->text.c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "FCU: %s", decoded->text.c_str());
    }
  }
};

}  // namespace px4_events
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::px4_events::PX4EventsPlugin)
