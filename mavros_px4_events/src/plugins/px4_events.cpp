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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "px4_events/acquisition.hpp"
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
 * Decode PX4 EVENT messages using local or automatically acquired metadata.
 * Automatic acquisition reuses validated cached files before downloading over FTP.
 * Core sys_status raw event logging and publication remain unchanged.
 */
class PX4EventsPlugin : public plugin::Plugin
{
public:
  explicit PX4EventsPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "px4_events"),
    event_decoder(std::make_shared<Decoder>(decoding_error_callback()))
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
    const auto cache_directory = node->declare_parameter<std::string>(
      "event_metadata_cache_dir", "", configuration);
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
    }
    if (mode == "auto" && error.empty()) {
      event_download_delay = download_delay;
      std::shared_ptr<const MetadataCache> cache;
      if (cache_enabled) {
        if (cache_max_bytes <= 0) {
          RCLCPP_ERROR(get_logger(),
                "PX4 metadata cache disabled: event_metadata_cache_max_bytes must be positive; "
                "continuing automatic decoding");
        } else {
          try {
            cache = std::make_shared<MetadataCache>(cache_directory, cache_max_bytes);
          } catch (const std::exception & ex) {
            RCLCPP_WARN(get_logger(),
                  "PX4 metadata cache disabled: %s; continuing automatic decoding", ex.what());
          }
        }
      }
      try {
        auto discovery = Discovery::create(node);
        auto acquisition = Acquisition::create(node, decoding_error_callback(), std::move(cache));
        event_discovery = std::move(discovery);
        event_acquisition = std::move(acquisition);
        enable_connection_cb();
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(),
              "PX4 event metadata setup failed: %s; retaining core raw event logging", ex.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(),
              "PX4 event metadata setup failed; retaining core raw event logging");
      }
    }
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
  using EventClock = std::chrono::steady_clock;
  // start()/cancel() may deliver completion inline; it re-enters this lock.
  std::recursive_mutex event_mutex;
  std::shared_ptr<Decoder> event_decoder;
  std::shared_ptr<Discovery> event_discovery;
  std::shared_ptr<Acquisition> event_acquisition;
  rclcpp::TimerBase::SharedPtr event_start_timer;
  double event_download_delay{30.0};
  EventClock::time_point event_connected_at{};
  uint64_t event_generation{0};
  bool event_connected{false};
  bool event_download_ready{false};
  bool event_acquisition_started{false};

  Decoder::ErrorCallback decoding_error_callback() const
  {
    return [logger = get_logger()](uint32_t id, const char * reason) {
             RCLCPP_WARN_ONCE(logger,
              "PX4 event decoding failed for ID %u: %s; retaining core raw event logging",
          static_cast<unsigned int>(id), reason);
           };
  }

  void event_acquisition_complete(uint64_t generation, Acquisition::Result result)
  {
    std::lock_guard<std::recursive_mutex> lock(event_mutex);
    if (generation != event_generation || !event_connected ||
      !uas->is_connected() || !uas->is_px4())
    {
      return;
    }
    if (event_start_timer) {
      event_start_timer->cancel();
    }
    if (!result.decoder) {
      return;
    }
    event_decoder = std::shared_ptr<Decoder>(std::move(result.decoder));
    RCLCPP_INFO(get_logger(), "PX4 event metadata ready; decoded logging enabled");
  }

  // event_mutex serializes start with disconnect/cancel; no intermediate result queue.
  void maybe_start_event_acquisition_locked(EventClock::time_point now)
  {
    if (!event_connected) {
      return;
    }
    if (!event_download_ready) {
      if (std::chrono::duration<double>(now - event_connected_at).count() >= event_download_delay) {
        event_download_ready = true;
        event_acquisition->allow_downloads();
        if (event_start_timer) {
          event_start_timer->cancel();
        }
      }
    }
    if (event_acquisition_started) {
      return;
    }
    const auto metadata = event_discovery->metadata();
    if (!metadata) {
      return;  // A later advertisement uses the same start gate.
    }
    event_acquisition_started = true;
    const auto generation = event_generation;
    const std::weak_ptr<PX4EventsPlugin> weak =
      std::static_pointer_cast<PX4EventsPlugin>(shared_from_this());
    if (!event_acquisition->start(*metadata,
      [weak, generation](Acquisition::Result result) {
        if (const auto self = weak.lock()) {
          self->event_acquisition_complete(generation, std::move(result));
        }
        }, event_download_ready))
    {
      event_acquisition_started = false;
      // Wait for the previous connection's cleanup before starting this one.
      if (event_start_timer) {
        event_start_timer->reset();
      }
    }
  }

  void event_download_delay_cb(uint64_t generation, EventClock::time_point now)
  {
    std::lock_guard<std::recursive_mutex> lock(event_mutex);
    if (generation != event_generation) {
      return;
    }
    try {
      maybe_start_event_acquisition_locked(now);
    } catch (const std::exception & ex) {
      event_acquisition_started = true;
      if (event_start_timer) {
        event_start_timer->cancel();
      }
      event_acquisition->cancel();
      RCLCPP_ERROR(get_logger(), "PX4 event metadata start failed: %s; retaining raw logging",
            ex.what());
    } catch (...) {
      event_acquisition_started = true;
      if (event_start_timer) {
        event_start_timer->cancel();
      }
      event_acquisition->cancel();
      RCLCPP_ERROR(get_logger(), "PX4 event metadata start failed; retaining raw logging");
    }
  }

  void connection_cb(bool connected) override
  {
    if (!event_discovery) {
      return;  // Disabled/local/invalid configuration has no automatic resources.
    }
    std::lock_guard<std::recursive_mutex> lock(event_mutex);
    ++event_generation;  // Invalidate results before cancellation can deliver a callback.
    event_connected = connected && uas->is_px4();
    event_acquisition_started = false;
    event_download_ready = false;
    event_connected_at = EventClock::now();
    event_decoder.reset();
    try {
      if (event_start_timer) {
        event_start_timer->cancel();
      }
      event_acquisition->cancel();
      event_discovery->connection(
        event_connected, uas->get_tgt_system(), uas->get_tgt_component());
      if (!event_connected) {
        return;
      }
      const auto generation = event_generation;
      const std::weak_ptr<PX4EventsPlugin> weak =
        std::static_pointer_cast<PX4EventsPlugin>(shared_from_this());
      event_start_timer = node->create_wall_timer(std::chrono::milliseconds(250),
          [weak, generation]() {
            if (const auto self = weak.lock()) {
              self->event_download_delay_cb(generation, EventClock::now());
            }
        });
      event_download_delay_cb(generation, EventClock::now());
    } catch (const std::exception & ex) {
      event_acquisition_started = true;
      RCLCPP_ERROR(get_logger(), "PX4 event metadata timer setup failed: %s; retaining raw logging",
        ex.what());
    } catch (...) {
      event_acquisition_started = true;
      RCLCPP_ERROR(get_logger(), "PX4 event metadata timer setup failed; retaining raw logging");
    }
  }

  void handle_component_metadata(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::COMPONENT_METADATA & metadata,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    std::lock_guard<std::recursive_mutex> lock(event_mutex);
    if (event_discovery && event_connected && uas->is_connected() && uas->is_px4()) {
      event_discovery->advertisement(
        msg->sysid, msg->compid, std::string_view(metadata.uri.data(), metadata.uri.size()),
        metadata.file_crc);
      event_download_delay_cb(event_generation, EventClock::now());
    }
  }

  void handle_component_information(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::COMPONENT_INFORMATION & metadata,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    std::lock_guard<std::recursive_mutex> lock(event_mutex);
    if (event_discovery && event_connected && uas->is_connected() && uas->is_px4()) {
      event_discovery->advertisement(
        msg->sysid, msg->compid,
        std::string_view(metadata.general_metadata_uri.data(),
            metadata.general_metadata_uri.size()),
        metadata.general_metadata_file_crc);
      event_download_delay_cb(event_generation, EventClock::now());
    }
  }

  void handle_event(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::EVENT & event,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    std::shared_ptr<Decoder> decoder;
    {
      std::lock_guard<std::recursive_mutex> lock(event_mutex);
      decoder = event_decoder;
    }
    if (!uas->is_px4() || !decoder) {
      return;
    }
    const auto decoded = decoder->decode(event.id, event.log_levels, event.arguments);
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
