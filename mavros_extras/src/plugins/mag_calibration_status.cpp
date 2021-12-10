/*
 * Copyright 2021 André Ferreira <andre.ferreira@beyond-vision.pt>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MagCalStatus plugin
 * @file MagCalStatus.cpp
 * @author André Ferreira <andre.ferreira@beyond-vision.pt>
 *
 * @example MagCalStatus.cpp
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "std_msgs/msg/u_int8.hpp"
#include "mavros_msgs/msg/magnetometer_reporter.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief MagCalStatus plugin.
 * @plugin mag_calibration_status
 *
 * Example and "how to" for users.
 */
class MagCalStatusPlugin : public plugin::Plugin
{
public:
  explicit MagCalStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "mag_calibration")
  {
    // TODO(vooon): use QoS for "latched" topics
    mcs_pub = node->create_publisher<std_msgs::msg::UInt8>("~/status", 2);
    mcr_pub = node->create_publisher<mavros_msgs::msg::MagnetometerReporter>("~/report", 2);
  }

  Subscriptions get_subscriptions()
  {
    return {
      make_handler(&MagCalStatusPlugin::handle_status),
      make_handler(&MagCalStatusPlugin::handle_report),
    };
  }

private:
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mcs_pub;
  rclcpp::Publisher<mavros_msgs::msg::MagnetometerReporter>::SharedPtr mcr_pub;

  std::array<bool, 8> calibration_show;
  std::array<uint8_t, 8> _rg_compass_cal_progress;

  // Send progress of magnetometer calibration
  void handle_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::MAG_CAL_PROGRESS & mp,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto mcs = std_msgs::msg::UInt8();

    // How many compasses are we calibrating?
    std::bitset<8> compass_calibrating = mp.cal_mask;

    if (compass_calibrating[mp.compass_id]) {
      // Each compass gets a portion of the overall progress
      if (mp.completion_pct < 95) {
        calibration_show[mp.compass_id] = true;
      }
      _rg_compass_cal_progress[mp.compass_id] = mp.completion_pct;
    }

    // Prevent data over 100% after cal_mask reset bit assigned to compass_id
    uint16_t total_percentage = 0;
    for (size_t i = 0; i < 8 && (compass_calibrating >> i).any(); i++) {
      if (compass_calibrating[i]) {
        total_percentage += static_cast<uint8_t>(_rg_compass_cal_progress[i]);
      }
    }

    mcs.data = total_percentage / compass_calibrating.count();

    mcs_pub->publish(mcs);
  }

  // Send report after calibration is done
  void handle_report(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::MAG_CAL_REPORT & mr,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (mr.compass_id >= calibration_show.size()) {
      return;
    }
    if (calibration_show[mr.compass_id]) {
      auto mcr = mavros_msgs::msg::MagnetometerReporter();

      mcr.header.stamp = node->now();
      mcr.header.frame_id = std::to_string(mr.compass_id);
      mcr.report = mr.cal_status;
      mcr.confidence = mr.orientation_confidence;

      mcr_pub->publish(mcr);
      calibration_show[mr.compass_id] = false;
    }
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::MagCalStatusPlugin)
