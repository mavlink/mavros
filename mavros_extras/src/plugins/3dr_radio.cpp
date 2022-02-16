/*
 * Copyright 2014,2015,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief 3DR Radio status plugin
 * @file 3dr_radio.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <memory>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/radio_status.hpp"

namespace mavros
{
namespace extra_plugins
{

class TDRFilter : public plugin::filter::Filter
{
public:
  inline bool operator()(
    plugin::filter::UASPtr uas, const mavlink::mavlink_message_t * cmsg,
    const plugin::filter::Framing framing) override
  {
    if (cmsg->sysid != '3' || cmsg->compid != 'D') {
      RCLCPP_WARN_THROTTLE(
        uas->get_logger(),
        *uas->get_clock(), 30, "RADIO_STATUS not from 3DR modem?");
    }

    return framing == plugin::filter::Framing::ok;
  }
};

/**
 * @brief 3DR Radio plugin.
 * @plugin tdr_radio
 */
class TDRRadioPlugin : public plugin::Plugin
{
public:
  explicit TDRRadioPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "tdr_radio"),
    has_radio_status(false),
    diag_added(false),
    low_rssi(0)
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "low_rssi", 40, [&](const rclcpp::Parameter & p) {
        low_rssi = p.as_int();
      });

    auto sensor_qos = rclcpp::SensorDataQoS();

    status_pub = node->create_publisher<mavros_msgs::msg::RadioStatus>("radio_status", sensor_qos);

    enable_connection_cb();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&TDRRadioPlugin::handle_radio_status),
      make_handler(&TDRRadioPlugin::handle_radio),
    };
  }

private:
  bool has_radio_status;
  bool diag_added;
  int low_rssi;

  rclcpp::Publisher<mavros_msgs::msg::RadioStatus>::SharedPtr status_pub;

  std::mutex diag_mutex;
  mavros_msgs::msg::RadioStatus::SharedPtr last_status;

  /* -*- message handlers -*- */

  void handle_radio_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::RADIO_STATUS & rst,
    TDRFilter filter [[maybe_unused]]
  )
  {
    has_radio_status = true;
    handle_message(msg, rst);
  }

  void handle_radio(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::RADIO & rst,
    TDRFilter filter [[maybe_unused]]
  )
  {
    if (has_radio_status) {
      return;
    }

    // actually the same data, but from earlier modems
    handle_message(msg, rst);
  }

  template<typename msgT>
  void handle_message(const mavlink::mavlink_message_t * mmsg [[maybe_unused]], msgT & rst)
  {
    auto msg = std::make_shared<mavros_msgs::msg::RadioStatus>();

    msg->header.stamp = node->now();

#define RST_COPY(field) msg->field = rst.field
    RST_COPY(rssi);
    RST_COPY(remrssi);
    RST_COPY(txbuf);
    RST_COPY(noise);
    RST_COPY(remnoise);
    RST_COPY(rxerrors);
    RST_COPY(fixed);
#undef RST_COPY

    // valid for 3DR modem
    msg->rssi_dbm = (rst.rssi / 1.9) - 127;
    msg->remrssi_dbm = (rst.remrssi / 1.9) - 127;

    // add diag at first event
    if (!diag_added) {
      uas->diagnostic_updater.add("3DR Radio", this, &TDRRadioPlugin::diag_run);
      diag_added = true;
    }

    // store last status for diag
    {
      std::lock_guard<std::mutex> lock(diag_mutex);
      last_status = msg;
    }

    status_pub->publish(*msg);
  }


  void diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(diag_mutex);

    if (!last_status) {
      stat.summary(2, "No data");
      return;
    } else if (last_status->rssi < low_rssi) {
      stat.summary(1, "Low RSSI");
    } else if (last_status->remrssi < low_rssi) {
      stat.summary(1, "Low remote RSSI");
    } else {
      stat.summary(0, "Normal");
    }

    stat.addf("RSSI", "%u", last_status->rssi);
    stat.addf("RSSI (dBm)", "%.1f", last_status->rssi_dbm);
    stat.addf("Remote RSSI", "%u", last_status->remrssi);
    stat.addf("Remote RSSI (dBm)", "%.1f", last_status->remrssi_dbm);
    stat.addf("Tx buffer (%)", "%u", last_status->txbuf);
    stat.addf("Noice level", "%u", last_status->noise);
    stat.addf("Remote noice level", "%u", last_status->remnoise);
    stat.addf("Rx errors", "%u", last_status->rxerrors);
    stat.addf("Fixed", "%u", last_status->fixed);
  }

  void connection_cb(bool connected [[maybe_unused]]) override
  {
    uas->diagnostic_updater.removeByName("3DR Radio");
    diag_added = false;
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TDRRadioPlugin)
