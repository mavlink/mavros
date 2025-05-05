/*
 * Copyright 2024 Gus Meyer.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Open Drone ID plugin to satisfy remote ID requirements
 * @file open_drone_id.cpp
 * @author Gus Meyer <gus@robotics88.com>
 *
 * @addtogroup plugin
 * @{
 */


#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/open_drone_id_basic_id.hpp"
#include "mavros_msgs/msg/open_drone_id_operator_id.hpp"
#include "mavros_msgs/msg/open_drone_id_self_id.hpp"
#include "mavros_msgs/msg/open_drone_id_system.hpp"
#include "mavros_msgs/msg/open_drone_id_system_update.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Open Drone ID plugin
 * @plugin open_drone_id
 *
 * Sends Open Drone ID data to the FCU
 */
class OpenDroneIDPlugin : public plugin::Plugin
{
public:
  explicit OpenDroneIDPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "open_drone_id")
  {
    basic_id_sub = node->create_subscription<mavros_msgs::msg::OpenDroneIDBasicID>(
                        "~/basic_id", 1, std::bind(
                        &OpenDroneIDPlugin::basic_id_cb, this,
                        _1));

    operator_id_sub = node->create_subscription<mavros_msgs::msg::OpenDroneIDOperatorID>(
                        "~/operator_id", 1, std::bind(
                        &OpenDroneIDPlugin::operator_id_cb, this,
                        _1));

    self_id_sub = node->create_subscription<mavros_msgs::msg::OpenDroneIDSelfID>(
                        "~/self_id", 1, std::bind(
                        &OpenDroneIDPlugin::self_id_cb, this,
                        _1));

    system_sub = node->create_subscription<mavros_msgs::msg::OpenDroneIDSystem>(
                        "~/system", 1, std::bind(
                        &OpenDroneIDPlugin::system_cb, this,
                        _1));

    system_update_sub = node->create_subscription<mavros_msgs::msg::OpenDroneIDSystemUpdate>(
                        "~/system_update", 1, std::bind(
                        &OpenDroneIDPlugin::system_update_cb, this,
                        _1));
  }

  Subscriptions get_subscriptions()
  {
    return {};
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::OpenDroneIDBasicID>::SharedPtr basic_id_sub;
  rclcpp::Subscription<mavros_msgs::msg::OpenDroneIDOperatorID>::SharedPtr operator_id_sub;
  rclcpp::Subscription<mavros_msgs::msg::OpenDroneIDSelfID>::SharedPtr self_id_sub;
  rclcpp::Subscription<mavros_msgs::msg::OpenDroneIDSystem>::SharedPtr system_sub;
  rclcpp::Subscription<mavros_msgs::msg::OpenDroneIDSystemUpdate>::SharedPtr system_update_sub;


  void basic_id_cb(const mavros_msgs::msg::OpenDroneIDBasicID::SharedPtr msg)
  {
    mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID basic_id{};

    uas->msg_set_target(basic_id);
    set_string_z(basic_id.id_or_mac, msg->id_or_mac);
    basic_id.id_type = msg->id_type;
    basic_id.ua_type = msg->ua_type;
    set_string_z(basic_id.uas_id, msg->uas_id);

    uas->send_message(basic_id);
  }

  void operator_id_cb(const mavros_msgs::msg::OpenDroneIDOperatorID::SharedPtr msg)
  {
    mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID operator_id{};

    uas->msg_set_target(operator_id);
    set_string_z(operator_id.id_or_mac, msg->id_or_mac);
    operator_id.operator_id_type = msg->operator_id_type;
    mavlink::set_string_z(operator_id.operator_id, msg->operator_id);

    uas->send_message(operator_id);
  }

  void self_id_cb(const mavros_msgs::msg::OpenDroneIDSelfID::SharedPtr msg)
  {
    mavlink::common::msg::OPEN_DRONE_ID_SELF_ID self_id{};

    uas->msg_set_target(self_id);
    set_string_z(self_id.id_or_mac, msg->id_or_mac);
    self_id.description_type = msg->description_type;
    mavlink::set_string_z(self_id.description, msg->description);

    uas->send_message(self_id);
  }

  void system_cb(const mavros_msgs::msg::OpenDroneIDSystem::SharedPtr msg)
  {
    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM system{};

    uas->msg_set_target(system);
    set_string_z(system.id_or_mac, msg->id_or_mac);
    system.operator_location_type = msg->operator_location_type;
    system.classification_type = msg->classification_type;
    system.operator_latitude = msg->operator_latitude;
    system.operator_longitude = msg->operator_longitude;
    system.area_count = msg->area_count;
    system.area_radius = msg->area_radius;
    system.area_ceiling = msg->area_ceiling;
    system.area_floor = msg->area_floor;
    system.category_eu = msg->category_eu;
    system.class_eu = msg->class_eu;
    system.operator_altitude_geo = msg->operator_altitude_geo;
    system.timestamp = to_timestamp(msg->header);

    uas->send_message(system);
  }

  void system_update_cb(const mavros_msgs::msg::OpenDroneIDSystemUpdate::SharedPtr msg)
  {
    mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE system_update{};

    uas->msg_set_target(system_update);
    system_update.operator_latitude = msg->operator_latitude;
    system_update.operator_longitude = msg->operator_longitude;
    system_update.operator_altitude_geo = msg->operator_altitude_geo;
    system_update.timestamp = to_timestamp(msg->header);

    uas->send_message(system_update);
  }

  //! ODID timestamp is a 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
  uint32_t to_timestamp(std_msgs::msg::Header & hdr)
  {
    auto s = hdr.stamp.sec;

    // 2019.01.01 00:00:00 UTC
    const int32_t epoch = 1546300800;

    if (s > epoch) {
      return s - epoch;
    }

    return 0;
  }

  /** A variant of mavlink::set_string_z, but for uint8_t,
   * because of broken convention on ODID messages
   */
  template<size_t _N>
  void set_string_z(std::array<uint8_t, _N> & a, const std::string & s)
  {
    uint8_t * datap = a.data();
    strncpy(static_cast<char *>(static_cast<void *>(datap)), s.c_str(), a.size() - 1);
    a[a.size() - 1] = '\0';
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::OpenDroneIDPlugin)
