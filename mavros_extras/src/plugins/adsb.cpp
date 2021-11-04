/*
 * Copyright 2017 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Automatic dependent surveillance-broadcast Vehicle plugin
 * @file adsb.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "mavros_msgs/msg/adsb_vehicle.hpp"

namespace mavros
{
namespace extra_plugins
{
using mavlink::common::ADSB_EMITTER_TYPE;
using mavlink::common::ADSB_ALTITUDE_TYPE;

/**
 * @brief ADS-B Vehicle plugin
 * @plugin adsb
 *
 * Publish/subscribe Automatic dependent surveillance-broadcast data to/from a vehicle.
 */
class ADSBPlugin : public plugin::Plugin
{
public:
  explicit ADSBPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "adsb")
  {
    adsb_pub = node->create_publisher<mavros_msgs::msg::ADSBVehicle>("~/vehicle", 10);
    adsb_sub =
      node->create_subscription<mavros_msgs::msg::ADSBVehicle>(
      "~/send", 10,
      std::bind(&ADSBPlugin::adsb_cb, this, std::placeholders::_1));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ADSBPlugin::handle_adsb)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::ADSBVehicle>::SharedPtr adsb_pub;
  rclcpp::Subscription<mavros_msgs::msg::ADSBVehicle>::SharedPtr adsb_sub;

  void handle_adsb(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ADSB_VEHICLE & adsb,
    plugin::filter::SystemAndOk filter [[maybe_unused]]
  )
  {
    auto adsb_msg = mavros_msgs::msg::ADSBVehicle();

    // TODO(vooon): request add time_boot_ms to msg definition
    adsb_msg.header.stamp = node->now();

    // [[[cog:
    // def ent(ros, mav=None, scale=None, to_ros=None, to_mav=None):
    //     return (ros, mav or ros, scale, to_ros, to_mav)
    //
    // TR_TAB = (
    // ent('icao_address', 'ICAO_address'),
    // ent('callsign', to_ros='mavlink::to_string({mm}.{mav})',
    //   to_mav='mavlink::set_string_z({mm}.{mav}, {rmp}->{ros})'),
    // ent('latitude', 'lat', '1e7'),
    // ent('longitude', 'lon', '1e7'),
    // ent('altitude', 'altitude', '1e3'),
    // ent('altitude_type', ),
    // ent('heading', scale='1e2'),
    // ent('hor_velocity', scale='1e2'),
    // ent('ver_velocity', scale='1e2'),
    // ent('altitude_type'),
    // ent('emitter_type'),
    // ent('tslc', to_ros='rclcpp::Duration({mm}.{mav}, 0)',
    //   to_mav='{mm}.{mav} = {rmp}->{ros}.sec'),
    // ent('flags'),
    // ent('squawk'),
    // )
    //
    // for ros, mav, scale, to_ros, _ in TR_TAB:
    //     if to_ros is None:
    //         scale_ex = '' if scale is None else ' / ' + scale
    //         cog.outl(f"""adsb_msg.{ros} = adsb.{mav}{scale_ex};""")
    //     else:
    //         cog.outl(f"""adsb_msg.{ros} = {to_ros.format(mm='adsb', **locals())};""")
    // ]]]
    adsb_msg.icao_address = adsb.ICAO_address;
    adsb_msg.callsign = mavlink::to_string(adsb.callsign);
    adsb_msg.latitude = adsb.lat / 1e7;
    adsb_msg.longitude = adsb.lon / 1e7;
    adsb_msg.altitude = adsb.altitude / 1e3;
    adsb_msg.altitude_type = adsb.altitude_type;
    adsb_msg.heading = adsb.heading / 1e2;
    adsb_msg.hor_velocity = adsb.hor_velocity / 1e2;
    adsb_msg.ver_velocity = adsb.ver_velocity / 1e2;
    adsb_msg.altitude_type = adsb.altitude_type;
    adsb_msg.emitter_type = adsb.emitter_type;
    adsb_msg.tslc = rclcpp::Duration(adsb.tslc, 0);
    adsb_msg.flags = adsb.flags;
    adsb_msg.squawk = adsb.squawk;
    // [[[end]]] (checksum: ae8f818682cc2c23db50504f5af97127)

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "ADSB: recv type: " << utils::to_string_enum<ADSB_ALTITUDE_TYPE>(adsb.altitude_type) <<
        " emitter: " << utils::to_string_enum<ADSB_EMITTER_TYPE>(adsb.emitter_type) <<
        " flags: 0x" << std::hex << adsb.flags);

    adsb_pub->publish(adsb_msg);
  }

  void adsb_cb(const mavros_msgs::msg::ADSBVehicle::SharedPtr req)
  {
    mavlink::common::msg::ADSB_VEHICLE adsb{};

    // [[[cog:
    // for ros, mav, scale, _, to_mav in TR_TAB:
    //     if to_mav is None:
    //         scale_ex = '' if scale is None else ' * ' + scale
    //         cog.outl(f"""adsb.{mav} = req->{ros}{scale_ex};""")
    //     else:
    //         cog.outl(to_mav.format(mm='adsb', rmp='req', **locals()) + ';')
    // ]]]
    adsb.ICAO_address = req->icao_address;
    mavlink::set_string_z(adsb.callsign, req->callsign);
    adsb.lat = req->latitude * 1e7;
    adsb.lon = req->longitude * 1e7;
    adsb.altitude = req->altitude * 1e3;
    adsb.altitude_type = req->altitude_type;
    adsb.heading = req->heading * 1e2;
    adsb.hor_velocity = req->hor_velocity * 1e2;
    adsb.ver_velocity = req->ver_velocity * 1e2;
    adsb.altitude_type = req->altitude_type;
    adsb.emitter_type = req->emitter_type;
    adsb.tslc = req->tslc.sec;
    adsb.flags = req->flags;
    adsb.squawk = req->squawk;
    // [[[end]]] (checksum: e586b680a3d86ec594e5b7f4a59bbe6c)

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "ADSB: send type: " << utils::to_string_enum<ADSB_ALTITUDE_TYPE>(adsb.altitude_type) <<
        " emitter: " << utils::to_string_enum<ADSB_EMITTER_TYPE>(adsb.emitter_type) <<
        " flags: 0x" << std::hex << adsb.flags);

    uas->send_message(adsb);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::ADSBPlugin)
