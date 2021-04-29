/*
 * Copyright 2017 Thomas Stastny, Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief HomePosition plugin
 * @file home_position.cpp
 * @author Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <tf2_eigen/tf2_eigen.h>

#include <memory>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/msg/home_position.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT

/**
 * @brief home position plugin.
 * @plugin home_position
 *
 * Publishes home position.
 */
class HomePositionPlugin : public plugin::Plugin
{
public:
  explicit HomePositionPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "home_position")
  {
    auto state_qos = rclcpp::QoS(10).transient_local();

    hp_pub = node->create_publisher<mavros_msgs::msg::HomePosition>("~/home", state_qos);
    hp_sub =
      node->create_subscription<mavros_msgs::msg::HomePosition>(
      "~/set", 10,
      std::bind(&HomePositionPlugin::home_position_cb, this, _1));
    update_srv =
      node->create_service<std_srvs::srv::Trigger>(
      "~/req_update",
      std::bind(&HomePositionPlugin::req_update_cb, this, _1, _2));


    poll_timer =
      node->create_wall_timer(REQUEST_POLL_TIME, std::bind(&HomePositionPlugin::timeout_cb, this));
    poll_timer->cancel();

    enable_connection_cb();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&HomePositionPlugin::handle_home_position),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::HomePosition>::SharedPtr hp_pub;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr hp_sub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr update_srv;

  rclcpp::TimerBase::SharedPtr poll_timer;

  const std::chrono::nanoseconds REQUEST_POLL_TIME = 10s;

  bool call_get_home_position(void)
  {
    using mavlink::common::MAV_CMD;

    bool ret = false;

    try {
      auto client = node->create_client<mavros_msgs::srv::CommandLong>("cmd/command");

      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      cmdrq->command = utils::enum_value(MAV_CMD::GET_HOME_POSITION);

      auto future = client->async_send_request(cmdrq);
      auto response = future.get();
      ret = response->success;
    } catch (std::exception & ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "HP: %s" << ex.what());
    }

    return ret;
  }

  void handle_home_position(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::HOME_POSITION & home_position,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    poll_timer->cancel();

    auto hp = mavros_msgs::msg::HomePosition();

    auto pos =
      ftf::transform_frame_ned_enu(
      Eigen::Vector3d(
        home_position.x, home_position.y,
        home_position.z));
    auto q = ftf::transform_orientation_ned_enu(ftf::mavlink_to_quaternion(home_position.q));
    auto hp_approach_enu =
      ftf::transform_frame_ned_enu(
      Eigen::Vector3d(
        home_position.approach_x,
        home_position.approach_y, home_position.approach_z));

    hp.header.stamp = uas->synchronise_stamp(home_position.time_usec);
    hp.geo.latitude = home_position.latitude / 1E7;     // deg
    hp.geo.longitude = home_position.longitude / 1E7;   // deg
    hp.geo.altitude = home_position.altitude / 1E3 +
      uas->data.geoid_to_ellipsoid_height(hp.geo);      // in meters
    hp.orientation = tf2::toMsg(q);
    hp.position = tf2::toMsg(pos);
    tf2::toMsg(hp_approach_enu, hp.approach);

    RCLCPP_DEBUG(
      get_logger(), "HP: Home lat %f, long %f, alt %f", hp.geo.latitude,
      hp.geo.longitude, hp.geo.altitude);

    hp_pub->publish(hp);
  }

  void home_position_cb(const mavros_msgs::msg::HomePosition::SharedPtr req)
  {
    mavlink::common::msg::SET_HOME_POSITION hp {};

    Eigen::Vector3d pos, approach;
    Eigen::Quaterniond q;

    tf2::fromMsg(req->position, pos);
    pos = ftf::transform_frame_enu_ned(pos);

    tf2::fromMsg(req->orientation, q);
    q = ftf::transform_orientation_enu_ned(q);

    tf2::fromMsg(req->approach, approach);
    approach = ftf::transform_frame_enu_ned(approach);

    hp.target_system = uas->get_tgt_system();
    ftf::quaternion_to_mavlink(q, hp.q);

    hp.time_usec = get_time_usec(req->header.stamp);
    hp.altitude = req->geo.altitude * 1e3 + uas->data.ellipsoid_to_geoid_height(req->geo);
    // [[[cog:
    // for f, m in (('latitude', '1e7'), ('longitude', '1e7')):
    //     cog.outl(f"hp.{f} = req->geo.{f} * {m};")
    // for a, b in (('', 'pos'), ('approach_', 'approach')):
    //     for f in "xyz":
    //         cog.outl(f"hp.{a}{f} = {b}.{f}();")
    // ]]]
    hp.latitude = req->geo.latitude * 1e7;
    hp.longitude = req->geo.longitude * 1e7;
    hp.x = pos.x();
    hp.y = pos.y();
    hp.z = pos.z();
    hp.approach_x = approach.x();
    hp.approach_y = approach.y();
    hp.approach_z = approach.z();
    // [[[end]]] (checksum: 0865d5fcc1f7a15e36e177fc49dee70f)

    uas->send_message(hp);
  }

  void req_update_cb(
    const std_srvs::srv::Trigger::Request::SharedPtr req [[maybe_unused]],
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    res->success = call_get_home_position();
  }

  void timeout_cb()
  {
    RCLCPP_INFO(get_logger(), "HP: requesting home position");
    call_get_home_position();
  }

  void connection_cb(bool connected) override
  {
    if (connected) {
      poll_timer->reset();
    } else {
      poll_timer->cancel();
    }
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::HomePositionPlugin)
