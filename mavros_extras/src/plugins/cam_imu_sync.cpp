/*
 * Copyright 2015 Mohammed Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Camera IMU synchronisation plugin
 * @file cam_imu_sync.cpp
 * @author Mohammed Kabir < mhkabir98@gmail.com >
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/cam_imu_stamp.hpp"

namespace mavros
{
namespace extra_plugins
{
/**
 * @brief Camera IMU synchronisation plugin
 * @plugin cam_imu_sync
 *
 * This plugin publishes a timestamp for when a external camera system was
 * triggered by the FCU. Sequence ID from the message and the image sequence from
 * camera can be corellated to get the exact shutter trigger time.
 */
class CamIMUSyncPlugin : public plugin::Plugin
{
public:
  explicit CamIMUSyncPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "cam_imu_sync")
  {
    cam_imu_pub = node->create_publisher<mavros_msgs::msg::CamIMUStamp>("~/cam_imu_stamp", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&CamIMUSyncPlugin::handle_cam_trig)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::CamIMUStamp>::SharedPtr cam_imu_pub;

  void handle_cam_trig(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::CAMERA_TRIGGER & ctrig,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto sync_msg = mavros_msgs::msg::CamIMUStamp();

    sync_msg.frame_stamp = uas->synchronise_stamp(ctrig.time_usec);
    sync_msg.frame_seq_id = ctrig.seq;

    cam_imu_pub->publish(sync_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::CamIMUSyncPlugin)
