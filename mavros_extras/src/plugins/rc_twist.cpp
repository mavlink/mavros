/**
 * @brief RC Twsit plugin
 * @file rc_twist.cpp
 * @author Tien Luc Vu <luc001@e.ntu.edu.sg>
 *
 * @addtogroup plugin
 * @{
 */

#include <vector>
#include <cstdint>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief RC Twist plugin
 * @plugin rc_twist
 */
class RCTwistPlugin : public plugin::Plugin
{
public:
  explicit RCTwistPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "rc_twist")
  {
    twist_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "~/send", 10, std::bind(
        &RCTwistPlugin::twist_cb, this,
        _1));

    enable_connection_cb();
  }

  Subscriptions get_subscriptions() override
  {
    return {};
  }

private:
  using lock_guard = std::lock_guard<std::mutex>;
  std::mutex mutex;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;

  void connection_cb([[maybe_unused]] bool connected) override
  {
    lock_guard lock(mutex);
  }

  /* -*- callbacks -*- */

  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr req)
  {
    if (!uas->is_ardupilotmega() && !uas->is_px4()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 30000, "RC override not supported by this FCU!");
    }

    mavlink::common::msg::RC_CHANNELS_OVERRIDE ovr = {};
    uas->msg_set_target(ovr);

    // [[[cog:
    // for i in range(7, 19):
    //     cog.outl(f"ovr.chan{i}_raw = UINT16_MAX;")
    // ]]]

    ovr.chan1_raw = convert_normalized_to_pwm(req->angular.y);  // Pitch
    ovr.chan2_raw = convert_normalized_to_pwm(req->angular.x);  // Roll
    ovr.chan3_raw = convert_normalized_to_pwm(req->linear.z);  // Throttle
    ovr.chan4_raw = convert_normalized_to_pwm(req->angular.z);  // Yaw
    ovr.chan5_raw = convert_normalized_to_pwm(req->linear.x);  // Forward
    ovr.chan6_raw = convert_normalized_to_pwm(req->linear.y);  // Lateral
    ovr.chan7_raw = UINT16_MAX;
    ovr.chan8_raw = UINT16_MAX;
    ovr.chan9_raw = UINT16_MAX;
    ovr.chan10_raw = UINT16_MAX;
    ovr.chan11_raw = UINT16_MAX;
    ovr.chan12_raw = UINT16_MAX;
    ovr.chan13_raw = UINT16_MAX;
    ovr.chan14_raw = UINT16_MAX;
    ovr.chan15_raw = UINT16_MAX;
    ovr.chan16_raw = UINT16_MAX;
    ovr.chan17_raw = UINT16_MAX;
    ovr.chan18_raw = UINT16_MAX;
    // [[[end]]] (checksum: f878252945b6d1ca47c416b4f3aec145)

    uas->send_message(ovr);
  }

  uint16_t convert_normalized_to_pwm(double normalized)
  {
    normalized = std::max(-1.0, std::min(1.0, normalized));
    if (abs(normalized) < 0.01) {
      return UINT16_MAX;
    }
    return static_cast<uint16_t>(normalized * 400 + 1500);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::RCTwistPlugin)
