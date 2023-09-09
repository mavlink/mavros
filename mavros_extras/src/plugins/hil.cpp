/*
 * Copyright 2016,2017 Mohamed Abdelkader, Nuno Marques, Pavel Vechersky, Beat Küng.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Hil plugin
 * @file hil.cpp
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Pavel Vechersky <pvechersky@student.ethz.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/hil_controls.hpp"
#include "mavros_msgs/msg/hil_actuator_controls.hpp"
#include "mavros_msgs/msg/hil_state_quaternion.hpp"
#include "mavros_msgs/msg/hil_gps.hpp"
#include "mavros_msgs/msg/hil_sensor.hpp"
#include "mavros_msgs/msg/optical_flow_rad.hpp"
#include "mavros_msgs/msg/rc_in.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

//! Tesla to Gauss coeff
static constexpr double TESLA_TO_GAUSS = 1.0e4;
//! Pascal to millBar coeff
static constexpr double PASCAL_TO_MILLIBAR = 1.0e-2;

/**
 * @brief Hil plugin
 * @plugin hil
 */
class HilPlugin : public plugin::Plugin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit HilPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "hil")
  {
    hil_state_quaternion_sub = node->create_subscription<mavros_msgs::msg::HilStateQuaternion>(
      "~/state", 10, std::bind(&HilPlugin::state_quat_cb, this, _1));
    hil_gps_sub =
      node->create_subscription<mavros_msgs::msg::HilGPS>(
      "~/gps", 10,
      std::bind(&HilPlugin::gps_cb, this, _1));
    hil_sensor_sub = node->create_subscription<mavros_msgs::msg::HilSensor>(
      "~/imu_ned", 10, std::bind(
        &HilPlugin::sensor_cb, this,
        _1));
    hil_flow_sub = node->create_subscription<mavros_msgs::msg::OpticalFlowRad>(
      "~/optical_flow", 10, std::bind(
        &HilPlugin::optical_flow_cb, this,
        _1));
    hil_rcin_sub =
      node->create_subscription<mavros_msgs::msg::RCIn>(
      "~/rc_inputs", 10,
      std::bind(&HilPlugin::rcin_raw_cb, this, _1));

    hil_controls_pub = node->create_publisher<mavros_msgs::msg::HilControls>("~/controls", 10);
    hil_actuator_controls_pub = node->create_publisher<mavros_msgs::msg::HilActuatorControls>(
      "~/actuator_controls", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&HilPlugin::handle_hil_controls),
      make_handler(&HilPlugin::handle_hil_actuator_controls),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::HilControls>::SharedPtr hil_controls_pub;
  rclcpp::Publisher<mavros_msgs::msg::HilActuatorControls>::SharedPtr hil_actuator_controls_pub;

  rclcpp::Subscription<mavros_msgs::msg::HilStateQuaternion>::SharedPtr hil_state_quaternion_sub;
  rclcpp::Subscription<mavros_msgs::msg::HilGPS>::SharedPtr hil_gps_sub;
  rclcpp::Subscription<mavros_msgs::msg::HilSensor>::SharedPtr hil_sensor_sub;
  rclcpp::Subscription<mavros_msgs::msg::OpticalFlowRad>::SharedPtr hil_flow_sub;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr hil_rcin_sub;

  Eigen::Quaterniond enu_orientation;

  /* -*- rx handlers -*- */

  void handle_hil_controls(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::HIL_CONTROLS & hil_controls,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto hil_controls_msg = mavros_msgs::msg::HilControls();

    hil_controls_msg.header.stamp = uas->synchronise_stamp(hil_controls.time_usec);
    // [[[cog:
    // for f in (
    //     'roll_ailerons', 'pitch_elevator', 'yaw_rudder', 'throttle',
    //     'aux1', 'aux2', 'aux3', 'aux4', 'mode', 'nav_mode'):
    //     cog.outl(f"hil_controls_msg.{f} = hil_controls.{f};")
    // ]]]
    hil_controls_msg.roll_ailerons = hil_controls.roll_ailerons;
    hil_controls_msg.pitch_elevator = hil_controls.pitch_elevator;
    hil_controls_msg.yaw_rudder = hil_controls.yaw_rudder;
    hil_controls_msg.throttle = hil_controls.throttle;
    hil_controls_msg.aux1 = hil_controls.aux1;
    hil_controls_msg.aux2 = hil_controls.aux2;
    hil_controls_msg.aux3 = hil_controls.aux3;
    hil_controls_msg.aux4 = hil_controls.aux4;
    hil_controls_msg.mode = hil_controls.mode;
    hil_controls_msg.nav_mode = hil_controls.nav_mode;
    // [[[end]]] (checksum: c213771db088869eb1a7776f03eb1c23)

    hil_controls_pub->publish(hil_controls_msg);
  }

  void handle_hil_actuator_controls(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::HIL_ACTUATOR_CONTROLS & hil_actuator_controls,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto hil_actuator_controls_msg = mavros_msgs::msg::HilActuatorControls();

    hil_actuator_controls_msg.header.stamp = uas->synchronise_stamp(
      hil_actuator_controls.time_usec);
    const auto & arr = hil_actuator_controls.controls;
    std::copy(arr.cbegin(), arr.cend(), hil_actuator_controls_msg.controls.begin());
    hil_actuator_controls_msg.mode = hil_actuator_controls.mode;
    hil_actuator_controls_msg.flags = hil_actuator_controls.flags;

    hil_actuator_controls_pub->publish(hil_actuator_controls_msg);
  }

  /* -*- callbacks / low level send -*- */

  /**
   * @brief Send hil_state_quaternion to FCU.
   * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION
   */
  void state_quat_cb(const mavros_msgs::msg::HilStateQuaternion::SharedPtr req)
  {
    mavlink::common::msg::HIL_STATE_QUATERNION state_quat = {};

    state_quat.time_usec = get_time_usec(req->header.stamp);
    auto q = ftf::transform_orientation_baselink_aircraft(
      ftf::transform_orientation_enu_ned(
        ftf::to_eigen(req->orientation)));
    ftf::quaternion_to_mavlink(q, state_quat.attitude_quaternion);
    state_quat.lat = req->geo.latitude * 1E7;
    state_quat.lon = req->geo.longitude * 1E7;
    // @warning geographic_msgs/GeoPoint.msg uses WGS 84 reference ellipsoid
    // @TODO: Convert altitude to AMSL to be received by the FCU
    // related to issue #529
    state_quat.alt = req->geo.altitude * 1E3;
    state_quat.ind_airspeed = req->ind_airspeed * 1E2;
    state_quat.true_airspeed = req->true_airspeed * 1E2;
    // WRT world frame
    auto ang_vel = ftf::transform_frame_enu_ned(
      ftf::transform_frame_baselink_aircraft(
        ftf::to_eigen(req->angular_velocity)));
    auto lin_vel = ftf::transform_frame_enu_ned<Eigen::Vector3d>(
      ftf::to_eigen(req->linear_velocity)) * 1E2;
    // linear acceleration - WRT world frame
    auto lin_acc = ftf::transform_frame_baselink_aircraft(
      ftf::to_eigen(req->linear_acceleration));

    // [[[cog:
    // for a, b in zip(('rollspeed', 'pitchspeed', 'yawspeed'), "xyz"):
    //     cog.outl(f"state_quat.{a} = ang_vel.{b}();")
    // for f in "xyz":
    //     cog.outl(f"state_quat.v{f} = lin_vel.{f}();")
    // for f in "xyz":
    //     cog.outl(f"state_quat.{f}acc = lin_acc.{f}();")
    // ]]]
    state_quat.rollspeed = ang_vel.x();
    state_quat.pitchspeed = ang_vel.y();
    state_quat.yawspeed = ang_vel.z();
    state_quat.vx = lin_vel.x();
    state_quat.vy = lin_vel.y();
    state_quat.vz = lin_vel.z();
    state_quat.xacc = lin_acc.x();
    state_quat.yacc = lin_acc.y();
    state_quat.zacc = lin_acc.z();
    // [[[end]]] (checksum: 59683585adc102a8c5ec530d99f8664d)

    uas->send_message(state_quat);
  }

  /**
   * @brief Send hil_gps to FCU.
   * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_GPS
   */
  void gps_cb(const mavros_msgs::msg::HilGPS::SharedPtr req)
  {
    mavlink::common::msg::HIL_GPS gps = {};

    gps.time_usec = get_time_usec(req->header.stamp);
    gps.fix_type = req->fix_type;
    gps.lat = req->geo.latitude * 1E7;
    gps.lon = req->geo.longitude * 1E7;
    // @warning geographic_msgs/GeoPoint.msg uses WGS 84 reference ellipsoid
    // @TODO: Convert altitude to AMSL to be received by the FCU
    // related to issue #529
    gps.alt = req->geo.altitude * 1E3;
    // [[[cog:
    // for f in (
    //     'eph', 'epv', 'vel', 'vn', 've', 'vd', 'cog'):
    //     cog.outl(f"gps.{f} = req->{f} * 1E2;")
    // ]]]
    gps.eph = req->eph * 1E2;
    gps.epv = req->epv * 1E2;
    gps.vel = req->vel * 1E2;
    gps.vn = req->vn * 1E2;
    gps.ve = req->ve * 1E2;
    gps.vd = req->vd * 1E2;
    gps.cog = req->cog * 1E2;
    // [[[end]]] (checksum: b71b4e33be4574667105126a43507e82)
    gps.satellites_visible = req->satellites_visible;

    uas->send_message(gps);
  }

  /**
   * @brief Send hil_sensor to FCU.
   * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_SENSOR
   */
  void sensor_cb(const mavros_msgs::msg::HilSensor::SharedPtr req)
  {
    mavlink::common::msg::HIL_SENSOR sensor = {};

    sensor.time_usec = get_time_usec(req->header.stamp);
    // WRT world frame
    auto acc = ftf::transform_frame_baselink_aircraft(
      ftf::to_eigen(req->acc));
    auto gyro = ftf::transform_frame_baselink_aircraft(
      ftf::to_eigen(req->gyro));
    auto mag = ftf::transform_frame_baselink_aircraft<Eigen::Vector3d>(
      ftf::to_eigen(req->mag) * TESLA_TO_GAUSS);

    // [[[cog:
    // for a in ('acc', 'gyro', 'mag'):
    //     for b in "xyz":
    //         cog.outl(f"sensor.{b}{a} = {a}.{b}();")
    // for f in (('abs_pressure', 'PASCAL_TO_MILLIBAR'),
    //           ('diff_pressure', 'PASCAL_TO_MILLIBAR'),
    //           'pressure_alt', 'temperature', 'fields_updated'):
    //           f1 = f if isinstance(f, str) else f[0]
    //           f2 = f if isinstance(f, str) else f'{f[0]} * {f[1]}'
    //           cog.outl(f"sensor.{f1} = req->{f2};")
    // ]]]
    sensor.xacc = acc.x();
    sensor.yacc = acc.y();
    sensor.zacc = acc.z();
    sensor.xgyro = gyro.x();
    sensor.ygyro = gyro.y();
    sensor.zgyro = gyro.z();
    sensor.xmag = mag.x();
    sensor.ymag = mag.y();
    sensor.zmag = mag.z();
    sensor.abs_pressure = req->abs_pressure * PASCAL_TO_MILLIBAR;
    sensor.diff_pressure = req->diff_pressure * PASCAL_TO_MILLIBAR;
    sensor.pressure_alt = req->pressure_alt;
    sensor.temperature = req->temperature;
    sensor.fields_updated = req->fields_updated;
    // [[[end]]] (checksum: e1f6502cf1195ffdf3018f0c4d0c9329)

    uas->send_message(sensor);
  }

  /**
   * @brief Send simulated optical flow to FCU.
   * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW
   */
  void optical_flow_cb(const mavros_msgs::msg::OpticalFlowRad::SharedPtr req)
  {
    mavlink::common::msg::HIL_OPTICAL_FLOW of = {};

    auto int_xy = ftf::transform_frame_aircraft_baselink(
      Eigen::Vector3d(
        req->integrated_x,
        req->integrated_y,
        0.0));
    auto int_gyro = ftf::transform_frame_aircraft_baselink(
      Eigen::Vector3d(
        req->integrated_xgyro,
        req->integrated_ygyro,
        req->integrated_zgyro));

    of.time_usec = get_time_usec(req->header.stamp);
    of.sensor_id = INT8_MAX;    // while we don't find a better way of handling it
    of.integration_time_us = req->integration_time_us;
    // [[[cog:
    // for f in "xy":
    //     cog.outl(f"of.integrated_{f} = int_xy.{f}();")
    // for f in "xyz":
    //     cog.outl(f"of.integrated_{f}gyro = int_gyro.{f}();")
    // for f in ('time_delta_distance_us', 'distance', 'quality'):
    //     cog.outl(f"of.{f} = req->{f};")
    // ]]]
    of.integrated_x = int_xy.x();
    of.integrated_y = int_xy.y();
    of.integrated_xgyro = int_gyro.x();
    of.integrated_ygyro = int_gyro.y();
    of.integrated_zgyro = int_gyro.z();
    of.time_delta_distance_us = req->time_delta_distance_us;
    of.distance = req->distance;
    of.quality = req->quality;
    // [[[end]]] (checksum: 4dc7f3f9b5de60b4d1685bde42c66b26)
    of.temperature = req->temperature * 100.0f;     // in centi-degrees celsius

    uas->send_message(of);
  }

  /**
   * @brief Send simulated received RAW values of the RC channels to the FCU.
   * Message specification: @p https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW
   */
  void rcin_raw_cb(const mavros_msgs::msg::RCIn::SharedPtr req)
  {
    mavlink::common::msg::HIL_RC_INPUTS_RAW rcin {};

    constexpr size_t MAX_CHANCNT = 12;

    std::array<uint16_t, MAX_CHANCNT> channels;
    auto n = std::min(req->channels.size(), channels.size());
    std::copy(req->channels.begin(), req->channels.begin() + n, channels.begin());
    std::fill(channels.begin() + n, channels.end(), UINT16_MAX);

    rcin.time_usec = get_time_usec(req->header.stamp);
    // [[[cog:
    // for i in range(1,13):
    //     cog.outl(f"rcin.chan{i}_raw = channels[{i-1}];")
    // ]]]
    rcin.chan1_raw = channels[0];
    rcin.chan2_raw = channels[1];
    rcin.chan3_raw = channels[2];
    rcin.chan4_raw = channels[3];
    rcin.chan5_raw = channels[4];
    rcin.chan6_raw = channels[5];
    rcin.chan7_raw = channels[6];
    rcin.chan8_raw = channels[7];
    rcin.chan9_raw = channels[8];
    rcin.chan10_raw = channels[9];
    rcin.chan11_raw = channels[10];
    rcin.chan12_raw = channels[11];
    // [[[end]]] (checksum: 342673b0690e47f16c8b89803ab00e68)

    uas->send_message(rcin);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::HilPlugin)
