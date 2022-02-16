/*
 * Copyright 2013-2017,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief IMU and attitude data parser plugin
 * @file imu.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <tf2_eigen/tf2_eigen.h>

#include <cmath>
#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

//! Gauss to Tesla coeff
static constexpr double GAUSS_TO_TESLA = 1.0e-4;
//! millTesla to Tesla coeff
static constexpr double MILLIT_TO_TESLA = 1000.0;
//! millRad/Sec to Rad/Sec coeff
static constexpr double MILLIRS_TO_RADSEC = 1.0e-3;
//! millG to m/s**2 coeff
static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
//! millm/s**2 to m/s**2 coeff
static constexpr double MILLIMS2_TO_MS2 = 1.0e-3;
//! millBar to Pascal coeff
static constexpr double MILLIBAR_TO_PASCAL = 1.0e2;
//! Radians to degrees
static constexpr double RAD_TO_DEG = 180.0 / M_PI;


/**
 * @brief IMU and attitude data publication plugin
 * @plugin imu
 */
class IMUPlugin : public plugin::Plugin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit IMUPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "imu"),
    has_hr_imu(false),
    has_raw_imu(false),
    has_scaled_imu(false),
    has_att_quat(false),
    received_linear_accel(false),
    linear_accel_vec_flu(Eigen::Vector3d::Zero()),
    linear_accel_vec_frd(Eigen::Vector3d::Zero())
  {
    enable_node_watch_parameters();

    /**
     * @warning A rotation from the aircraft-frame to the base_link frame is applied.
     * Additionally, it is reported the orientation of the vehicle to describe the
     * transformation from the ENU frame to the base_link frame (ENU <-> base_link).
     * THIS ORIENTATION IS NOT THE SAME AS THAT REPORTED BY THE FCU (NED <-> aircraft).
     */
    node_declare_and_watch_parameter(
      "frame_id", "base_link", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    node_declare_and_watch_parameter(
      "linear_acceleration_stdev", 0.0003, [&](const rclcpp::Parameter & p) {
        auto linear_stdev = p.as_double();
        setup_covariance(linear_acceleration_cov, linear_stdev);
      });
    node_declare_and_watch_parameter(
      "angular_velocity_stdev", 0.02 * (M_PI / 180.0), [&](const rclcpp::Parameter & p) {
        auto angular_stdev = p.as_double();
        setup_covariance(angular_velocity_cov, angular_stdev);
      });
    node_declare_and_watch_parameter(
      "orientation_stdev", 1.0, [&](const rclcpp::Parameter & p) {
        auto orientation_stdev = p.as_double();
        setup_covariance(orientation_cov, orientation_stdev);
      });
    node_declare_and_watch_parameter(
      "magnetic_stdev", 0.0, [&](const rclcpp::Parameter & p) {
        auto mag_stdev = p.as_double();
        setup_covariance(magnetic_cov, mag_stdev);
      });

    setup_covariance(unk_orientation_cov, 0.0);

    auto sensor_qos = rclcpp::SensorDataQoS();

    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("~/data", sensor_qos);
    imu_raw_pub = node->create_publisher<sensor_msgs::msg::Imu>("~/data_raw", sensor_qos);
    magn_pub = node->create_publisher<sensor_msgs::msg::MagneticField>("~/mag", sensor_qos);
    temp_imu_pub = node->create_publisher<sensor_msgs::msg::Temperature>(
      "~/temperature_imu",
      sensor_qos);
    temp_baro_pub = node->create_publisher<sensor_msgs::msg::Temperature>(
      "~/temperature_baro",
      sensor_qos);
    static_press_pub = node->create_publisher<sensor_msgs::msg::FluidPressure>(
      "~/static_pressure",
      sensor_qos);
    diff_press_pub = node->create_publisher<sensor_msgs::msg::FluidPressure>(
      "~/diff_pressure",
      sensor_qos);

    // Reset has_* flags on connection change
    enable_connection_cb();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&IMUPlugin::handle_attitude),
      make_handler(&IMUPlugin::handle_attitude_quaternion),
      make_handler(&IMUPlugin::handle_highres_imu),
      make_handler(&IMUPlugin::handle_raw_imu),
      make_handler(&IMUPlugin::handle_scaled_imu),
      make_handler(&IMUPlugin::handle_scaled_pressure),
    };
  }

private:
  std::string frame_id;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magn_pub;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_baro_pub;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr static_press_pub;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr diff_press_pub;

  std::atomic<bool> has_hr_imu;
  std::atomic<bool> has_raw_imu;
  std::atomic<bool> has_scaled_imu;
  std::atomic<bool> has_att_quat;
  std::atomic<bool> received_linear_accel;
  Eigen::Vector3d linear_accel_vec_flu;
  Eigen::Vector3d linear_accel_vec_frd;
  ftf::Covariance3d linear_acceleration_cov;
  ftf::Covariance3d angular_velocity_cov;
  ftf::Covariance3d orientation_cov;
  ftf::Covariance3d unk_orientation_cov;
  ftf::Covariance3d magnetic_cov;

  /* -*- helpers -*- */

  /**
   * @brief Setup 3x3 covariance matrix
   * @param cov		Covariance matrix
   * @param stdev		Standard deviation
   * @remarks		Diagonal computed from the stdev
   */
  void setup_covariance(ftf::Covariance3d & cov, double stdev)
  {
    ftf::EigenMapCovariance3d c(cov.data());

    c.setZero();
    if (stdev) {
      double sr = stdev * stdev;
      c.diagonal() << sr, sr, sr;
    } else {
      c(0, 0) = -1.0;
    }
  }

  /**
   * @brief Fill and publish IMU data message.
   * @param time_boot_ms     Message timestamp (not syncronized)
   * @param orientation_enu  Orientation in the base_link ENU frame
   * @param orientation_ned  Orientation in the aircraft NED frame
   * @param gyro_flu         Angular velocity/rate in the base_link Forward-Left-Up frame
   * @param gyro_frd         Angular velocity/rate in the aircraft Forward-Right-Down frame
   */
  void publish_imu_data(
    uint32_t time_boot_ms, Eigen::Quaterniond & orientation_enu,
    Eigen::Quaterniond & orientation_ned, Eigen::Vector3d & gyro_flu, Eigen::Vector3d & gyro_frd)
  {
    auto imu_ned_msg = sensor_msgs::msg::Imu();
    auto imu_enu_msg = sensor_msgs::msg::Imu();

    // Fill message header
    imu_enu_msg.header = uas->synchronized_header(frame_id, time_boot_ms);
    imu_ned_msg.header = uas->synchronized_header("aircraft", time_boot_ms);

    // Convert from Eigen::Quaternond to geometry_msgs::Quaternion
    imu_enu_msg.orientation = tf2::toMsg(orientation_enu);
    imu_ned_msg.orientation = tf2::toMsg(orientation_ned);

    // Convert from Eigen::Vector3d to geometry_msgs::Vector3
    tf2::toMsg(gyro_flu, imu_enu_msg.angular_velocity);
    tf2::toMsg(gyro_frd, imu_ned_msg.angular_velocity);

    // Eigen::Vector3d from HIGHRES_IMU or RAW_IMU, to geometry_msgs::Vector3
    tf2::toMsg(linear_accel_vec_flu, imu_enu_msg.linear_acceleration);
    tf2::toMsg(linear_accel_vec_frd, imu_ned_msg.linear_acceleration);

    // Pass ENU msg covariances
    imu_enu_msg.orientation_covariance = orientation_cov;
    imu_enu_msg.angular_velocity_covariance = angular_velocity_cov;
    imu_enu_msg.linear_acceleration_covariance = linear_acceleration_cov;

    // Pass NED msg covariances
    imu_ned_msg.orientation_covariance = orientation_cov;
    imu_ned_msg.angular_velocity_covariance = angular_velocity_cov;
    imu_ned_msg.linear_acceleration_covariance = linear_acceleration_cov;

    if (!received_linear_accel) {
      // Set element 0 of covariance matrix to -1
      // if no data received as per sensor_msgs/Imu defintion
      imu_enu_msg.linear_acceleration_covariance[0] = -1;
      imu_ned_msg.linear_acceleration_covariance[0] = -1;
    }

    /** Store attitude in base_link ENU
     *  @snippet src/plugins/imu.cpp store_enu
     */
    // [store_enu]
    uas->data.update_attitude_imu_enu(imu_enu_msg);
    // [store_enu]

    /** Store attitude in aircraft NED
     *  @snippet src/plugins/imu.cpp store_ned
     */
    // [store_enu]
    uas->data.update_attitude_imu_ned(imu_ned_msg);
    // [store_ned]

    /** Publish only base_link ENU message
     *  @snippet src/plugins/imu.cpp pub_enu
     */
    // [pub_enu]
    imu_pub->publish(imu_enu_msg);
    // [pub_enu]
  }

  /**
   * @brief Fill and publish IMU data_raw message; store linear acceleration for IMU data
   * @param header      Message frame_id and timestamp
   * @param gyro_flu    Orientation in the base_link Forward-Left-Up frame
   * @param accel_flu   Linear acceleration in the base_link Forward-Left-Up frame
   * @param accel_frd   Linear acceleration in the aircraft Forward-Right-Down frame
   */
  void publish_imu_data_raw(
    const std_msgs::msg::Header & header, const Eigen::Vector3d & gyro_flu,
    const Eigen::Vector3d & accel_flu, const Eigen::Vector3d & accel_frd)
  {
    auto imu_msg = sensor_msgs::msg::Imu();

    // Fill message header
    imu_msg.header = header;

    tf2::toMsg(gyro_flu, imu_msg.angular_velocity);
    tf2::toMsg(accel_flu, imu_msg.linear_acceleration);

    // Save readings
    linear_accel_vec_flu = accel_flu;
    linear_accel_vec_frd = accel_frd;
    received_linear_accel = true;

    imu_msg.orientation_covariance = unk_orientation_cov;
    imu_msg.angular_velocity_covariance = angular_velocity_cov;
    imu_msg.linear_acceleration_covariance = linear_acceleration_cov;

    // Publish message [ENU frame]
    imu_raw_pub->publish(imu_msg);
  }

  /**
   * @brief Publish magnetic field data
   * @param header	Message frame_id and timestamp
   * @param mag_field	Magnetic field in the base_link ENU frame
   */
  void publish_mag(const std_msgs::msg::Header & header, const Eigen::Vector3d & mag_field)
  {
    auto magn_msg = sensor_msgs::msg::MagneticField();

    // Fill message header
    magn_msg.header = header;

    tf2::toMsg(mag_field, magn_msg.magnetic_field);
    magn_msg.magnetic_field_covariance = magnetic_cov;

    // Publish message [ENU frame]
    magn_pub->publish(magn_msg);
  }

  /* -*- message handlers -*- */

  /**
   * @brief Handle ATTITUDE MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html#ATTITUDE
   * @param msg	Received Mavlink msg
   * @param att	ATTITUDE msg
   */
  void handle_attitude(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ATTITUDE & att,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (has_att_quat) {
      return;
    }

    /** Orientation on the NED-aicraft frame:
     *  @snippet src/plugins/imu.cpp ned_aircraft_orient1
     */
    // [ned_aircraft_orient1]
    auto ned_aircraft_orientation = ftf::quaternion_from_rpy(att.roll, att.pitch, att.yaw);
    // [ned_aircraft_orient1]

    /** Angular velocity on the NED-aicraft frame:
     *  @snippet src/plugins/imu.cpp ned_ang_vel1
     */
    // [frd_ang_vel1]
    auto gyro_frd = Eigen::Vector3d(att.rollspeed, att.pitchspeed, att.yawspeed);
    // [frd_ang_vel1]

    /** The RPY describes the rotation: aircraft->NED.
     *  It is required to change this to aircraft->base_link:
     *  @snippet src/plugins/imu.cpp ned->baselink->enu
     */
    // [ned->baselink->enu]
    auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
      ftf::transform_orientation_ned_enu(ned_aircraft_orientation));
    // [ned->baselink->enu]

    /** The angular velocity expressed in the aircraft frame.
     *  It is required to apply the static rotation to get it into the base_link frame:
     *  @snippet src/plugins/imu.cpp rotate_gyro
     */
    // [rotate_gyro]
    auto gyro_flu = ftf::transform_frame_aircraft_baselink(gyro_frd);
    // [rotate_gyro]

    publish_imu_data(
      att.time_boot_ms, enu_baselink_orientation, ned_aircraft_orientation, gyro_flu,
      gyro_frd);
  }

  /**
   * @brief Handle ATTITUDE_QUATERNION MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html/#ATTITUDE_QUATERNION
   * @param msg		Received Mavlink msg
   * @param att_q		ATTITUDE_QUATERNION msg
   */
  void handle_attitude_quaternion(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ATTITUDE_QUATERNION & att_q,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), !has_att_quat.exchange(
        true), "IMU: Attitude quaternion IMU detected!");

    /** Orientation on the NED-aicraft frame:
     *  @snippet src/plugins/imu.cpp ned_aircraft_orient2
     */
    // [ned_aircraft_orient2]
    auto ned_aircraft_orientation = Eigen::Quaterniond(att_q.q1, att_q.q2, att_q.q3, att_q.q4);
    // [ned_aircraft_orient2]

    /** Angular velocity on the NED-aicraft frame:
     *  @snippet src/plugins/imu.cpp ned_ang_vel2
     */
    // [frd_ang_vel2]
    auto gyro_frd = Eigen::Vector3d(att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed);
    // [frd_ang_vel2]

    /** MAVLink quaternion exactly matches Eigen convention.
     *  The RPY describes the rotation: aircraft->NED.
     *  It is required to change this to aircraft->base_link:
     *  @snippet src/plugins/imu.cpp ned->baselink->enu
     */
    auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
      ftf::transform_orientation_ned_enu(ned_aircraft_orientation));

    /** The angular velocity expressed in the aircraft frame.
     *  It is required to apply the static rotation to get it into the base_link frame:
     *  @snippet src/plugins/imu.cpp rotate_gyro
     */
    auto gyro_flu = ftf::transform_frame_aircraft_baselink(gyro_frd);

    publish_imu_data(
      att_q.time_boot_ms, enu_baselink_orientation, ned_aircraft_orientation,
      gyro_flu, gyro_frd);
  }

  /**
   * @brief Handle HIGHRES_IMU MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html/#HIGHRES_IMU
   * @param msg		Received Mavlink msg
   * @param imu_hr	HIGHRES_IMU msg
   */
  void handle_highres_imu(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::HIGHRES_IMU & imu_hr, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), !has_hr_imu.exchange(
        true), "IMU: High resolution IMU detected!");

    auto header = uas->synchronized_header(frame_id, imu_hr.time_usec);
    /** @todo Make more paranoic check of HIGHRES_IMU.fields_updated
     */

    /** Check if accelerometer + gyroscope data are available.
     *  Data is expressed in aircraft frame it is required to rotate to the base_link frame:
     *  @snippet src/plugins/imu.cpp accel_available
     */
    // [accel_available]
    if (imu_hr.fields_updated & ((7 << 3) | (7 << 0))) {
      auto gyro_flu =
        ftf::transform_frame_aircraft_baselink(
        Eigen::Vector3d(
          imu_hr.xgyro, imu_hr.ygyro,
          imu_hr.zgyro));

      auto accel_frd = Eigen::Vector3d(imu_hr.xacc, imu_hr.yacc, imu_hr.zacc);
      auto accel_flu = ftf::transform_frame_aircraft_baselink(accel_frd);

      publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);
    }
    // [accel_available]

    /** Check if magnetometer data is available:
     *  @snippet src/plugins/imu.cpp mag_available
     */
    // [mag_available]
    if (imu_hr.fields_updated & (7 << 6)) {
      auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
        Eigen::Vector3d(imu_hr.xmag, imu_hr.ymag, imu_hr.zmag) * GAUSS_TO_TESLA);

      publish_mag(header, mag_field);
    }
    // [mag_available]

    /** Check if static pressure sensor data is available:
     *  @snippet src/plugins/imu.cpp static_pressure_available
     */
    // [static_pressure_available]
    if (imu_hr.fields_updated & (1 << 9)) {
      auto static_pressure_msg = sensor_msgs::msg::FluidPressure();

      static_pressure_msg.header = header;
      static_pressure_msg.fluid_pressure = imu_hr.abs_pressure;

      static_press_pub->publish(static_pressure_msg);
    }
    // [static_pressure_available]

    /** Check if differential pressure sensor data is available:
     *  @snippet src/plugins/imu.cpp differential_pressure_available
     */
    // [differential_pressure_available]
    if (imu_hr.fields_updated & (1 << 10)) {
      auto differential_pressure_msg = sensor_msgs::msg::FluidPressure();

      differential_pressure_msg.header = header;
      differential_pressure_msg.fluid_pressure = imu_hr.diff_pressure;

      diff_press_pub->publish(differential_pressure_msg);
    }
    // [differential_pressure_available]

    /** Check if temperature data is available:
     *  @snippet src/plugins/imu.cpp temperature_available
     */
    // [temperature_available]
    if (imu_hr.fields_updated & (1 << 12)) {
      auto temp_msg = sensor_msgs::msg::Temperature();

      temp_msg.header = header;
      temp_msg.temperature = imu_hr.temperature;

      temp_imu_pub->publish(temp_msg);
    }
    // [temperature_available]
  }

  /**
   * @brief Handle RAW_IMU MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html/#RAW_IMU
   * @param msg		Received Mavlink msg
   * @param imu_raw	RAW_IMU msg
   */
  void handle_raw_imu(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::RAW_IMU & imu_raw, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    RCLCPP_INFO_EXPRESSION(get_logger(), !has_raw_imu.exchange(true), "IMU: Raw IMU message used.");

    if (has_hr_imu || has_scaled_imu) {
      return;
    }

    auto imu_msg = sensor_msgs::msg::Imu();
    auto header = uas->synchronized_header(frame_id, imu_raw.time_usec);

    /** @note APM send SCALED_IMU data as RAW_IMU
     */
    auto gyro_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
      Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
    auto accel_frd = Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc);
    auto accel_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(accel_frd);

    if (uas->is_ardupilotmega()) {
      accel_frd *= MILLIG_TO_MS2;
      accel_flu *= MILLIG_TO_MS2;
    } else if (uas->is_px4()) {
      accel_frd *= MILLIMS2_TO_MS2;
      accel_flu *= MILLIMS2_TO_MS2;
    }

    publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);

    if (!uas->is_ardupilotmega()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 60000, "IMU: linear acceleration on RAW_IMU known on APM only.");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 60000,
        "IMU: ~imu/data_raw stores unscaled raw acceleration report.");
      linear_accel_vec_flu.setZero();
      linear_accel_vec_frd.setZero();
    }

    /** Magnetic field data:
     *  @snippet src/plugins/imu.cpp mag_field
     */
    // [mag_field]
    auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
      Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);
    // [mag_field]

    publish_mag(header, mag_field);
  }

  /**
   * @brief Handle SCALED_IMU MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html/#SCALED_IMU
   * @param msg		Received Mavlink msg
   * @param imu_raw	SCALED_IMU msg
   */
  void handle_scaled_imu(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SCALED_IMU & imu_raw,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (has_hr_imu) {
      return;
    }

    RCLCPP_INFO_EXPRESSION(
      get_logger(), !has_scaled_imu.exchange(
        true), "IMU: Scaled IMU message used.");

    auto header = uas->synchronized_header(frame_id, imu_raw.time_boot_ms);

    auto gyro_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
      Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
    auto accel_frd = Eigen::Vector3d(
      Eigen::Vector3d(
        imu_raw.xacc, imu_raw.yacc,
        imu_raw.zacc) * MILLIG_TO_MS2);
    auto accel_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(accel_frd);

    publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);

    /** Magnetic field data:
     *  @snippet src/plugins/imu.cpp mag_field
     */
    auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
      Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);

    publish_mag(header, mag_field);
  }

  /**
   * @brief Handle SCALED_PRESSURE MAVlink message.
   * Message specification: https://mavlink.io/en/messages/common.html/#SCALED_PRESSURE
   * @param msg		Received Mavlink msg
   * @param press		SCALED_PRESSURE msg
   */
  void handle_scaled_pressure(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SCALED_PRESSURE & press,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (has_hr_imu) {
      return;
    }

    auto header = uas->synchronized_header(frame_id, press.time_boot_ms);

    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header = header;
    temp_msg.temperature = press.temperature / 100.0;
    temp_baro_pub->publish(temp_msg);

    auto static_pressure_msg = sensor_msgs::msg::FluidPressure();
    static_pressure_msg.header = header;
    static_pressure_msg.fluid_pressure = press.press_abs * 100.0;
    static_press_pub->publish(static_pressure_msg);

    auto differential_pressure_msg = sensor_msgs::msg::FluidPressure();
    differential_pressure_msg.header = header;
    differential_pressure_msg.fluid_pressure = press.press_diff * 100.0;
    diff_press_pub->publish(differential_pressure_msg);
  }

  // Checks for connection and overrides variable values
  void connection_cb([[maybe_unused]] bool connected) override
  {
    has_hr_imu = false;
    has_raw_imu = false;
    has_scaled_imu = false;
    has_att_quat = false;
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::IMUPlugin)
