/*
 * Copyright 2014,2015,2016,2017,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS UAS Node
 * @file mavros_uas.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup nodelib
 * @{
 */

#pragma once

#ifndef MAVROS__MAVROS_UAS_HPP_
#define MAVROS__MAVROS_UAS_HPP_

#include <tf2_ros/buffer.h>                         // NOLINT
#include <tf2_ros/transform_listener.h>             // NOLINT
#include <tf2_ros/transform_broadcaster.h>          // NOLINT
#include <tf2_ros/static_transform_broadcaster.h>   // NOLINT

#include <array>
#include <atomic>
#include <memory>
#include <type_traits>
#include <string>
#include <vector>
#include <unordered_map>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "mavconn/interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "GeographicLib/Geoid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "mavros/utils.hpp"
#include "mavros/plugin.hpp"
#include "mavros/frame_tf.hpp"
#include "mavros/uas_executor.hpp"

namespace mavros
{
namespace uas
{

//! default source component
static constexpr auto MAV_COMP_ID_ONBOARD_COMPUTER = 191;

using s_unique_lock = std::unique_lock<std::shared_timed_mutex>;
using s_shared_lock = std::shared_lock<std::shared_timed_mutex>;

// common enums used by UAS
using MAV_TYPE = mavlink::minimal::MAV_TYPE;
using MAV_AUTOPILOT = mavlink::minimal::MAV_AUTOPILOT;
using MAV_MODE_FLAG = mavlink::minimal::MAV_MODE_FLAG;
using MAV_STATE = mavlink::minimal::MAV_STATE;
using MAV_CAP = mavlink::common::MAV_PROTOCOL_CAPABILITY;
using timesync_mode = utils::timesync_mode;


/**
 * @brief UAS Node data
 *
 * This class stores some useful data;
 *
 * Currently it stores:
 * - IMU data (@a mavplugin::IMUPubPlugin)
 * - GPS data (@a mavplugin::GPSPlugin)
 */
class Data
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Data();
  ~Data() = default;

  /* -*- IMU data -*- */

  /**
   * @brief Store IMU data [ENU]
   */
  void update_attitude_imu_enu(const sensor_msgs::msg::Imu & imu);

  /**
   * @brief Store IMU data [NED]
   */
  void update_attitude_imu_ned(const sensor_msgs::msg::Imu & imu);

  /**
   * @brief Get IMU data [ENU]
   */
  sensor_msgs::msg::Imu get_attitude_imu_enu();

  /**
   * @brief Get IMU data [NED]
   */
  sensor_msgs::msg::Imu get_attitude_imu_ned();

  /**
   * @brief Get Attitude orientation quaternion
   * @return orientation quaternion [ENU]
   */
  geometry_msgs::msg::Quaternion get_attitude_orientation_enu();

  /**
   * @brief Get Attitude orientation quaternion
   * @return orientation quaternion [NED]
   */
  geometry_msgs::msg::Quaternion get_attitude_orientation_ned();

  /**
   * @brief Get angular velocity from IMU data
   * @return vector3 [ENU]
   */
  geometry_msgs::msg::Vector3 get_attitude_angular_velocity_enu();

  /**
   * @brief Get angular velocity from IMU data
   * @return vector3 [NED]
   */
  geometry_msgs::msg::Vector3 get_attitude_angular_velocity_ned();


  /* -*- GPS data -*- */

  //! Store GPS RAW data
  void update_gps_fix_epts(
    const sensor_msgs::msg::NavSatFix & fix,
    float eph, float epv,
    int fix_type, int satellites_visible);

  //! Returns EPH, EPV, Fix type and satellites visible
  void get_gps_epts(float & eph, float & epv, int & fix_type, int & satellites_visible);

  //! Retunrs last GPS RAW message
  sensor_msgs::msg::NavSatFix get_gps_fix();

  /* -*- GograpticLib utils -*- */

  /**
   * @brief Geoid dataset used to convert between AMSL and WGS-84
   *
   * That class loads egm96_5 dataset to RAM, it is about 24 MiB.
   */
  static std::shared_ptr<GeographicLib::Geoid> egm96_5;

  /**
   * @brief Conversion from height above geoid (AMSL)
   * to height above ellipsoid (WGS-84)
   */
  template<class T, std::enable_if_t<std::is_pointer<T>::value, bool> = true>
  inline double geoid_to_ellipsoid_height(const T lla)
  {
    if (egm96_5) {
      return GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(lla->latitude, lla->longitude);
    } else {
      return 0.0;
    }
  }

  template<class T, std::enable_if_t<std::is_class<T>::value, bool> = true>
  inline double geoid_to_ellipsoid_height(const T & lla)
  {
    return geoid_to_ellipsoid_height(&lla);
  }

  /**
   * @brief Conversion from height above ellipsoid (WGS-84)
   * to height above geoid (AMSL)
   */
  template<class T, std::enable_if_t<std::is_pointer<T>::value, bool> = true>
  inline double ellipsoid_to_geoid_height(const T lla)
  {
    if (egm96_5) {
      return GeographicLib::Geoid::ELLIPSOIDTOGEOID * (*egm96_5)(lla->latitude, lla->longitude);
    } else {
      return 0.0;
    }
  }

  template<class T, std::enable_if_t<std::is_class<T>::value, bool> = true>
  inline double ellipsoid_to_geoid_height(const T & lla)
  {
    return ellipsoid_to_geoid_height(&lla);
  }

private:
  std::shared_timed_mutex mu;

  sensor_msgs::msg::Imu imu_enu_data;
  sensor_msgs::msg::Imu imu_ned_data;

  sensor_msgs::msg::NavSatFix gps_fix;
  float gps_eph;
  float gps_epv;
  int gps_fix_type;
  int gps_satellites_visible;

  //! init_geographiclib() once flag
  static std::once_flag init_flag;

  //! Initialize egm96-5
  static void init_geographiclib();
};

/**
 * @brief UAS Node
 *
 * This class implements main translation mode.
 * It loads plugins to support various sub-protocols.
 *
 * Also each plugin can use that node to create sub-nodes or other RCLCPP objects.
 *
 * UAS Node provides:
 * - FCU messaging link
 * - FCU System & Component ID pair
 * - Connection status (@a mavplugin::SystemStatusPlugin)
 * - Autopilot type (@a mavplugin::SystemStatusPlugin)
 * - Vehicle type (@a mavplugin::SystemStatusPlugin)
 * - Additional data trough mavros::uas::Data class
 */
class UAS : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(UAS)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // other UAS aliases
  using ConnectionCb = std::function<void (bool)>;
  using CapabilitiesCb = std::function<void (MAV_CAP)>;

  using StrV = std::vector<std::string>;

  explicit UAS(const std::string & name_ = "mavros")
  : UAS(rclcpp::NodeOptions(), name_) {}

  explicit UAS(
    const rclcpp::NodeOptions & options_ = rclcpp::NodeOptions(),
    const std::string & name_ = "mavros",
    const std::string & uas_url_ = "/uas1", uint8_t target_system_ = 1,
    uint8_t target_component_ = 1);

  ~UAS() = default;

  /**
   * @brief Mavros diagnostic updater
   */
  diagnostic_updater::Updater diagnostic_updater;

  //! Data that can be useful to pass between plugins
  Data data;

  /**
   * @brief Return connection status
   */
  inline bool is_connected()
  {
    return connected;
  }

  /* -*- HEARTBEAT data -*- */

  /**
   * Update autopilot type on every HEARTBEAT
   */
  void update_heartbeat(uint8_t type_, uint8_t autopilot_, uint8_t base_mode_);

  /**
   * Update autopilot connection status (every HEARTBEAT/conn_timeout)
   */
  void update_connection_status(bool conn_);

  /**
   * @brief Add connection change handler callback
   */
  void add_connection_change_handler(ConnectionCb cb);

  /**
   * @brief Returns vehicle type
   */
  inline MAV_TYPE get_type()
  {
    std::underlying_type<MAV_TYPE>::type type_ = type;
    return static_cast<MAV_TYPE>(type_);
  }

  /**
   * @brief Returns autopilot type
   */
  inline MAV_AUTOPILOT get_autopilot()
  {
    std::underlying_type<MAV_AUTOPILOT>::type autopilot_ = autopilot;
    return static_cast<MAV_AUTOPILOT>(autopilot_);
  }

  /**
   * @brief Returns arming status
   *
   * @note There may be race condition between SET_MODE and HEARTBEAT.
   */
  inline bool get_armed()
  {
    uint8_t base_mode_ = base_mode;
    return base_mode_ & utils::enum_value(MAV_MODE_FLAG::SAFETY_ARMED);
  }

  /**
   * @brief Returns HIL status
   */
  inline bool get_hil_state()
  {
    uint8_t base_mode_ = base_mode;
    return base_mode_ & utils::enum_value(MAV_MODE_FLAG::HIL_ENABLED);
  }

  /* -*- FCU target id pair -*- */

  /**
   * @brief Return communication target system
   */
  inline uint8_t get_tgt_system()
  {
    return target_system;
  }

  /**
   * @brief Return communication target component
   */
  inline uint8_t get_tgt_component()
  {
    return target_component;
  }

  inline void set_tgt(uint8_t sys, uint8_t comp)
  {
    target_system = sys;
    target_component = comp;
  }

  /* -*- transform -*- */

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  tf2_ros::TransformBroadcaster tf2_broadcaster;
  tf2_ros::StaticTransformBroadcaster tf2_static_broadcaster;

  /**
   * @brief Add static transform. To publish all static transforms at once, we stack them in a std::vector.
   *
   * @param frame_id    parent frame for transform
   * @param child_id    child frame for transform
   * @param tr          transform
   * @param vector      vector of transforms
   */
  void add_static_transform(
    const std::string & frame_id, const std::string & child_id,
    const Eigen::Affine3d & tr,
    std::vector<geometry_msgs::msg::TransformStamped> & vector);

  /**
   * @brief Publishes static transform.
   *
   * @param frame_id    parent frame for transform
   * @param child_id    child frame for transform
   * @param tr          transform
   */
  void publish_static_transform(
    const std::string & frame_id, const std::string & child_id,
    const Eigen::Affine3d & tr);

  /* -*- time sync -*- */

  inline void set_time_offset(uint64_t offset_ns)
  {
    time_offset = offset_ns;
  }

  inline uint64_t get_time_offset(void)
  {
    return time_offset;
  }

  inline void set_timesync_mode(timesync_mode mode)
  {
    tsync_mode = mode;
  }

  inline timesync_mode get_timesync_mode(void)
  {
    return tsync_mode;
  }

  /**
   * @brief Compute FCU message time from time_boot_ms or time_usec field
   *
   * Uses time_offset for calculation
   *
   * @return FCU time if it is known else current wall time.
   */
  rclcpp::Time synchronise_stamp(uint32_t time_boot_ms);
  rclcpp::Time synchronise_stamp(uint64_t time_usec);

  /**
   * @brief Create message header from time_boot_ms or time_usec stamps and frame_id.
   *
   * Setting frame_id and stamp are pretty common, this little helper should reduce LOC.
   *
   * @param[in] frame_id    frame for header
   * @param[in] time_stamp  mavlink message time
   * @return Header with syncronized stamp and frame id
   */
  template<typename T>
  inline std_msgs::msg::Header synchronized_header(const std::string & frame_id, const T time_stamp)
  {
    std_msgs::msg::Header out;
    out.frame_id = frame_id;
    out.stamp = synchronise_stamp(time_stamp);
    return out;
  }

  /* -*- autopilot version -*- */

  uint64_t get_capabilities();

  /**
   * @brief Function to check if the flight controller has a capability
   *
   * @param capabilities can accept a multiple capability params either in enum or int from
   */
  template<typename T>
  bool has_capability(T capability)
  {
    static_assert(
      std::is_enum<T>::value,
      "Only query capabilities using the UAS::MAV_CAP enum.");
    return get_capabilities() & utils::enum_value(capability);
  }

  /**
   * @brief Function to check if the flight controller has a set of capabilities
   *
   * @param capabilities can accept a multiple capability params either in enum or int from
   */
  template<typename ... Ts>
  bool has_capabilities(Ts ... capabilities)
  {
    bool ret = true;
    std::initializer_list<bool> capabilities_list {has_capability<Ts>(capabilities) ...};
    for (auto has_cap : capabilities_list) {
      ret &= has_cap;
    }
    return ret;
  }

  /**
   * @brief Update the capabilities if they've changed every VERSION/timeout
   */
  void update_capabilities(bool known, uint64_t caps = 0);

  /**
   * @brief Adds a function to the capabilities callback queue
   *
   * @param cb A void function that takes a single mavlink::common::MAV_PROTOCOL_CAPABILITY(MAV_CAP) param
   */
  void add_capabilities_change_handler(CapabilitiesCb cb);

  /* -*- utils -*- */

  /**
   * Helper template to set target id's of message.
   */
  template<typename _T>
  inline void msg_set_target(_T & msg)
  {
    msg.target_system = get_tgt_system();
    msg.target_component = get_tgt_component();
  }

  /**
   * @brief Check that sys/comp id's is my target
   */
  inline bool is_my_target(uint8_t sysid, uint8_t compid)
  {
    return sysid == get_tgt_system() && compid == get_tgt_component();
  }

  /**
   * @brief Check that system id is my target
   */
  inline bool is_my_target(uint8_t sysid)
  {
    return sysid == get_tgt_system();
  }

  /**
   * @brief Check that FCU is APM
   */
  inline bool is_ardupilotmega()
  {
    return MAV_AUTOPILOT::ARDUPILOTMEGA == get_autopilot();
  }

  /**
   * @brief Check that FCU is PX4
   */
  inline bool is_px4()
  {
    return MAV_AUTOPILOT::PX4 == get_autopilot();
  }

  /**
   * @brief Represent FCU mode as string
   *
   * Port pymavlink mavutil.mode_string_v10
   *
   * Supported FCU's:
   * - APM:Plane
   * - APM:Copter
   * - PX4
   *
   * @param[in] base_mode    base mode
   * @param[in] custom_mode  custom mode data
   */
  std::string str_mode_v10(uint8_t base_mode, uint32_t custom_mode);

  /**
   * @brief Lookup custom mode for given string
   *
   * Complimentary to @a str_mode_v10()
   *
   * @param[in]  cmode_str   string representation of mode
   * @param[out] custom_mode decoded value
   * @return true if success
   */
  bool cmode_from_str(std::string cmode_str, uint32_t & custom_mode);

  /**
   * @brief Send message to UAS
   */
  inline void send_message(const mavlink::Message & msg)
  {
    send_message(msg, source_component);
  }

  /**
   * @brief Send message to UAS with custom component id
   */
  void send_message(const mavlink::Message & msg, const uint8_t src_compid);

  //! sets protocol version
  void set_protocol_version(mavconn::Protocol ver);

private:
  friend class TestUAS;

  // params
  uint8_t source_system;
  uint8_t source_component;
  uint8_t target_system;
  uint8_t target_component;

  std::string uas_url;

  StrV plugin_allowlist;
  StrV plugin_denylist;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_ptr;
  rclcpp::TimerBase::SharedPtr startup_delay_timer;

  // XXX(vooon): we have to use own executor because Node::create_sub_node() doesn't work for us.
  using thread_ptr = std::unique_ptr<std::thread, std::function<void (std::thread *)>>;
  thread_ptr exec_spin_thd;
  // rclcpp::executors::MultiThreadedExecutor exec;
  UASExecutor exec;

  // plugins
  pluginlib::ClassLoader<plugin::PluginFactory> plugin_factory_loader;
  std::vector<plugin::Plugin::SharedPtr> loaded_plugins;

  //! UAS link -> router -> plugin handler
  std::unordered_map<mavlink::msgid_t, plugin::Plugin::Subscriptions> plugin_subscriptions;

  std::shared_timed_mutex mu;

  // essential data
  std::atomic<uint8_t> type;
  std::atomic<uint8_t> autopilot;
  std::atomic<uint8_t> base_mode;
  std::atomic<bool> fcu_caps_known;
  std::atomic<uint64_t> fcu_capabilities;

  std::atomic<bool> connected;
  std::vector<ConnectionCb> connection_cb_vec;
  std::vector<CapabilitiesCb> capabilities_cb_vec;

  std::atomic<uint64_t> time_offset;
  timesync_mode tsync_mode;

  // UAS -> Router connection
  mavlink::mavlink_status_t mavlink_status;
  rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr source;    // FCU -> UAS
  rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr sink;         // UAS -> FCU

  //! initialize connection to the Router
  void connect_to_router();

  //! uas message receive handler
  void recv_message(const mavros_msgs::msg::Mavlink::SharedPtr rmsg);

  //! message router
  void plugin_route(const mavlink::mavlink_message_t * mmsg, const mavconn::Framing framing);

  //! check if plugin allowed to load
  bool is_plugin_allowed(const std::string & pl_name);

  //! makes an instance of the plugin
  virtual plugin::Plugin::SharedPtr create_plugin_instance(const std::string & pl_name);

  //! load plugin
  void add_plugin(const std::string & pl_name);

  rcl_interfaces::msg::SetParametersResult on_set_parameters_cb(
    const std::vector<rclcpp::Parameter> & parameters);

  void log_connect_change(bool connected);

  void diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat);
};
}              // namespace uas
}       // namespace mavros

#endif  // MAVROS__MAVROS_UAS_HPP_
