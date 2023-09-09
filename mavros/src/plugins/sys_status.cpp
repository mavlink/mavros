/*
 * Copyright 2013,2014,2015,2016,2021 Vladimir Ermakov.
 * Copyright 2022 Dr.-Ing. Amilcar do Carmo Lucas, IAV GmbH.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief System Status plugin
 * @file sys_status.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <utility>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/estimator_status.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/srv/stream_rate.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/msg/vehicle_info.hpp"
#include "mavros_msgs/srv/vehicle_info_get.hpp"
#include "mavros_msgs/srv/message_interval.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace mavros
{
namespace std_plugins
{
using mavlink::minimal::MAV_TYPE;
using mavlink::minimal::MAV_AUTOPILOT;
using mavlink::minimal::MAV_STATE;
using utils::enum_value;
using BatteryMsg = sensor_msgs::msg::BatteryState;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT

#define MAX_NR_BATTERY_STATUS 10

/**
 * Heartbeat status publisher
 *
 * Based on diagnostic_updater::FrequencyStatus
 */
class HeartbeatStatus : public diagnostic_updater::DiagnosticTask
{
public:
  HeartbeatStatus(const std::string & name, size_t win_size)
  : diagnostic_updater::DiagnosticTask(name),
    times_(win_size),
    seq_nums_(win_size),
    window_size_(win_size),
    min_freq_(0.2),
    max_freq_(100),
    tolerance_(0.1),
    autopilot(MAV_AUTOPILOT::GENERIC),
    type(MAV_TYPE::GENERIC),
    system_status(MAV_STATE::UNINIT)
  {
    clear();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex);
    rclcpp::Time curtime = clock.now();
    count_ = 0;

    for (size_t i = 0; i < window_size_; i++) {
      times_[i] = curtime;
      seq_nums_[i] = count_;
    }

    hist_indx_ = 0;
  }

  void tick(
    uint8_t type_, uint8_t autopilot_,
    std::string & mode_, uint8_t system_status_)
  {
    std::lock_guard<std::mutex> lock(mutex);
    count_++;

    type = static_cast<MAV_TYPE>(type_);
    autopilot = static_cast<MAV_AUTOPILOT>(autopilot_);
    mode = mode_;
    system_status = static_cast<MAV_STATE>(system_status_);
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mutex);

    rclcpp::Time curtime = clock.now();
    int curseq = count_;
    int events = curseq - seq_nums_[hist_indx_];
    double window = (curtime - times_[hist_indx_]).seconds();
    double freq = events / window;
    seq_nums_[hist_indx_] = curseq;
    times_[hist_indx_] = curtime;
    hist_indx_ = (hist_indx_ + 1) % window_size_;

    if (events == 0) {
      stat.summary(DiagnosticStatus::ERROR, "No events recorded.");
    } else if (freq < min_freq_ * (1 - tolerance_)) {
      stat.summary(DiagnosticStatus::WARN, "Frequency too low.");
    } else if (freq > max_freq_ * (1 + tolerance_)) {
      stat.summary(DiagnosticStatus::WARN, "Frequency too high.");
    } else {
      stat.summary(DiagnosticStatus::OK, "Normal");
    }

    stat.addf("Heartbeats since startup", "%d", count_);
    stat.addf("Frequency (Hz)", "%f", freq);
    stat.add("Vehicle type", utils::to_string(type));
    stat.add("Autopilot type", utils::to_string(autopilot));
    stat.add("Mode", mode);
    stat.add("System status", utils::to_string(system_status));
  }

private:
  rclcpp::Clock clock;
  int count_;
  std::vector<rclcpp::Time> times_;
  std::vector<int> seq_nums_;
  int hist_indx_;
  std::mutex mutex;
  const size_t window_size_;
  const double min_freq_;
  const double max_freq_;
  const double tolerance_;

  MAV_AUTOPILOT autopilot;
  MAV_TYPE type;
  std::string mode;
  MAV_STATE system_status;
};


/**
 * @brief System status diagnostic updater
 */
class SystemStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
  explicit SystemStatusDiag(const std::string & name)
  : diagnostic_updater::DiagnosticTask(name),
    last_st{}
  {}

  void set(mavlink::common::msg::SYS_STATUS & st)
  {
    std::lock_guard<std::mutex> lock(mutex);
    last_st = st;
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mutex);

    if ((last_st.onboard_control_sensors_health & last_st.onboard_control_sensors_enabled) !=
      last_st.onboard_control_sensors_enabled)
    {
      stat.summary(DiagnosticStatus::ERROR, "Sensor health");
    } else {
      stat.summary(DiagnosticStatus::OK, "Normal");
    }

    stat.addf("Sensor present", "0x%08X", last_st.onboard_control_sensors_present);
    stat.addf("Sensor enabled", "0x%08X", last_st.onboard_control_sensors_enabled);
    stat.addf("Sensor health", "0x%08X", last_st.onboard_control_sensors_health);

    using STS = mavlink::common::MAV_SYS_STATUS_SENSOR;

    auto check_flag = [&](const std::string & name, STS flag) {
        if (last_st.onboard_control_sensors_enabled & enum_value(flag)) {
          stat.add(
            name, (last_st.onboard_control_sensors_health & enum_value(flag)) ? "Ok" : "Fail");
        }
      };

    // [[[cog:
    // import pymavlink.dialects.v20.common as common
    // ename = 'MAV_SYS_STATUS_SENSOR'
    // ename_pfx2 = 'MAV_SYS_STATUS_'
    //
    // enum = sorted(common.enums[ename].items())
    // enum.pop() # -> remove ENUM_END
    //
    // for k, e in enum:
    //     desc = (e.description.split(' ', 1)[1]
    //         if e.description.startswith('0x')
    //         else e.description)
    //     sts = e.name
    //
    //     if sts.startswith(ename + '_'):
    //         sts = sts[len(ename) + 1:]
    //     if sts.startswith(ename_pfx2):
    //         sts = sts[len(ename_pfx2):]
    //     if sts[0].isdigit():
    //         sts = 'SENSOR_' + sts
    //
    //     cog.outl(f"""check_flag("{desc.strip()}", STS::{sts});""")
    // ]]]
    check_flag("3D gyro", STS::SENSOR_3D_GYRO);
    check_flag("3D accelerometer", STS::SENSOR_3D_ACCEL);
    check_flag("3D magnetometer", STS::SENSOR_3D_MAG);
    check_flag("absolute pressure", STS::ABSOLUTE_PRESSURE);
    check_flag("differential pressure", STS::DIFFERENTIAL_PRESSURE);
    check_flag("GPS", STS::GPS);
    check_flag("optical flow", STS::OPTICAL_FLOW);
    check_flag("computer vision position", STS::VISION_POSITION);
    check_flag("laser based position", STS::LASER_POSITION);
    check_flag("external ground truth (Vicon or Leica)", STS::EXTERNAL_GROUND_TRUTH);
    check_flag("3D angular rate control", STS::ANGULAR_RATE_CONTROL);
    check_flag("attitude stabilization", STS::ATTITUDE_STABILIZATION);
    check_flag("yaw position", STS::YAW_POSITION);
    check_flag("z/altitude control", STS::Z_ALTITUDE_CONTROL);
    check_flag("x/y position control", STS::XY_POSITION_CONTROL);
    check_flag("motor outputs / control", STS::MOTOR_OUTPUTS);
    check_flag("rc receiver", STS::RC_RECEIVER);
    check_flag("2nd 3D gyro", STS::SENSOR_3D_GYRO2);
    check_flag("2nd 3D accelerometer", STS::SENSOR_3D_ACCEL2);
    check_flag("2nd 3D magnetometer", STS::SENSOR_3D_MAG2);
    check_flag("geofence", STS::GEOFENCE);
    check_flag("AHRS subsystem health", STS::AHRS);
    check_flag("Terrain subsystem health", STS::TERRAIN);
    check_flag("Motors are reversed", STS::REVERSE_MOTOR);
    check_flag("Logging", STS::LOGGING);
    check_flag("Battery", STS::BATTERY);
    check_flag("Proximity", STS::PROXIMITY);
    check_flag("Satellite Communication", STS::SATCOM);
    check_flag("pre-arm check status. Always healthy when armed", STS::PREARM_CHECK);
    check_flag("Avoidance/collision prevention", STS::OBSTACLE_AVOIDANCE);
    check_flag("propulsion (actuator, esc, motor or propellor)", STS::PROPULSION);
    // [[[end]]] (checksum: 435a149e38737aac78b4be94b670a6dd)

    stat.addf("CPU Load (%)", "%.1f", last_st.load / 10.0);
    stat.addf("Drop rate (%)", "%.1f", last_st.drop_rate_comm / 10.0);
    stat.addf("Errors comm", "%d", last_st.errors_comm);
    stat.addf("Errors count #1", "%d", last_st.errors_count1);
    stat.addf("Errors count #2", "%d", last_st.errors_count2);
    stat.addf("Errors count #3", "%d", last_st.errors_count3);
    stat.addf("Errors count #4", "%d", last_st.errors_count4);
  }

private:
  std::mutex mutex;
  mavlink::common::msg::SYS_STATUS last_st;
};


/**
 * @brief Battery diagnostic updater
 */
class BatteryStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
  explicit BatteryStatusDiag(const std::string & name)
  : diagnostic_updater::DiagnosticTask(name),
    voltage(-1.0f),
    current(0.0f),
    remaining(0.0f),
    min_voltage(6.0f)
  {}

  // Move constructor, required to dynamically create an array of instances of this class
  // because it contains an unique mutex object
  BatteryStatusDiag(BatteryStatusDiag && other) noexcept
  : diagnostic_updater::DiagnosticTask(""),
    voltage(-1.0f),
    current(0.0f),
    remaining(0.0f),
    min_voltage(6.0f)
  {
    *this = std::move(other);
  }

  // Move assignment operator, required to dynamically create an array of instances of this class
  // because it contains an unique mutex object
  BatteryStatusDiag & operator=(BatteryStatusDiag && other) noexcept
  {
    if (this != &other) {
      *this = std::move(other);
    }
    return *this;
  }

  void set_min_voltage(float volt)
  {
    std::lock_guard<std::mutex> lock(mutex);
    min_voltage = volt;
  }

  void set(float volt, float curr, float rem)
  {
    std::lock_guard<std::mutex> lock(mutex);
    voltage = volt;
    current = curr;
    remaining = rem;
  }

  void setcell_v(const std::vector<float> voltages)
  {
    std::lock_guard<std::mutex> lock(mutex);
    cell_voltage = voltages;
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mutex);

    if (voltage < 0.0f) {
      stat.summary(DiagnosticStatus::ERROR, "No data");
    } else if (voltage < min_voltage) {
      stat.summary(DiagnosticStatus::WARN, "Low voltage");
    } else {
      stat.summary(DiagnosticStatus::OK, "Normal");
    }

    stat.addf("Voltage", "%.2f", voltage);
    stat.addf("Current", "%.1f", current);
    stat.addf("Remaining", "%.1f", remaining * 100.0f);
    const int nr_cells = cell_voltage.size();
    if (nr_cells > 1) {
      for (int i = 1; i <= nr_cells; ++i) {
        stat.addf(utils::format("Cell %u", i), "%.2f", cell_voltage[i - 1]);
      }
    }
  }

private:
  std::mutex mutex;
  float voltage;
  float current;
  float remaining;
  float min_voltage;
  std::vector<float> cell_voltage;
};


/**
 * @brief Memory usage diag (APM-only)
 */
class MemInfo : public diagnostic_updater::DiagnosticTask
{
public:
  explicit MemInfo(const std::string & name)
  : diagnostic_updater::DiagnosticTask(name),
    freemem(UINT32_MAX),
    brkval(0),
    last_rcd(0)
  {}

  void set(uint32_t f, uint16_t b)
  {
    freemem = f;
    brkval = b;
    last_rcd = clock.now().nanoseconds();
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    // access atomic variables just once
    size_t freemem_ = freemem;
    uint16_t brkval_ = brkval;
    rclcpp::Time last_rcd_(last_rcd.load());
    const rclcpp::Duration timeout(10s);              // seconds

    // summarize the results
    if (last_rcd_.nanoseconds() == 0) {
      stat.summary(DiagnosticStatus::ERROR, "Not initialised");
    } else if (clock.now() - last_rcd_ > timeout) {
      stat.summary(
        DiagnosticStatus::STALE,
        "Not received for more than " + std::to_string(timeout.seconds()));
    } else {
      if (freemem == UINT32_MAX) {
        stat.summary(DiagnosticStatus::ERROR, "No data");
      } else if (freemem < 200) {
        stat.summary(DiagnosticStatus::WARN, "Low mem");
      } else {
        stat.summary(DiagnosticStatus::OK, "Normal");
      }
    }
    stat.addf("Free memory (B)", "%zd", freemem_);
    stat.addf("Heap top", "0x%04X", brkval_);
  }

private:
  rclcpp::Clock clock;
  std::atomic<size_t> freemem;
  std::atomic<uint16_t> brkval;
  std::atomic<uint64_t> last_rcd;
};


/**
 * @brief Hardware status (APM-only)
 */
class HwStatus : public diagnostic_updater::DiagnosticTask
{
public:
  explicit HwStatus(const std::string & name)
  : diagnostic_updater::DiagnosticTask(name),
    vcc(-1.0),
    i2cerr(0),
    i2cerr_last(0),
    last_rcd(0)
  {}

  void set(uint16_t v, uint8_t e)
  {
    std::lock_guard<std::mutex> lock(mutex);
    vcc = v * 0.001f;
    i2cerr = e;
    last_rcd = clock.now();
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const rclcpp::Duration timeout(10s);               // seconds
    if (last_rcd.nanoseconds() == 0) {
      stat.summary(DiagnosticStatus::ERROR, "Not initialised");
    } else if (clock.now() - last_rcd > timeout) {
      stat.summary(
        DiagnosticStatus::STALE, "Not received for more than " + std::to_string(
          timeout.seconds()));
    } else {
      if (vcc < 0) {
        stat.summary(DiagnosticStatus::ERROR, "No data");
      } else if (vcc < 4.5) {
        stat.summary(DiagnosticStatus::WARN, "Low voltage");
      } else if (i2cerr != i2cerr_last) {
        i2cerr_last = i2cerr;
        stat.summary(DiagnosticStatus::WARN, "New I2C error");
      } else {
        stat.summary(DiagnosticStatus::OK, "Normal");
      }
    }
    stat.addf("Core voltage", "%f", vcc);
    stat.addf("I2C errors", "%zu", i2cerr);
  }

private:
  rclcpp::Clock clock;
  std::mutex mutex;
  float vcc;
  size_t i2cerr;
  size_t i2cerr_last;
  rclcpp::Time last_rcd;
};


/**
 * @brief System status plugin.
 * @plugin sys_status
 *
 * Required by all plugins.
 */
class SystemStatusPlugin : public plugin::Plugin
{
public:
  explicit SystemStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "sys"),
    hb_diag("Heartbeat", 10),
    mem_diag("APM Memory"),
    hwst_diag("APM Hardware"),
    sys_diag("System"),
    conn_heartbeat_mav_type(MAV_TYPE::ONBOARD_CONTROLLER),
    version_retries(RETRIES_COUNT),
    disable_diag(false),
    has_battery_status0(false)
  {
    batt_diag.reserve(MAX_NR_BATTERY_STATUS);
    batt_diag.emplace_back("Battery");
    for (size_t i = 2; i <= MAX_NR_BATTERY_STATUS; ++i) {
      batt_diag.emplace_back(utils::format("Battery %u", i));
    }

    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "conn_timeout", 10.0, [&](const rclcpp::Parameter & p) {
        auto conn_timeout = rclcpp::Duration::from_seconds(p.as_double());

        timeout_timer =
        node->create_wall_timer(
          conn_timeout.to_chrono<std::chrono::nanoseconds>(),
          std::bind(&SystemStatusPlugin::timeout_cb, this));
      });

    node_declare_and_watch_parameter(
      "min_voltage", std::vector<double>({10.0}), [&](const rclcpp::Parameter & p) {
        min_voltage = p.as_double_array();
        for (size_t i = 0; i < batt_diag.size() && i < min_voltage.size(); ++i) {
          batt_diag[i].set_min_voltage(min_voltage[i]);
        }
      });

    node_declare_and_watch_parameter(
      "disable_diag", false, [&](const rclcpp::Parameter & p) {
        disable_diag = p.as_bool();

        if (!disable_diag) {
          uas->diagnostic_updater.add(sys_diag);
          for (size_t i = 0; i < batt_diag.size() && i < min_voltage.size(); ++i) {
            uas->diagnostic_updater.add(batt_diag[i]);
          }

        } else {
          uas->diagnostic_updater.removeByName(sys_diag.getName());
          uas->diagnostic_updater.removeByName(mem_diag.getName());
          uas->diagnostic_updater.removeByName(hwst_diag.getName());
          for (auto & d : batt_diag) {
            uas->diagnostic_updater.removeByName(d.getName());
          }
        }
      });

    node_declare_and_watch_parameter(
      "heartbeat_mav_type", utils::enum_to_name(
        conn_heartbeat_mav_type), [&](const rclcpp::Parameter & p) {
        conn_heartbeat_mav_type = utils::mav_type_from_str(p.as_string());
      });

    node_declare_and_watch_parameter(
      "heartbeat_rate", 1.0, [&](const rclcpp::Parameter & p) {
        auto rate_d = p.as_double();

        if (rate_d == 0) {
          if (heartbeat_timer) {
            heartbeat_timer->cancel();
            heartbeat_timer.reset();
          }
        } else {
          rclcpp::WallRate rate(rate_d);

          heartbeat_timer =
          node->create_wall_timer(
            rate.period(), std::bind(
              &SystemStatusPlugin::heartbeat_cb,
              this));
        }
      });

    auto state_qos = rclcpp::QoS(10).transient_local();
    auto sensor_qos = rclcpp::SensorDataQoS();

    state_pub = node->create_publisher<mavros_msgs::msg::State>(
      "state", state_qos);
    extended_state_pub = node->create_publisher<mavros_msgs::msg::ExtendedState>(
      "extended_state", state_qos);
    estimator_status_pub = node->create_publisher<mavros_msgs::msg::EstimatorStatus>(
      "estimator_status", state_qos);
    batt_pub = node->create_publisher<BatteryMsg>("battery", sensor_qos);

    statustext_pub = node->create_publisher<mavros_msgs::msg::StatusText>(
      "statustext/recv", sensor_qos);
    statustext_sub = node->create_subscription<mavros_msgs::msg::StatusText>(
      "statustext/send", sensor_qos,
      std::bind(&SystemStatusPlugin::statustext_cb, this, _1));

    srv_cg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    mode_srv = node->create_service<mavros_msgs::srv::SetMode>(
      "set_mode",
      std::bind(
        &SystemStatusPlugin::set_mode_cb, this, _1,
        _2), rmw_qos_profile_services_default, srv_cg);
    stream_rate_srv = node->create_service<mavros_msgs::srv::StreamRate>(
      "set_stream_rate",
      std::bind(
        &SystemStatusPlugin::set_rate_cb, this, _1,
        _2), rmw_qos_profile_services_default, srv_cg);
    message_interval_srv = node->create_service<mavros_msgs::srv::MessageInterval>(
      "set_message_interval",
      std::bind(
        &SystemStatusPlugin::set_message_interval_cb, this, _1,
        _2), rmw_qos_profile_services_default, srv_cg);
    vehicle_info_get_srv = node->create_service<mavros_msgs::srv::VehicleInfoGet>(
      "vehicle_info_get", std::bind(
        &SystemStatusPlugin::vehicle_info_get_cb, this, _1,
        _2), rmw_qos_profile_services_default, srv_cg);

    uas->diagnostic_updater.add(hb_diag);

    autopilot_version_timer = node->create_wall_timer(
      1s, std::bind(&SystemStatusPlugin::autopilot_version_cb, this),
      srv_cg);

    // init state topic
    publish_disconnection();
    enable_connection_cb();
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&SystemStatusPlugin::handle_heartbeat),
      make_handler(&SystemStatusPlugin::handle_sys_status),
      make_handler(&SystemStatusPlugin::handle_statustext),
      make_handler(&SystemStatusPlugin::handle_meminfo),
      make_handler(&SystemStatusPlugin::handle_hwstatus),
      make_handler(&SystemStatusPlugin::handle_autopilot_version),
      make_handler(&SystemStatusPlugin::handle_extended_sys_state),
      make_handler(&SystemStatusPlugin::handle_battery_status),
      make_handler(&SystemStatusPlugin::handle_estimator_status),
    };
  }

private:
  HeartbeatStatus hb_diag;
  MemInfo mem_diag;
  HwStatus hwst_diag;
  SystemStatusDiag sys_diag;
  std::vector<BatteryStatusDiag> batt_diag;

  rclcpp::TimerBase::SharedPtr timeout_timer;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  rclcpp::TimerBase::SharedPtr autopilot_version_timer;

  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub;
  rclcpp::Publisher<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_pub;
  rclcpp::Publisher<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_status_pub;
  rclcpp::Publisher<BatteryMsg>::SharedPtr batt_pub;

  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr statustext_pub;
  rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr statustext_sub;

  rclcpp::CallbackGroup::SharedPtr srv_cg;
  rclcpp::Service<mavros_msgs::srv::StreamRate>::SharedPtr stream_rate_srv;
  rclcpp::Service<mavros_msgs::srv::MessageInterval>::SharedPtr message_interval_srv;
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr mode_srv;
  rclcpp::Service<mavros_msgs::srv::VehicleInfoGet>::SharedPtr vehicle_info_get_srv;

  MAV_TYPE conn_heartbeat_mav_type;
  static constexpr int RETRIES_COUNT = 6;
  int version_retries;
  bool disable_diag;
  bool has_battery_status0;
  float battery_voltage;
  std::vector<double> min_voltage;

  using M_VehicleInfo = std::unordered_map<uint16_t, mavros_msgs::msg::VehicleInfo>;
  M_VehicleInfo vehicles;

  /* -*- mid-level helpers -*- */

  // Get vehicle key for the unordered map containing all vehicles
  inline uint16_t get_vehicle_key(uint8_t sysid, uint8_t compid)
  {
    return sysid << 8 | compid;
  }

  // Find or create vehicle info
  inline M_VehicleInfo::iterator find_or_create_vehicle_info(uint8_t sysid, uint8_t compid)
  {
    auto key = get_vehicle_key(sysid, compid);
    M_VehicleInfo::iterator ret = vehicles.find(key);

    if (ret == vehicles.end()) {
      // Not found
      mavros_msgs::msg::VehicleInfo v;
      v.sysid = sysid;
      v.compid = compid;
      v.available_info = 0;

      auto res = vehicles.emplace(key, v);                      //-> pair<iterator, bool>
      ret = res.first;
    }

    rcpputils::assert_true(ret != vehicles.end());
    return ret;
  }

  /**
   * Sent STATUSTEXT message to rosout
   *
   * @param[in] severity  Levels defined in common.xml
   */
  void process_statustext_normal(uint8_t severity, const std::string & text)
  {
    using mavlink::common::MAV_SEVERITY;

    switch (severity) {
      // [[[cog:
      // for l1, l2 in (
      //     (('EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR'), 'ERROR'),
      //     (('WARNING', 'NOTICE'), 'WARN'),
      //     (('INFO', ), 'INFO'),
      //     (('DEBUG', ), 'DEBUG')
      //     ):
      //     for v in l1:
      //         cog.outl(f"case enum_value(MAV_SEVERITY::{v}):")
      //     cog.outl(f"  RCLCPP_{l2}_STREAM(node->get_logger(), \"FCU: \" << text);")
      //     cog.outl(f"  break;")
      // ]]]
      case enum_value(MAV_SEVERITY::EMERGENCY):
      case enum_value(MAV_SEVERITY::ALERT):
      case enum_value(MAV_SEVERITY::CRITICAL):
      case enum_value(MAV_SEVERITY::ERROR):
        RCLCPP_ERROR_STREAM(node->get_logger(), "FCU: " << text);
        break;
      case enum_value(MAV_SEVERITY::WARNING):
      case enum_value(MAV_SEVERITY::NOTICE):
        RCLCPP_WARN_STREAM(node->get_logger(), "FCU: " << text);
        break;
      case enum_value(MAV_SEVERITY::INFO):
        RCLCPP_INFO_STREAM(node->get_logger(), "FCU: " << text);
        break;
      case enum_value(MAV_SEVERITY::DEBUG):
        RCLCPP_DEBUG_STREAM(node->get_logger(), "FCU: " << text);
        break;
      // [[[end]]] (checksum: d05760afbeece46673c8f73f89b63f3d)
      default:
        RCLCPP_WARN_STREAM(node->get_logger(), "FCU: UNK(" << +severity << "): " << text);
        break;
    }
  }

  static std::string custom_version_to_hex_string(const std::array<uint8_t, 8> & array)
  {
    // should be little-endian
    uint64_t b;
    memcpy(&b, array.data(), sizeof(uint64_t));
    b = le64toh(b);

    return utils::format("%016llx", b);
  }

  void process_autopilot_version_normal(
    mavlink::common::msg::AUTOPILOT_VERSION & apv,
    uint8_t sysid, uint8_t compid)
  {
    char prefix[16];
    std::snprintf(prefix, sizeof(prefix), "VER: %d.%d", sysid, compid);

    auto lg = node->get_logger();
    auto log_info = [&lg, &prefix] < typename ... Args > (const std::string & fmt, Args ... args) {
      RCLCPP_INFO(lg, fmt.c_str(), prefix, args ...);
    };  // NOLINT

    log_info("%s: Capabilities         0x%016llx", apv.capabilities);
    log_info(
      "%s: Flight software:     %08x (%s)",
      apv.flight_sw_version, custom_version_to_hex_string(apv.flight_custom_version).c_str());
    log_info(
      "%s: Middleware software: %08x (%s)",
      apv.middleware_sw_version,
      custom_version_to_hex_string(apv.middleware_custom_version).c_str());
    log_info(
      "%s: OS software:         %08x (%s)",
      apv.os_sw_version, custom_version_to_hex_string(apv.os_custom_version).c_str());
    log_info("%s: Board hardware:      %08x", apv.board_version);
    log_info("%s: VID/PID:             %04x:%04x", apv.vendor_id, apv.product_id);
    log_info("%s: UID:                 %016llx", apv.uid);
  }

  void process_autopilot_version_apm_quirk(
    mavlink::common::msg::AUTOPILOT_VERSION & apv,
    uint8_t sysid, uint8_t compid)
  {
    char prefix[16];
    std::snprintf(prefix, sizeof(prefix), "VER: %d.%d", sysid, compid);

    auto lg = node->get_logger();
    auto log_info = [&lg, &prefix] < typename ... Args > (const std::string & fmt, Args ... args) {
      RCLCPP_INFO(lg, fmt.c_str(), prefix, args ...);
    };  // NOLINT

    // Note based on current APM's impl.
    // APM uses custom version array[8] as a string
    log_info("%s: Capabilities         0x%016llx", apv.capabilities);
    log_info(
      "%s: Flight software:     %08x (%*s)",
      apv.flight_sw_version, 8, apv.flight_custom_version.data());
    log_info(
      "%s: Middleware software: %08x (%*s)",
      apv.middleware_sw_version, 8, apv.middleware_custom_version.data());
    log_info(
      "%s: OS software:         %08x (%*s)",
      apv.os_sw_version, 8, apv.os_custom_version.data());
    log_info("%s: Board hardware:      %08x", apv.board_version);
    log_info("%s: VID/PID:             %04x:%04x", apv.vendor_id, apv.product_id);
    log_info("%s: UID:                 %016llx", apv.uid);
  }

  void publish_disconnection()
  {
    auto state_msg = mavros_msgs::msg::State();
    state_msg.header.stamp = node->now();
    state_msg.connected = false;
    state_msg.armed = false;
    state_msg.guided = false;
    state_msg.mode = "";
    state_msg.system_status = enum_value(MAV_STATE::UNINIT);

    state_pub->publish(state_msg);
  }

  /* -*- message handlers -*- */

  void handle_heartbeat(
    const mavlink::mavlink_message_t * msg,
    mavlink::minimal::msg::HEARTBEAT & hb, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    using mavlink::minimal::MAV_MODE_FLAG;

    // XXX(vooon): i assume that UAS not interested in HBs from non-target system.

    // Store generic info of all heartbeats seen
    auto it = find_or_create_vehicle_info(msg->sysid, msg->compid);

    auto vehicle_mode = uas->str_mode_v10(hb.base_mode, hb.custom_mode);
    auto stamp = node->now();

    // Update vehicle data
    it->second.header.stamp = stamp;
    it->second.available_info |= mavros_msgs::msg::VehicleInfo::HAVE_INFO_HEARTBEAT;
    it->second.autopilot = hb.autopilot;
    it->second.type = hb.type;
    it->second.system_status = hb.system_status;
    it->second.base_mode = hb.base_mode;
    it->second.custom_mode = hb.custom_mode;
    it->second.mode = vehicle_mode;

    if (!(hb.base_mode & enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED))) {
      it->second.mode_id = hb.base_mode;
    } else {
      it->second.mode_id = hb.custom_mode;
    }

    // Continue from here only if vehicle is my target
    if (!uas->is_my_target(msg->sysid, msg->compid)) {
      RCLCPP_DEBUG(node->get_logger(), "HEARTBEAT from %d.%d dropped.", msg->sysid, msg->compid);
      return;
    }

    // update context && setup connection timeout
    uas->update_heartbeat(hb.type, hb.autopilot, hb.base_mode);
    uas->update_connection_status(true);
    timeout_timer->reset();

    // build state message after updating uas
    auto state_msg = mavros_msgs::msg::State();
    state_msg.header.stamp = stamp;
    state_msg.connected = true;
    state_msg.armed = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::SAFETY_ARMED));
    state_msg.guided = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::GUIDED_ENABLED));
    state_msg.manual_input = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::MANUAL_INPUT_ENABLED));
    state_msg.mode = vehicle_mode;
    state_msg.system_status = hb.system_status;

    state_pub->publish(state_msg);
    hb_diag.tick(hb.type, hb.autopilot, state_msg.mode, hb.system_status);
  }

  void handle_extended_sys_state(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::EXTENDED_SYS_STATE & state,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto state_msg = mavros_msgs::msg::ExtendedState();
    state_msg.header.stamp = node->now();
    state_msg.vtol_state = state.vtol_state;
    state_msg.landed_state = state.landed_state;

    extended_state_pub->publish(state_msg);
  }

  void handle_sys_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::SYS_STATUS & stat,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    using MC = mavlink::minimal::MAV_COMPONENT;
    if (static_cast<MC>(msg->compid) == MC::COMP_ID_GIMBAL) {
      return;
    }

    float volt = stat.voltage_battery / 1000.0f;                // mV
    float curr = stat.current_battery / 100.0f;                 // 10 mA or -1
    float rem = stat.battery_remaining / 100.0f;                // or -1

    battery_voltage = volt;
    sys_diag.set(stat);

    if (has_battery_status0) {
      return;
    }

    batt_diag[0].set(volt, curr, rem);

    auto batt_msg = BatteryMsg();
    batt_msg.header.stamp = node->now();
    batt_msg.voltage = volt;
    batt_msg.current = -curr;
    batt_msg.charge = NAN;
    batt_msg.capacity = NAN;
    batt_msg.design_capacity = NAN;
    batt_msg.percentage = rem;
    batt_msg.power_supply_status = BatteryMsg::POWER_SUPPLY_STATUS_DISCHARGING;
    batt_msg.power_supply_health = BatteryMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    batt_msg.present = true;
    batt_msg.cell_voltage.clear();             // not necessary. Cell count and Voltage unknown.
    batt_msg.location = "";
    batt_msg.serial_number = "";

    batt_pub->publish(batt_msg);
  }

  void handle_statustext(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::STATUSTEXT & textm,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto text = mavlink::to_string(textm.text);
    process_statustext_normal(textm.severity, text);

    auto st_msg = mavros_msgs::msg::StatusText();
    st_msg.header.stamp = node->now();
    st_msg.severity = textm.severity;
    st_msg.text = text;

    statustext_pub->publish(st_msg);
  }

  void handle_meminfo(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::MEMINFO & mem,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    mem_diag.set(std::max(static_cast<uint32_t>(mem.freemem), mem.freemem32), mem.brkval);
  }

  void handle_hwstatus(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::HWSTATUS & hwst,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    hwst_diag.set(hwst.Vcc, hwst.I2Cerr);
  }

  void handle_autopilot_version(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::AUTOPILOT_VERSION & apv,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // XXX(vooon): i assume that UAS no longer interested in other systems
    //             so will recv versions only from target system's components

    // we want to store only FCU caps
    if (uas->is_my_target(msg->sysid, msg->compid)) {
      autopilot_version_timer->cancel();
      uas->update_capabilities(true, apv.capabilities);
    }

    // but print all version responses
    if (uas->is_ardupilotmega()) {
      process_autopilot_version_apm_quirk(apv, msg->sysid, msg->compid);
    } else {
      process_autopilot_version_normal(apv, msg->sysid, msg->compid);
    }

    // Store generic info of all autopilot seen
    auto it = find_or_create_vehicle_info(msg->sysid, msg->compid);

    // Update vehicle data
    it->second.header.stamp = node->now();
    it->second.available_info |= mavros_msgs::msg::VehicleInfo::HAVE_INFO_AUTOPILOT_VERSION;
    it->second.capabilities = apv.capabilities;
    it->second.flight_sw_version = apv.flight_sw_version;
    it->second.middleware_sw_version = apv.middleware_sw_version;
    it->second.os_sw_version = apv.os_sw_version;
    it->second.board_version = apv.board_version;
    it->second.flight_custom_version = custom_version_to_hex_string(apv.flight_custom_version);
    it->second.vendor_id = apv.vendor_id;
    it->second.product_id = apv.product_id;
    it->second.uid = apv.uid;
  }

  void handle_battery_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::BATTERY_STATUS & bs,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // PX4.
    using BT = mavlink::common::MAV_BATTERY_TYPE;

    auto batt_msg = BatteryMsg();
    batt_msg.header.stamp = node->now();
    batt_msg.voltage = battery_voltage;
    batt_msg.current = -(bs.current_battery / 100.0f);                 // 10 mA
    batt_msg.charge = NAN;
    batt_msg.capacity = NAN;
    batt_msg.design_capacity = NAN;
    batt_msg.percentage = bs.battery_remaining / 100.0f;
    batt_msg.power_supply_status = BatteryMsg::POWER_SUPPLY_STATUS_DISCHARGING;
    batt_msg.power_supply_health = BatteryMsg::POWER_SUPPLY_HEALTH_UNKNOWN;

    switch (bs.type) {
      // [[[cog:
      // for f in (
      //     'LIPO',
      //     'LIFE',
      //     'LION',
      //     'NIMH',
      //     'UNKNOWN'):
      //     cog.outl(f"case enum_value(BT::{f}):")
      //     if f == 'UNKNOWN':
      //         cog.outl("default:")
      //     cog.outl(
      //         f"  batt_msg.power_supply_technology = "
      //         f"BatteryMsg::POWER_SUPPLY_TECHNOLOGY_{f};")
      //     cog.outl(f"  break;")
      // ]]]
      case enum_value(BT::LIPO):
        batt_msg.power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LIPO;
        break;
      case enum_value(BT::LIFE):
        batt_msg.power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LIFE;
        break;
      case enum_value(BT::LION):
        batt_msg.power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_LION;
        break;
      case enum_value(BT::NIMH):
        batt_msg.power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_NIMH;
        break;
      case enum_value(BT::UNKNOWN):
      default:
        batt_msg.power_supply_technology = BatteryMsg::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        break;
        // [[[end]]] (checksum: 9ff6279a1b2c35e23e0b2e09e915c5ca)
    }

    batt_msg.present = true;

    batt_msg.cell_voltage.clear();
    batt_msg.cell_voltage.reserve(bs.voltages.size() + bs.voltages_ext.size());
    float cell_voltage;
    float voltage_acc = 0.0f;
    float total_voltage = 0.0f;
    // 65,534V cell voltage means that the next element in the array must be added to this one
    constexpr float coalesce_voltage = (UINT16_MAX - 1) * 0.001f;
    for (auto v : bs.voltages) {
      if (v == UINT16_MAX) {
        break;
      }

      if (v == UINT16_MAX - 1) {    // cell voltage is above 65,534V
        voltage_acc += coalesce_voltage;
        continue;                   // add to the next array element to get the correct voltage
      }

      cell_voltage = voltage_acc + (v * 0.001f);    // 1 mV
      voltage_acc = 0.0f;
      batt_msg.cell_voltage.push_back(cell_voltage);
      total_voltage += cell_voltage;
    }
    for (auto v : bs.voltages_ext) {
      if (v == UINT16_MAX || v == 0) {
        // this one is different from the for loop above to support mavlink2 message truncation
        break;
      }

      if (v == UINT16_MAX - 1) {
        // cell voltage is above 65,534V
        // add to the next array element to get the correct voltage
        voltage_acc += coalesce_voltage;
        continue;
      }

      cell_voltage = voltage_acc + (v * 0.001f);    // 1 mV
      voltage_acc = 0.0f;
      batt_msg.cell_voltage.push_back(cell_voltage);
      total_voltage += cell_voltage;
    }
    batt_msg.voltage = total_voltage;

    batt_msg.location = utils::format("id%u", bs.id);
    batt_msg.serial_number = "";

    batt_pub->publish(batt_msg);


    if (bs.id == 0) {
      has_battery_status0 = true;
    }

    if (!disable_diag && bs.id < MAX_NR_BATTERY_STATUS) {
      batt_diag[bs.id].set(total_voltage, batt_msg.current, batt_msg.percentage);
      batt_diag[bs.id].setcell_v(batt_msg.cell_voltage);
    }
  }

  void handle_estimator_status(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ESTIMATOR_STATUS & status,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    using ESF = mavlink::common::ESTIMATOR_STATUS_FLAGS;

    auto est_status_msg = mavros_msgs::msg::EstimatorStatus();
    est_status_msg.header.stamp = node->now();

    auto check_flag = [status](ESF flag) -> bool {
        return !!(status.flags & enum_value(flag));
      };

    // [[[cog:
    // import pymavlink.dialects.v20.common as common
    // ename = 'ESTIMATOR_STATUS_FLAGS'
    // ename_pfx2 = 'ESTIMATOR_'
    //
    // enum = sorted(common.enums[ename].items())
    // enum.pop() # -> remove ENUM_END
    //
    // for k, e in enum:
    //     esf = e.name
    //
    //     if esf.startswith(ename + '_'):
    //         esf = esf[len(ename) + 1:]
    //     if esf.startswith(ename_pfx2):
    //         esf = esf[len(ename_pfx2):]
    //     if esf[0].isdigit():
    //         esf = 'SENSOR_' + esf
    //     cog.outl(f"est_status_msg.{esf.lower()}_status_flag = check_flag(ESF::{esf});")
    // ]]]
    est_status_msg.attitude_status_flag = check_flag(ESF::ATTITUDE);
    est_status_msg.velocity_horiz_status_flag = check_flag(ESF::VELOCITY_HORIZ);
    est_status_msg.velocity_vert_status_flag = check_flag(ESF::VELOCITY_VERT);
    est_status_msg.pos_horiz_rel_status_flag = check_flag(ESF::POS_HORIZ_REL);
    est_status_msg.pos_horiz_abs_status_flag = check_flag(ESF::POS_HORIZ_ABS);
    est_status_msg.pos_vert_abs_status_flag = check_flag(ESF::POS_VERT_ABS);
    est_status_msg.pos_vert_agl_status_flag = check_flag(ESF::POS_VERT_AGL);
    est_status_msg.const_pos_mode_status_flag = check_flag(ESF::CONST_POS_MODE);
    est_status_msg.pred_pos_horiz_rel_status_flag = check_flag(ESF::PRED_POS_HORIZ_REL);
    est_status_msg.pred_pos_horiz_abs_status_flag = check_flag(ESF::PRED_POS_HORIZ_ABS);
    est_status_msg.gps_glitch_status_flag = check_flag(ESF::GPS_GLITCH);
    est_status_msg.accel_error_status_flag = check_flag(ESF::ACCEL_ERROR);
    // [[[end]]] (checksum: fc30da81f9490dede61a58e82c8a2d53)

    estimator_status_pub->publish(est_status_msg);
  }

  /* -*- timer callbacks -*- */

  void timeout_cb()
  {
    uas->update_connection_status(false);
  }

  void heartbeat_cb()
  {
    using mavlink::common::MAV_MODE;

    mavlink::minimal::msg::HEARTBEAT hb {};
    hb.type = enum_value(conn_heartbeat_mav_type);
    hb.autopilot = enum_value(MAV_AUTOPILOT::INVALID);
    hb.base_mode = enum_value(MAV_MODE::MANUAL_ARMED);
    hb.custom_mode = 0;
    hb.system_status = enum_value(MAV_STATE::ACTIVE);

    uas->send_message(hb);
  }

  void autopilot_version_cb()
  {
    using mavlink::common::MAV_CMD;

    auto lg = get_logger();
    bool ret = false;

    // Request from all first 3 times, then fallback to unicast
    bool do_broadcast = version_retries > RETRIES_COUNT / 2;

    try {
      auto client = node->create_client<mavros_msgs::srv::CommandLong>("cmd/command");

      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      cmdrq->broadcast = do_broadcast;
      cmdrq->command = enum_value(MAV_CMD::REQUEST_AUTOPILOT_CAPABILITIES);
      cmdrq->confirmation = false;
      cmdrq->param1 = 1.0;

      RCLCPP_DEBUG(
        lg, "VER: Sending %s request.",
        (do_broadcast) ? "broadcast" : "unicast");

      auto future = client->async_send_request(cmdrq);
      // NOTE(vooon): temporary hack from @Michel1968
      // See: https://github.com/mavlink/mavros/issues/1588#issuecomment-1027699924
      const auto future_status = future.wait_for(1s);
      if (future_status == std::future_status::ready) {
        auto response = future.get();
        ret = response->success;
      } else {
        RCLCPP_ERROR(lg, "VER: autopilot version service timeout");
      }
    } catch (std::exception & ex) {
      RCLCPP_ERROR_STREAM(lg, "VER: " << ex.what());
    }

    RCLCPP_ERROR_EXPRESSION(lg, !ret, "VER: command plugin service call failed!");

    if (version_retries > 0) {
      version_retries--;
      RCLCPP_WARN_EXPRESSION(
        lg, version_retries != RETRIES_COUNT - 1,
        "VER: %s request timeout, retries left %d",
        (do_broadcast) ? "broadcast" : "unicast",
        version_retries);
    } else {
      uas->update_capabilities(false);
      autopilot_version_timer->cancel();
      RCLCPP_WARN(
        lg,
        "VER: your FCU don't support AUTOPILOT_VERSION, "
        "switched to default capabilities");
    }
  }

  void connection_cb(bool connected) override
  {
    has_battery_status0 = false;

    // if connection changes, start delayed version request
    version_retries = RETRIES_COUNT;
    if (connected) {
      autopilot_version_timer->reset();
    } else {
      autopilot_version_timer->cancel();
    }

    // add/remove APM diag tasks
    if (connected && disable_diag && uas->is_ardupilotmega()) {
      uas->diagnostic_updater.add(mem_diag);
      uas->diagnostic_updater.add(hwst_diag);
    } else {
      uas->diagnostic_updater.removeByName(mem_diag.getName());
      uas->diagnostic_updater.removeByName(hwst_diag.getName());
    }

    if (!connected) {
      // publish connection change
      publish_disconnection();

      // Clear known vehicles
      vehicles.clear();
    }
  }

  /* -*- subscription callbacks -*- */

  void statustext_cb(const mavros_msgs::msg::StatusText::SharedPtr req)
  {
    mavlink::common::msg::STATUSTEXT statustext {};
    statustext.severity = req->severity;
    mavlink::set_string_z(statustext.text, req->text);

    // Limit the length of the string by null-terminating at the 50-th character
    RCLCPP_WARN_EXPRESSION(
      node->get_logger(),
      req->text.length() >= statustext.text.size(),
      "Status text too long: truncating...");

    uas->send_message(statustext);
  }

  /* -*- ros callbacks -*- */

  void set_rate_cb(
    const mavros_msgs::srv::StreamRate::Request::SharedPtr req,
    mavros_msgs::srv::StreamRate::Response::SharedPtr res [[maybe_unused]])
  {
    mavlink::common::msg::REQUEST_DATA_STREAM rq = {};

    uas->msg_set_target(rq);
    rq.req_stream_id = req->stream_id;
    rq.req_message_rate = req->message_rate;
    rq.start_stop = (req->on_off) ? 1 : 0;

    uas->send_message(rq);
  }

  void set_message_interval_cb(
    const mavros_msgs::srv::MessageInterval::Request::SharedPtr req,
    mavros_msgs::srv::MessageInterval::Response::SharedPtr res)
  {
    using mavlink::common::MAV_CMD;

    auto lg = get_logger();

    try {
      auto client = node->create_client<mavros_msgs::srv::CommandLong>("cmd/command");

      // calculate interval
      float interval_us;
      if (req->message_rate < 0) {
        interval_us = -1.0f;
      } else if (req->message_rate == 0) {
        interval_us = 0.0f;
      } else {
        interval_us = 1000000.0f / req->message_rate;
      }

      auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      cmdrq->broadcast = false;
      cmdrq->command = enum_value(MAV_CMD::SET_MESSAGE_INTERVAL);
      cmdrq->confirmation = false;
      cmdrq->param1 = req->message_id;
      cmdrq->param2 = interval_us;

      RCLCPP_DEBUG(
        lg,
        "SYS: Request msgid %u at %f hz",
        req->message_id, req->message_rate);

      auto future = client->async_send_request(cmdrq);
      // NOTE(vooon): same hack as for VER
      const auto future_status = future.wait_for(1s);
      if (future_status == std::future_status::ready) {
        auto response = future.get();
        res->success = response->success;
      } else {
        RCLCPP_ERROR(lg, "SYS: set_message_interval service timeout");
      }
    } catch (std::exception & ex) {
      RCLCPP_ERROR_STREAM(lg, "SYS: " << ex.what());
    }

    RCLCPP_ERROR_EXPRESSION(
      lg, !res->success,
      "SYS: command plugin service call failed!");
  }

  void set_mode_cb(
    const mavros_msgs::srv::SetMode::Request::SharedPtr req,
    mavros_msgs::srv::SetMode::Response::SharedPtr res)
  {
    using mavlink::minimal::MAV_MODE_FLAG;

    uint8_t base_mode = req->base_mode;
    uint32_t custom_mode = 0;

    if (req->custom_mode != "") {
      if (!uas->cmode_from_str(req->custom_mode, custom_mode)) {
        res->mode_sent = false;
        // XXX(vooon): throw?
        return;
      }

      /**
       * @note That call may trigger unexpected arming change because
       *       base_mode arming flag state based on previous HEARTBEAT
       *       message value.
       */
      base_mode |= (uas->get_armed()) ? enum_value(MAV_MODE_FLAG::SAFETY_ARMED) : 0;
      base_mode |= (uas->get_hil_state()) ? enum_value(MAV_MODE_FLAG::HIL_ENABLED) : 0;
      base_mode |= enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
    }

    mavlink::common::msg::SET_MODE sm = {};
    sm.target_system = uas->get_tgt_system();
    sm.base_mode = base_mode;
    sm.custom_mode = custom_mode;

    uas->send_message(sm);
    res->mode_sent = true;
  }

  void vehicle_info_get_cb(
    const mavros_msgs::srv::VehicleInfoGet::Request::SharedPtr req,
    mavros_msgs::srv::VehicleInfoGet::Response::SharedPtr res)
  {
    if (req->get_all) {
      // Send all vehicles
      for (const auto & got : vehicles) {
        res->vehicles.emplace_back(got.second);
      }

      res->success = true;
      return;
    }

    uint8_t req_sysid = req->sysid;
    uint8_t req_compid = req->compid;

    if (req->sysid == 0 && req->compid == 0) {
      // use target
      req_sysid = uas->get_tgt_system();
      req_compid = uas->get_tgt_component();
    }

    uint16_t key = get_vehicle_key(req_sysid, req_compid);
    auto it = vehicles.find(key);
    if (it == vehicles.end()) {
      // Vehicle not found
      res->success = false;
      return;
    }

    res->vehicles.emplace_back(it->second);
    res->success = true;
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SystemStatusPlugin)
