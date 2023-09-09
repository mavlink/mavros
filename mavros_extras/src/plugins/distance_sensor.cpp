/*
 * Copyright 2015 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Distance Sensor plugin
 * @file distance_sensor.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <yaml-cpp/yaml.h>

#include <unordered_map>
#include <string>
#include <memory>
#include <vector>
#include <shared_mutex>     // NOLINT cpplint, that is almost 4 years since standard release!

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "sensor_msgs/msg/range.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT
using utils::enum_value;
using sensor_msgs::msg::Range;

class DistanceSensorPlugin;

/**
 * @brief Distance sensor mapping storage item
 */
class DistanceSensorItem : public std::enable_shared_from_this<DistanceSensorItem>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit DistanceSensorItem(
    DistanceSensorPlugin * owner_, std::string topic_name_,
    YAML::Node config);

private:
  friend class DistanceSensorPlugin;

  // params
  bool is_subscriber;           //!< this item is a subscriber, else is a publisher
  bool send_tf;                 //!< defines if a transform is sent or not
  uint8_t sensor_id;            //!< id of the sensor
  double field_of_view;         //!< FOV of the sensor
  Eigen::Vector3d position;     //!< sensor position
  int orientation;              //!< check orientation of sensor if != -1
  int covariance;               //!< in centimeters, current specification
  std::string frame_id;         //!< frame id for send
  double horizontal_fov_ratio;      //!< horizontal fov ratio for ROS messages
  double vertical_fov_ratio;        //!< vertical fov ratio for ROS messages
  Eigen::Quaternionf quaternion;    //!< Orientation in vehicle body frame for ROTATION_CUSTOM

  // topic handle
  rclcpp::Publisher<Range>::SharedPtr pub;
  rclcpp::Subscription<Range>::SharedPtr sub;
  std::string topic_name;

  DistanceSensorPlugin * owner;

  std::vector<float> data;              //!< array allocation for measurements
  size_t data_index;                    //!< array index

  static constexpr size_t ACC_SIZE = 50;

  /**
   * Calculate measurements variance to send to the FCU.
   */
  float calculate_variance(float range)
  {
    if (data.size() < ACC_SIZE) {
      // limits the size of the array to 50 elements
      data.reserve(ACC_SIZE);
      data.push_back(range);
    } else {
      // it starts rewriting the values from 1st element
      data[data_index] = range;
      if (++data_index > ACC_SIZE - 1) {
        // restarts the index when achieves the last element
        data_index = 0;
      }
    }

    float average, variance, sum = 0, sum_ = 0;

    /*  Compute the sum of all elements */
    for (auto d : data) {
      sum += d;
    }

    average = sum / data.size();

    /*  Compute the variance */
    for (auto d : data) {
      sum_ += (d - average) * (d - average);
    }

    variance = sum_ / data.size();

    return variance;
  }

  //! Copy of Plugin::get_time_boot_ms() because it is private.
  inline uint32_t get_time_boot_ms(const builtin_interfaces::msg::Time & t)
  {
    return rclcpp::Time(t).nanoseconds() / 1000000;
  }

  //! sensor_msgs/Range subscription callback
  void range_cb(const Range::SharedPtr msg);
};


/**
 * @brief Distance sensor plugin
 * @plugin distance_sensor
 *
 * This plugin allows publishing distance sensor data, which is connected to
 * an offboard/companion computer through USB/Serial, to the FCU or vice-versa.
 */
class DistanceSensorPlugin : public plugin::Plugin
{
public:
  explicit DistanceSensorPlugin(plugin::UASPtr uas_)
  : plugin::Plugin(uas_, "distance_sensor")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "base_frame_id", "base_link", [&](const rclcpp::Parameter & p) {
        base_frame_id = p.as_string();
      });
    node_declare_and_watch_parameter(
      "config", "", [&](const rclcpp::Parameter & p) {
        std::unique_lock lock(mutex);

        sensor_map.clear();

        auto lg = get_logger();
        YAML::Node root_node;

        try {
          root_node = YAML::Load(p.as_string());
        } catch (const YAML::ParserException & ex) {
          RCLCPP_ERROR_STREAM(lg, "DS: Failed to parse config: " << ex.what());
          return;
        }

        if (root_node.IsNull()) {
          RCLCPP_INFO(lg, "DS: Plugin not configured!");
          return;
        } else if (!root_node.IsMap()) {
          RCLCPP_ERROR(lg, "DS: Config must be a map.");
          return;
        }

        for (auto it = root_node.begin(); it != root_node.end(); ++it) {
          auto key_s = it->first.as<std::string>();
          RCLCPP_INFO_STREAM(lg, "DS: " << key_s << ": Loading config: " << it->second);

          try {
            auto item = std::make_shared<DistanceSensorItem>(this, key_s, it->second);
            sensor_map[item->sensor_id] = item;
          } catch (const std::exception & ex) {
            RCLCPP_ERROR_STREAM(lg, "DS: " << key_s << ": Failed to load mapping: " << ex.what());
          }
        }
      });
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&DistanceSensorPlugin::handle_distance_sensor),
    };
  }

private:
  friend class DistanceSensorItem;
  using ItemPtr = std::shared_ptr<DistanceSensorItem>;

  std::string base_frame_id;

  std::shared_mutex mutex;
  std::unordered_map<uint8_t, ItemPtr> sensor_map;

  /* -*- low-level send -*- */
  void distance_sensor(
    uint32_t time_boot_ms,
    uint32_t min_distance,
    uint32_t max_distance,
    uint32_t current_distance,
    uint8_t type, uint8_t id,
    uint8_t orientation, uint8_t covariance,
    float horizontal_fov, float vertical_fov,
    std::array<float, 4> quaternion, uint8_t signal_quality)
  {
    mavlink::common::msg::DISTANCE_SENSOR ds = {};

    // [[[cog:
    // for f in ('time_boot_ms',
    //     'min_distance',
    //     'max_distance',
    //     'current_distance',
    //     'type',
    //     'id',
    //     'orientation',
    //     'covariance',
    //     'horizontal_fov',
    //     'vertical_fov',
    //     'quaternion',
    //     'signal_quality'):
    //     cog.outl(f"ds.{f} = {f};")
    // ]]]
    ds.time_boot_ms = time_boot_ms;
    ds.min_distance = min_distance;
    ds.max_distance = max_distance;
    ds.current_distance = current_distance;
    ds.type = type;
    ds.id = id;
    ds.orientation = orientation;
    ds.covariance = covariance;
    ds.horizontal_fov = horizontal_fov;
    ds.vertical_fov = vertical_fov;
    ds.quaternion = quaternion;
    ds.signal_quality = signal_quality;
    // [[[end]]] (checksum: b268a118afee5e2c6cb3e1094a578fff)

    uas->send_message(ds);
  }

  /* -*- mid-level helpers -*- */

  /**
   * Receive distance sensor data from FCU.
   */
  void handle_distance_sensor(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::DISTANCE_SENSOR & dist_sen,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    using mavlink::common::MAV_SENSOR_ORIENTATION;
    using mavlink::common::MAV_DISTANCE_SENSOR;

    std::shared_lock lock(mutex);

    auto lg = get_logger();

    auto it = sensor_map.find(dist_sen.id);
    if (it == sensor_map.end()) {
      RCLCPP_ERROR(
        lg,
        "DS: no mapping for sensor id: %d, type: %d, orientation: %d",
        dist_sen.id, dist_sen.type, dist_sen.orientation);
      return;
    }

    auto sensor = it->second;
    if (sensor->is_subscriber) {
      RCLCPP_ERROR(
        lg,
        "DS: %s (id %d) is subscriber, but i got sensor data for that id from FCU",
        sensor->topic_name.c_str(), sensor->sensor_id);

      return;
    }

    if (sensor->orientation >= 0 && dist_sen.orientation != sensor->orientation) {
      RCLCPP_ERROR(
        lg,
        "DS: %s: received sensor data has different orientation (%s) than in config (%s)!",
        sensor->topic_name.c_str(),
        utils::to_string_enum<MAV_SENSOR_ORIENTATION>(dist_sen.orientation).c_str(),
        utils::to_string_enum<MAV_SENSOR_ORIENTATION>(sensor->orientation).c_str());
      return;
    }

    auto range = Range();

    range.header = uas->synchronized_header(sensor->frame_id, dist_sen.time_boot_ms);

    range.min_range = dist_sen.min_distance * 1E-2;     // in meters
    range.max_range = dist_sen.max_distance * 1E-2;
    range.field_of_view = sensor->field_of_view;

    switch (dist_sen.type) {
      case enum_value(MAV_DISTANCE_SENSOR::LASER):
      case enum_value(MAV_DISTANCE_SENSOR::RADAR):
      case enum_value(MAV_DISTANCE_SENSOR::UNKNOWN):
        range.radiation_type = Range::INFRARED;
        break;
      case enum_value(MAV_DISTANCE_SENSOR::ULTRASOUND):
        range.radiation_type = Range::ULTRASOUND;
        break;
      default:
        RCLCPP_ERROR(
          lg,
          "DS: %s: Wrong/undefined type of sensor (type: %d). Dropping!...",
          sensor->topic_name.c_str(), dist_sen.type);
        return;
    }

    range.range = dist_sen.current_distance * 1E-2;     // in meters

    if (sensor->send_tf) {
      /* variables init */
      Eigen::Quaterniond q;
      if (dist_sen.orientation == enum_value(MAV_SENSOR_ORIENTATION::ROTATION_CUSTOM)) {
        q = ftf::mavlink_to_quaternion(dist_sen.quaternion);
      } else {
        q =
          utils::sensor_orientation_matching(
          static_cast<MAV_SENSOR_ORIENTATION>(dist_sen.orientation));
      }

      geometry_msgs::msg::TransformStamped transform;

      transform.header = uas->synchronized_header(base_frame_id, dist_sen.time_boot_ms);
      transform.child_frame_id = sensor->frame_id;

      /* rotation and position set */
      transform.transform.rotation = tf2::toMsg(q);
      tf2::toMsg(sensor->position, transform.transform.translation);

      /* transform broadcast */
      uas->tf2_broadcaster.sendTransform(transform);
    }

    sensor->pub->publish(range);
  }
};

DistanceSensorItem::DistanceSensorItem(
  DistanceSensorPlugin * owner_, std::string topic_name_,
  YAML::Node config)
: is_subscriber(false),
  send_tf(false),
  sensor_id(0),
  field_of_view(0),
  position(0.0, 0.0, 0.0),
  orientation(-1),
  covariance(0),
  horizontal_fov_ratio(1.0),
  vertical_fov_ratio(1.0),
  quaternion(0.f, 0.f, 0.f, 0.f),
  topic_name(topic_name_),
  owner(owner_),
  data{},
  data_index(0)
{
  using MSO = mavlink::common::MAV_SENSOR_ORIENTATION;
  std::string orientation_str{};

  // load and parse paras
  // first decide the type of topic (sub or pub)
  is_subscriber = config["subscriber"].as<bool>(false);

  // sensor id
  if (auto idn = config["id"]; idn) {
    sensor_id = idn.as<int>();
  } else {
    throw std::invalid_argument("`id` field required");
  }

  // orientation, checks later
  if (auto qn = config["orientation"]; qn) {
    // lookup for numeric value
    orientation_str = qn.as<std::string>();
    orientation = utils::sensor_orientation_from_str(orientation_str);
  } else {
    orientation = -1;                // not set
  }

  if (!is_subscriber) {
    // publisher params
    // frame_id and FOV is required
    frame_id = config["frame_id"].as<std::string>();
    field_of_view = config["field_of_view"].as<double>();

    // unset allowed, setted wrong - not
    if (orientation == -1 && !orientation_str.empty()) {
      throw std::invalid_argument("defined orientation is not valid!");
    }

    // optional
    send_tf = config["send_tf"].as<bool>(false);
    if (auto spn = config["sensor_position"]; spn && send_tf) {
      // sensor position defined if 'send_tf' set to TRUE
      position.x() = spn["x"].as<double>(0.0);
      position.y() = spn["y"].as<double>(0.0);
      position.z() = spn["z"].as<double>(0.0);
    }
  } else {
    // subscriber params
    // orientation is required
    if (orientation_str.empty()) {
      throw std::invalid_argument("`orientation` field required");
    }

    if (orientation == -1) {
      throw std::invalid_argument("defined orientation is not valid!");
    }

    if (orientation == enum_value(MSO::ROTATION_CUSTOM) &&
      !config["custom_orientation"])
    {
      throw std::invalid_argument("`custom_orientation` required for orientation=CUSTOM");
    }


    // optional
    covariance = config["covariance"].as<int>(0);
    horizontal_fov_ratio = config["horizontal_fov_ratio"].as<double>(0.0);
    vertical_fov_ratio = config["vertical_fov_ratio"].as<double>(0.0);
    if (auto con = config["custom_orientation"];
      con && orientation == enum_value(MSO::ROTATION_CUSTOM))
    {
      Eigen::Vector3d rpy;

      rpy.x() = con["roll"].as<double>(0);
      rpy.y() = con["pitch"].as<double>(0);
      rpy.z() = con["yaw"].as<double>(0);

      constexpr auto DEG_TO_RAD = (M_PI / 180.0);
      quaternion = Eigen::Quaternionf(ftf::quaternion_from_rpy(rpy * DEG_TO_RAD));
    }
  }

  // create topic handles
  auto sensor_qos = rclcpp::SensorDataQoS();
  if (!is_subscriber) {
    pub = owner->node->create_publisher<Range>(topic_name, sensor_qos);
  } else {
    sub =
      owner->node->create_subscription<Range>(
      topic_name, sensor_qos,
      std::bind(&DistanceSensorItem::range_cb, this, _1));
  }
}

void DistanceSensorItem::range_cb(const Range::SharedPtr msg)
{
  using mavlink::common::MAV_DISTANCE_SENSOR;

  uint8_t type = 0;
  uint8_t covariance_ = 0;

  if (covariance > 0) {
    covariance_ = covariance;
  } else {
    covariance_ = uint8_t(calculate_variance(msg->range) * 1E2);    // in cm
  }

  // current mapping, may change later
  if (msg->radiation_type == Range::INFRARED) {
    type = enum_value(MAV_DISTANCE_SENSOR::LASER);
  } else if (msg->radiation_type == Range::ULTRASOUND) {
    type = enum_value(MAV_DISTANCE_SENSOR::ULTRASOUND);
  }

  std::array<float, 4> q;
  ftf::quaternion_to_mavlink(quaternion, q);

  owner->distance_sensor(
    get_time_boot_ms(msg->header.stamp),
    msg->min_range / 1E-2,
    msg->max_range / 1E-2,
    msg->range / 1E-2,
    type,
    sensor_id,
    orientation,
    covariance_,
    msg->field_of_view * horizontal_fov_ratio,
    msg->field_of_view * vertical_fov_ratio,
    q,
    0);
}

}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::DistanceSensorPlugin)
