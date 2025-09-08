/**
 * @brief Sensor orientation helper function
 * @file enum_sensor_orientation.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <string>
#include <utility>
#include <unordered_map>

#include "mavros/utils.hpp"
#include "mavros/frame_tf.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mavros
{
namespace utils
{

using mavlink::common::MAV_SENSOR_ORIENTATION;

// internal type: name - rotation
using OrientationPair = std::pair<const std::string, const Eigen::Quaterniond>;

static auto logger = rclcpp::get_logger("uas.enum");

// internal data initializer
static const OrientationPair make_orientation(
  const std::string & name,
  const double roll,
  const double pitch,
  const double yaw)
{
  constexpr auto DEG_TO_RAD = (M_PI / 180.0);
  const Eigen::Quaterniond rot = ftf::quaternion_from_rpy(
    Eigen::Vector3d(
      roll, pitch,
      yaw) * DEG_TO_RAD);
  return std::make_pair(name, rot);
}

// [[[cog:
// import attr
// import pymavlink.dialects.v20.common as common
// ename = 'MAV_SENSOR_ORIENTATION'
// pfx2 = 'MAV_SENSOR_ROTATION_'
//
// enum = sorted(common.enums[ename].items())
// enum.pop() # remove ENUM_END
//
// @attr.s(auto_attribs=True)
// class Vector3:
//     Roll: float = 0.0
//     Pitch: float = 0.0
//     Yaw: float = 0.0
//
//     @classmethod
//     def parse_rpy(cls, desc):
//         try:
//             pairs = {
//                 f.strip(): float(v)
//                 for f, v in [v.split(":") for v in desc.split(',')]
//             }
//             return cls(**pairs)
//         except Exception as ex:
//             cog.msg(f"Parse Error: {ex}, desc: {desc}")
//             return cls()
//
//     def __str__(self):
//         return f"{self.Roll}, {self.Pitch}, {self.Yaw}"
//
// cog.outl(
//     f"static const std::unordered_map<typename std::underlying_type<{ename}>::type,\n"
//     f"  const OrientationPair> sensor_orientations{{{{")
// for k, e in enum:
//     name_short = e.name[len(pfx2):]
//     vec = Vector3.parse_rpy(e.description)
//     cog.outl(f"""  {{{k}, make_orientation("{name_short}", {vec})}},""")
//
// cog.outl("}};")
// ]]]
static const std::unordered_map<typename std::underlying_type<MAV_SENSOR_ORIENTATION>::type,
  const OrientationPair> sensor_orientations{{
  {0, make_orientation("NONE", 0.0, 0.0, 0.0)},
  {1, make_orientation("YAW_45", 0.0, 0.0, 45.0)},
  {2, make_orientation("YAW_90", 0.0, 0.0, 90.0)},
  {3, make_orientation("YAW_135", 0.0, 0.0, 135.0)},
  {4, make_orientation("YAW_180", 0.0, 0.0, 180.0)},
  {5, make_orientation("YAW_225", 0.0, 0.0, 225.0)},
  {6, make_orientation("YAW_270", 0.0, 0.0, 270.0)},
  {7, make_orientation("YAW_315", 0.0, 0.0, 315.0)},
  {8, make_orientation("ROLL_180", 180.0, 0.0, 0.0)},
  {9, make_orientation("ROLL_180_YAW_45", 180.0, 0.0, 45.0)},
  {10, make_orientation("ROLL_180_YAW_90", 180.0, 0.0, 90.0)},
  {11, make_orientation("ROLL_180_YAW_135", 180.0, 0.0, 135.0)},
  {12, make_orientation("PITCH_180", 0.0, 180.0, 0.0)},
  {13, make_orientation("ROLL_180_YAW_225", 180.0, 0.0, 225.0)},
  {14, make_orientation("ROLL_180_YAW_270", 180.0, 0.0, 270.0)},
  {15, make_orientation("ROLL_180_YAW_315", 180.0, 0.0, 315.0)},
  {16, make_orientation("ROLL_90", 90.0, 0.0, 0.0)},
  {17, make_orientation("ROLL_90_YAW_45", 90.0, 0.0, 45.0)},
  {18, make_orientation("ROLL_90_YAW_90", 90.0, 0.0, 90.0)},
  {19, make_orientation("ROLL_90_YAW_135", 90.0, 0.0, 135.0)},
  {20, make_orientation("ROLL_270", 270.0, 0.0, 0.0)},
  {21, make_orientation("ROLL_270_YAW_45", 270.0, 0.0, 45.0)},
  {22, make_orientation("ROLL_270_YAW_90", 270.0, 0.0, 90.0)},
  {23, make_orientation("ROLL_270_YAW_135", 270.0, 0.0, 135.0)},
  {24, make_orientation("PITCH_90", 0.0, 90.0, 0.0)},
  {25, make_orientation("PITCH_270", 0.0, 270.0, 0.0)},
  {26, make_orientation("PITCH_180_YAW_90", 0.0, 180.0, 90.0)},
  {27, make_orientation("PITCH_180_YAW_270", 0.0, 180.0, 270.0)},
  {28, make_orientation("ROLL_90_PITCH_90", 90.0, 90.0, 0.0)},
  {29, make_orientation("ROLL_180_PITCH_90", 180.0, 90.0, 0.0)},
  {30, make_orientation("ROLL_270_PITCH_90", 270.0, 90.0, 0.0)},
  {31, make_orientation("ROLL_90_PITCH_180", 90.0, 180.0, 0.0)},
  {32, make_orientation("ROLL_270_PITCH_180", 270.0, 180.0, 0.0)},
  {33, make_orientation("ROLL_90_PITCH_270", 90.0, 270.0, 0.0)},
  {34, make_orientation("ROLL_180_PITCH_270", 180.0, 270.0, 0.0)},
  {35, make_orientation("ROLL_270_PITCH_270", 270.0, 270.0, 0.0)},
  {36, make_orientation("ROLL_90_PITCH_180_YAW_90", 90.0, 180.0, 90.0)},
  {37, make_orientation("ROLL_90_YAW_270", 90.0, 0.0, 270.0)},
  {38, make_orientation("ROLL_90_PITCH_68_YAW_293", 90.0, 68.0, 293.0)},
  {39, make_orientation("PITCH_315", 0.0, 315.0, 0.0)},
  {40, make_orientation("ROLL_90_PITCH_315", 90.0, 315.0, 0.0)},
  {100, make_orientation("CUSTOM", 0.0, 0.0, 0.0)},
}};
// [[[end]]] (sum: 3beyExWw8G)

std::string to_string(MAV_SENSOR_ORIENTATION orientation)
{
  const auto idx = enum_value(orientation);
  auto it = sensor_orientations.find(idx);

  if (it == sensor_orientations.end()) {
    RCLCPP_ERROR(logger, "SENSOR: wrong orientation index: %d", idx);
    return std::to_string(idx);
  }

  return it->second.first;
}

Eigen::Quaterniond sensor_orientation_matching(MAV_SENSOR_ORIENTATION orientation)
{
  const auto idx = enum_value(orientation);
  auto it = sensor_orientations.find(idx);

  if (it == sensor_orientations.end()) {
    RCLCPP_ERROR(logger, "SENSOR: wrong orientation index: %d", idx);
    return Eigen::Quaterniond::Identity();
  }

  return it->second.second;
}

int sensor_orientation_from_str(const std::string & sensor_orientation)
{
  // XXX bsearch

  // 1. try to find by name
  for (const auto & kv : sensor_orientations) {
    if (kv.second.first == sensor_orientation) {
      return kv.first;
    }
  }

  // 2. try convert integer
  // fallback for old configs that uses numeric orientation.
  try {
    int idx = std::stoi(sensor_orientation, 0, 0);
    if (0 > idx || sensor_orientations.find(idx) == sensor_orientations.end()) {
      RCLCPP_ERROR(logger, "SENSOR: orientation index out of bound: %d", idx);
      return -1;
    } else {
      return idx;
    }
  } catch (std::invalid_argument & ex) {
    // failed
  }

  RCLCPP_ERROR_STREAM(logger, "SENSOR: wrong orientation str: " << sensor_orientation);

  return -1;
}

}       // namespace utils
}       // namespace mavros
