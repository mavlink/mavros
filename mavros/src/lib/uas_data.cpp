/**
 * @brief MAVROS UAS manager (data part)
 * @file uas_data.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014,2015,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <array>
#include <unordered_map>
#include <stdexcept>
#include <mavros/mavros_uas.hpp>
#include <mavros/utils.hpp>
#include <mavros/px4_custom_mode.hpp>

using namespace mavros::uas;  // NOLINT

std::once_flag Data::init_flag;
std::shared_ptr<GeographicLib::Geoid> Data::egm96_5;


Data::Data()
:   imu_enu_data{},
  imu_ned_data{},
  gps_fix{},
  gps_eph(NAN),
  gps_epv(NAN),
  gps_fix_type(0),
  gps_satellites_visible(0)
{
  std::call_once(init_flag, init_geographiclib);

  // Publish helper TFs used for frame transformation in the odometry plugin
  // std::vector<geometry_msgs::TransformStamped> transform_vector;
  // add_static_transform(
  //   "map", "map_ned", Eigen::Affine3d(
  //     ftf::quaternion_from_rpy(
  //       M_PI, 0,
  //       M_PI_2)),
  //   transform_vector);
  // add_static_transform(
  //   "odom", "odom_ned", Eigen::Affine3d(
  //     ftf::quaternion_from_rpy(
  //       M_PI, 0,
  //       M_PI_2)),
  //   transform_vector);
  // add_static_transform(
  //   "base_link", "base_link_frd",
  //   Eigen::Affine3d(ftf::quaternion_from_rpy(M_PI, 0, 0)), transform_vector);

  // tf2_static_broadcaster.sendTransform(transform_vector);
}

void Data::init_geographiclib()
{
  try {
    // Using smallest dataset with 5' grid,
    // From default location,
    // Use cubic interpolation, Thread safe
    egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);
  } catch (const std::exception & e) {
    rcpputils::require_true(
      false, utils::format(
        "UAS: GeographicLib exception: %s "
        "| Run install_geographiclib_dataset.sh script in order to install Geoid Model dataset!",
        e.what()));
  }
}

/* -*- IMU data -*- */

void Data::update_attitude_imu_enu(const sensor_msgs::msg::Imu & imu)
{
  s_unique_lock lock(mu);
  imu_enu_data = imu;
}

void Data::update_attitude_imu_ned(const sensor_msgs::msg::Imu & imu)
{
  s_unique_lock lock(mu);
  imu_ned_data = imu;
}

sensor_msgs::msg::Imu Data::get_attitude_imu_enu()
{
  s_shared_lock lock(mu);
  return imu_enu_data;
}

sensor_msgs::msg::Imu Data::get_attitude_imu_ned()
{
  s_shared_lock lock(mu);
  return imu_ned_data;
}

geometry_msgs::msg::Quaternion Data::get_attitude_orientation_enu()
{
  s_shared_lock lock(mu);

  //if (imu_enu_data) {
  return imu_enu_data.orientation;
  //} else {
  //  // fallback - return identity
  //  geometry_msgs::Quaternion q;
  //  q.w = 1.0; q.x = q.y = q.z = 0.0;
  //  return q;
  //}
}

geometry_msgs::msg::Quaternion Data::get_attitude_orientation_ned()
{
  s_shared_lock lock(mu);
  //if (imu_ned_data) {
  return imu_ned_data.orientation;
  //} else {
  //  // fallback - return identity
  //  geometry_msgs::Quaternion q;
  //  q.w = 1.0; q.x = q.y = q.z = 0.0;
  //  return q;
  //}
}

geometry_msgs::msg::Vector3 Data::get_attitude_angular_velocity_enu()
{
  s_shared_lock lock(mu);
  //if (imu_enu_data) {
  return imu_enu_data.angular_velocity;
  //} else {
  //  // fallback
  //  geometry_msgs::Vector3 v;
  //  v.x = v.y = v.z = 0.0;
  //  return v;
  //}
}

geometry_msgs::msg::Vector3 Data::get_attitude_angular_velocity_ned()
{
  s_shared_lock lock(mu);
  //if (imu_ned_data) {
  return imu_ned_data.angular_velocity;
  //} else {
  //  // fallback
  //  geometry_msgs::Vector3 v;
  //  v.x = v.y = v.z = 0.0;
  //  return v;
  //}
}

/* -*- GPS data -*- */

void Data::update_gps_fix_epts(
  const sensor_msgs::msg::NavSatFix & fix,
  float eph, float epv,
  int fix_type, int satellites_visible)
{
  s_unique_lock lock(mu);

  gps_fix = fix;
  gps_eph = eph;
  gps_epv = epv;
  gps_fix_type = fix_type;
  gps_satellites_visible = satellites_visible;
}

//! Returns EPH, EPV, Fix type and satellites visible
void Data::get_gps_epts(float & eph, float & epv, int & fix_type, int & satellites_visible)
{
  s_shared_lock lock(mu);

  eph = gps_eph;
  epv = gps_epv;
  fix_type = gps_fix_type;
  satellites_visible = gps_satellites_visible;
}

//! Retunrs last GPS RAW message
sensor_msgs::msg::NavSatFix Data::get_gps_fix()
{
  s_shared_lock lock(mu);
  return gps_fix;
}
