/*
 * Copyright 2014,2015,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS UAS TF part
 * @file uas_tf.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <array>
#include <string>
#include <vector>

#include "mavros/mavros_uas.hpp"
#include "tf2_eigen/tf2_eigen.h"

using namespace mavros::uas;  // NOLINT


void UAS::add_static_transform(
  const std::string & frame_id, const std::string & child_id,
  const Eigen::Affine3d & tr,
  std::vector<geometry_msgs::msg::TransformStamped> & vector)
{
  geometry_msgs::msg::TransformStamped static_transform = tf2::eigenToTransform(tr);

  static_transform.header.stamp = this->now();
  static_transform.header.frame_id = frame_id;
  static_transform.child_frame_id = child_id;

  vector.emplace_back(static_transform);
}

void UAS::publish_static_transform(
  const std::string & frame_id, const std::string & child_id,
  const Eigen::Affine3d & tr)
{
  geometry_msgs::msg::TransformStamped static_transform_stamped = tf2::eigenToTransform(tr);

  static_transform_stamped.header.stamp = this->now();
  static_transform_stamped.header.frame_id = frame_id;
  static_transform_stamped.child_frame_id = child_id;

  tf2_static_broadcaster.sendTransform(static_transform_stamped);
}
