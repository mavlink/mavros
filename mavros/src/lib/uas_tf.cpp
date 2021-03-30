/**
 * @brief MAVROS UAS TF part
 * @file uas_tf.cpp
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
#include <mavros/mavros_uas.hpp>


void UAS::add_static_transform(
  const std::string & frame_id, const std::string & child_id,
  const Eigen::Affine3d & tr,
  std::vector<geometry_msgs::msg::TransformStamped> & vector)
{
  // geometry_msgs::TransformStamped static_transform;

  // static_transform.header.stamp = ros::Time::now();
  // static_transform.header.frame_id = frame_id;
  // static_transform.child_frame_id = child_id;
  // tf::transformEigenToMsg(tr, static_transform.transform);

  // vector.emplace_back(static_transform);
}

void UAS::publish_static_transform(
  const std::string & frame_id, const std::string & child_id,
  const Eigen::Affine3d & tr)
{
  // geometry_msgs::TransformStamped static_transformStamped;

  // static_transformStamped.header.stamp = ros::Time::now();
  // static_transformStamped.header.frame_id = frame_id;
  // static_transformStamped.child_frame_id = child_id;
  // tf::transformEigenToMsg(tr, static_transformStamped.transform);

  // tf2_static_broadcaster.sendTransform(static_transformStamped);
}
