/*
 * Copyright 2015,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Eigen::Quaternion helter functions
 * @file uas_quaternion_utils.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */

#include <mavros/frame_tf.hpp>

namespace mavros
{
namespace ftf
{

/*
 * Note: order of axis are match tf2::LinearMath (bullet).
 * YPR rotation convention -> YAW first, Pitch second, Roll third
 * Compatibility checked by unittests.
 */

Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d & rpy)
{
  // YPR - ZYX
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
  );
}

Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond & q)
{
  // ZYX — atan2-based decomposition avoids Eigen::eulerAngles(2,1,0) clamping
  // yaw to [0, pi]. All three angles now cover their full atan2/asin range:
  //   roll  [-pi, pi], pitch [-pi/2, pi/2], yaw [-pi, pi]
  auto m = q.toRotationMatrix();
  return Eigen::Vector3d(
    std::atan2(m(2, 1), m(2, 2)),   // roll
    std::asin(-m(2, 0)),             // pitch
    std::atan2(m(1, 0), m(0, 0))    // yaw
  );
}

double quaternion_get_yaw(const Eigen::Quaterniond & q)
{
  // to match equation from:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const double & q0 = q.w();
  const double & q1 = q.x();
  const double & q2 = q.y();
  const double & q3 = q.z();

  return std::atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3));
}

}       // namespace ftf
}       // namespace mavros
