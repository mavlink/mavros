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
  // ZYX decomposition via rotation matrix. Avoids Eigen::eulerAngles(2,1,0)
  // clamping yaw to [0, pi].
  //
  // Normal case (|pitch| < pi/2): roll, pitch, yaw all recovered via atan2/asin.
  // Gimbal lock (pitch = ±pi/2): roll and yaw are not independently observable;
  // we set roll = 0 and place the combined degree of freedom in yaw.
  auto m = q.toRotationMatrix();

  double sin_pitch = std::clamp(-m(2, 0), -1.0, 1.0);
  double cos_pitch = std::sqrt(m(2, 1) * m(2, 1) + m(2, 2) * m(2, 2));

  if (cos_pitch > 1e-9) {
    return Eigen::Vector3d(
      std::atan2(m(2, 1), m(2, 2)),       // roll  ∈ (-π, π]
      std::atan2(sin_pitch, cos_pitch),   // pitch ∈ (-π/2, π/2)
      std::atan2(m(1, 0), m(0, 0)));      // yaw   ∈ (-π, π]
  }

  // Gimbal lock
  if (sin_pitch > 0) {
    // pitch = +π/2: only (yaw − roll) is observable
    return Eigen::Vector3d(0.0, M_PI / 2.0, std::atan2(m(1, 2), m(1, 1)));
  }
  // pitch = -π/2: only (yaw + roll) is observable
  return Eigen::Vector3d(0.0, -M_PI / 2.0, std::atan2(-m(1, 2), m(1, 1)));
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
