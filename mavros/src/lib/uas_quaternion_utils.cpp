/**
 * @brief Eigen::Quaternion helter functions
 * @file uas_quaternion_utils.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <mavros/mavros_uas.h>

using namespace mavros;

/** \geometry_module \ingroup Geometry_Module
  *
  *
  * \returns the Euler-angles of the rotation matrix \c *this using the convention defined by the triplet (\a a0,\a a1,\a a2)
  *
  * Each of the three parameters \a a0,\a a1,\a a2 represents the respective rotation axis as an integer in {0,1,2}.
  * For instance, in:
  * \code Vector3f ea = mat.eulerAngles(2, 0, 2); \endcode
  * "2" represents the z axis and "0" the x axis, etc. The returned angles are such that
  * we have the following equality:
  * \code
  * mat == AngleAxisf(ea[0], Vector3f::UnitZ())
  *      * AngleAxisf(ea[1], Vector3f::UnitX())
  *      * AngleAxisf(ea[2], Vector3f::UnitZ()); \endcode
  * This corresponds to the right-multiply conventions (with right hand side frames).
  *
  * @note This method is the original implementation from Eigen 3.1.4.
  *       In later releases of Eigen the behavior of Eigen's eulerAngles method has been changed considerably.
  *       But we need old behivor when Angles->Quaternion->Matrix->Angles
  *       result in the same angles before and after the conversations.
  *
  * @note Isuue #358
  * @note Example given by mira-project.org.
  */
template<typename Derived>
static inline Eigen::Matrix<typename Eigen::MatrixBase<Derived>::Scalar,3,1>
eulerAngles(
		const Eigen::MatrixBase<Derived> &mat,
		typename Eigen::MatrixBase<Derived>::Index a0,
		typename Eigen::MatrixBase<Derived>::Index a1,
		typename Eigen::MatrixBase<Derived>::Index a2)
{
  /* Implemented from Graphics Gems IV */
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,3)

  // added to use method outside from Eigen::MatrixBase
  using namespace Eigen;
  using Eigen::Matrix;
  using std::atan2;
  using std::sin;
  using std::cos;
  typedef typename MatrixBase<Derived>::Index Index;
  typedef typename MatrixBase<Derived>::Scalar Scalar;

  Matrix<Scalar,3,1> res;
  typedef Matrix<typename Derived::Scalar,2,1> Vector2;
  const Scalar epsilon = NumTraits<Scalar>::dummy_precision();

  const Index odd = ((a0+1)%3 == a1) ? 0 : 1;
  const Index i = a0;
  const Index j = (a0 + 1 + odd)%3;
  const Index k = (a0 + 2 - odd)%3;

  if (a0==a2)
  {
    Scalar s = Vector2(mat.coeff(j,i) , mat.coeff(k,i)).norm();
    res[1] = atan2(s, mat.coeff(i,i));
    if (s > epsilon)
    {
      res[0] = atan2(mat.coeff(j,i), mat.coeff(k,i));
      res[2] = atan2(mat.coeff(i,j),-mat.coeff(i,k));
    }
    else
    {
      res[0] = Scalar(0);
      res[2] = (mat.coeff(i,i)>0?1:-1)*atan2(-mat.coeff(k,j), mat.coeff(j,j));
    }
  }
  else
  {
    Scalar c = Vector2(mat.coeff(i,i) , mat.coeff(i,j)).norm();
    res[1] = atan2(-mat.coeff(i,k), c);
    if (c > epsilon)
    {
      res[0] = atan2(mat.coeff(j,k), mat.coeff(k,k));
      res[2] = atan2(mat.coeff(i,j), mat.coeff(i,i));
    }
    else
    {
      res[0] = Scalar(0);
      res[2] = (mat.coeff(i,k)>0?1:-1)*atan2(-mat.coeff(k,j), mat.coeff(j,j));
    }
  }
  if (!odd)
    res = -res;
  return res;
}

/*
 * Note: order of axis are match tf2::LinearMath (bullet).
 * Compatibility checked by unittests.
 */

Eigen::Quaterniond UAS::quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
	// YPR - ZYX
	return Eigen::Quaterniond(
			Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
			);
}

Eigen::Vector3d UAS::quaternion_to_rpy(const Eigen::Quaterniond &q)
{
	// YPR - ZYX
	return eulerAngles(q.toRotationMatrix(), 2, 1, 0);
}

