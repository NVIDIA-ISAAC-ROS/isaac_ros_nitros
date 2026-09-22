// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES.
//                         All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

//! @file
//! @brief Public interface for representing a rotation in 3d.

#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "cumotion/cumotion_export.h"

namespace cumotion {

//! Class representing a 3d rotation.
class CUMO_EXPORT Rotation3 {
 public:
  //! Construct rotation from a quaternion.
  //!
  //! By default, the `quaternion` will be normalized, but the argument `skip_normalization` can be
  //! set to `true` to skip this normalization. This should only be done if the `quaternion` is
  //! known to be normalized; otherwise, `Rotation3` operations will not work as expected.
  explicit Rotation3(const Eigen::Quaterniond &quaternion, bool skip_normalization = false);
  Rotation3(double w, double x, double y, double z);

  //! Return a rotation that rotates by an angle (in radians) around a given axis.
  //!
  //! By default, the `axis` will be normalized (and if the axis norm is near zero, an identity
  //! rotation will be constructed). The argument `skip_normalization` can set to `true` to skip
  //! this normalization. This should only be done if the `axis` is known to be normalized;
  //! otherwise, `Rotation3` may represent an invalid rotation.
  static Rotation3 FromAxisAngle(const Eigen::Vector3d &axis,
                                 double angle,
                                 bool skip_normalization = false);

  //! Return a rotation converted from a scaled axis.
  //!
  //! The magnitude of `scaled_axis` specifies the rotation angle in radians and the direction
  //! of `scaled_axis` specifies the axis of rotation.
  static Rotation3 FromScaledAxis(const Eigen::Vector3d &scaled_axis);

  //! Return a rotation converted from a 3 x 3 rotation matrix.
  //!
  //! Internally, the `rotation_matrix` will be converted to a `Eigen::Quaterniond`. By default,
  //! this quaternion will be normalized, but the argument `skip_normalization` can be
  //! set to `true` to skip this normalization. This should only be done if the `rotation_matrix` is
  //! known to be SO3; otherwise, `Rotation3` operations will not work as expected.
  static Rotation3 FromMatrix(const Eigen::Matrix3d &rotation_matrix,
                              bool skip_normalization = false);

  //! Create identity rotation.
  static Rotation3 Identity();

  //! Return quaternion representation of the rotation. Returned quaternion is always normalized.
  inline Eigen::Quaterniond quaternion() const { return quaternion_; }

  //! Return `w` component of the quaternion representation of the rotation.
  [[nodiscard]] double w() const { return quaternion_.w(); }

  //! Return `x` component of the quaternion representation of the rotation.
  [[nodiscard]] double x() const { return quaternion_.x(); }

  //! Return `y` component of the quaternion representation of the rotation.
  [[nodiscard]] double y() const { return quaternion_.y(); }

  //! Return `z` component of the quaternion representation of the rotation.
  [[nodiscard]] double z() const { return quaternion_.z(); }

  //! Return matrix representation of the rotation.
  [[nodiscard]] Eigen::Matrix3d matrix() const;

  //! Return scaled axis representation of the rotation where magnitude of the returned vector
  //! represents the rotation angle in radians.
  [[nodiscard]] Eigen::Vector3d scaledAxis() const;

  //! Returns the inverse rotation.
  [[nodiscard]] Rotation3 inverse() const;

  //! Compute the minimum angular distance (in radians) between two rotations.
  static double Distance(const Rotation3 &rotation0, const Rotation3 &rotation1);

  //! Smoothly interpolate between two rotations using spherical linear interpolation ("slerp").
  //!
  //! Intermediate rotations will follow a geodesic between `rotation0` and `rotation1` when
  //! represented as quaternions on a unit sphere.
  //!
  //! The parameter `t` must be in range [0, 1], with 0 corresponding to `rotation0` and 1
  //! corresponding to `rotation1`.
  static Rotation3 Slerp(const Rotation3 &rotation0, const Rotation3 &rotation1, double t);

  //! Compose rotations via * operator.
  friend CUMO_EXPORT Rotation3 operator*(const Rotation3 &lhs, const Rotation3 &rhs);

  //! Rotates a vector by the given rotation.
  friend CUMO_EXPORT Eigen::Vector3d operator*(const Rotation3 &rotation,
                                               const Eigen::Vector3d &vector);

  //! See https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Quaterniond quaternion_;
};

}  // namespace cumotion
