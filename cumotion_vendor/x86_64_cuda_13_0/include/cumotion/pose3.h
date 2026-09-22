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
//! @brief Public interface for representing a pose in 3d.

#pragma once

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "cumotion/cumotion_export.h"
#include "cumotion/rotation3.h"

namespace cumotion {

//! Class representing a 3d pose.
struct CUMO_EXPORT Pose3 {
 public:
  //! Default constructor (set `rotation` identity and `translation` to zero).
  Pose3();

  //! Construct pose from a `rotation` and `translation`.
  Pose3(const Rotation3 &rotation, const Eigen::Vector3d &translation);

  //! Construct pose from a 4 x 4 homogeneous transform matrix.
  //!
  //! @note The bottom row of the input matrix is assumed to be `[0, 0, 0, 1]`, but is not
  //! explicitly checked.
  explicit Pose3(const Eigen::Matrix<double, 4, 4> &matrix);

  //! Create a pure rotational pose.
  static Pose3 FromRotation(const Rotation3 &rotation);

  //! Create a pure translational pose.
  static Pose3 FromTranslation(const Eigen::Vector3d &translation);

  //! Create identity pose.
  static Pose3 Identity();

  //! Return matrix representation of the pose.
  [[nodiscard]] Eigen::Matrix4d matrix() const;

  //! Returns the inverse pose.
  [[nodiscard]] Pose3 inverse() const;

  //! Transforms a collection of 3d `vectors` by this pose.
  [[nodiscard]]
  std::vector<Eigen::Vector3d> transformVectors(const std::vector<Eigen::Vector3d> &vectors) const;

  //! Transforms a collection of 3d `vectors` by this pose.
  //!
  //! Each column of `vectors` represents a 3d vector to be transformed.
  [[nodiscard]] Eigen::Matrix<double, 3, Eigen::Dynamic> transformVectors(
      const Eigen::Matrix<double, 3, Eigen::Dynamic> &vectors) const;

  //! Compose poses via * operator.
  friend CUMO_EXPORT Pose3 operator*(const Pose3 &lhs, const Pose3 &rhs);

  //! Transforms a vector by the given pose.
  friend CUMO_EXPORT Eigen::Vector3d operator*(const Pose3 &pose, const Eigen::Vector3d &vector);

  //! Rotation component of pose.
  Rotation3 rotation;

  //! Translation component of pose.
  Eigen::Vector3d translation;
};

}  // namespace cumotion
