// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES.
//                         All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

#pragma once

#include "cumotion/cumotion_export.h"
#include "cumotion/pose3.h"

namespace cumotion {

//! Represent a path through task space (i.e., SE(3) group representing 6-dof poses).
//!
//! This path is parameterized by independent variable `s` and is generally expected to be
//! continuous, but not necessarily smooth.
class CUMO_EXPORT TaskSpacePath {
 public:
  virtual ~TaskSpacePath() = default;

  //! Indicates continuous range of independent values, `s`, for which the path is defined.
  struct CUMO_EXPORT Domain {
    //! Minimum value of `s` defining domain.
    double lower;

    //! Maximum value of `s` defining domain.
    double upper;

    //! Convenience function to return the span of `s` values included in domain.
    [[nodiscard]] double span() const { return upper - lower; }
  };

  //! Return the domain for the path.
  [[nodiscard]] virtual Domain domain() const = 0;

  //! Evaluate the path at the given `s`.
  //!
  //! A fatal error is logged if `s` is outside of the path `domain()`.
  [[nodiscard]] virtual Pose3 eval(double s) const = 0;

  //! Return the total translation distance accumulated along the path.
  //!
  //! This length is not, in general, equal to the translation distance between the poses at the
  //! lower and upper bounds of the domain.
  //!
  //! E.g., if a path moved linearly from (0,0,0) to (0,1,0) to (1,1,0) to (1,0,0), then the path
  //! length would be 3 while the net translation between the positions at the lower (0,0,0) and
  //! upper (1,0,0) bounds of the domain would be 1.
  [[nodiscard]] virtual double pathLength() const = 0;

  //! Return the total angular distance (in radians) accumulated throughout the path.
  //!
  //! Similar to `pathLength()`, this value is not, in general, equal to the minimum angle between
  //! the rotations at the lower and upper bounds of the domain.
  //!
  //! E.g., if a path rotates 2 pi around a single axis followed by pi around another axis, then the
  //! accumulated rotation would be 3 pi, while the net rotation between the rotations at the lower
  //! and upper bounds of the domain would be pi.
  [[nodiscard]] virtual double accumulatedRotation() const = 0;

  //! Return the minimum position of the path within the defined `domain()`.
  //!
  //! NOTE: These minimum position values are not, in general, synchronous. Instead, they represent
  //!       the minimum position independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::Vector3d minPosition() const = 0;

  //! Return the maximum position of the path within the defined `domain()`.
  //!
  //! NOTE: These maximum position values are not, in general, synchronous. Instead, they represent
  //!       the maximum position independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::Vector3d maxPosition() const = 0;
};

}  // namespace cumotion
