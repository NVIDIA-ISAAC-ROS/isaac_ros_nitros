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

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"

namespace cumotion {

//! Represent a path through configuration space (i.e., c-space or "joint space").
//!
//! This path is parameterized by independent variable `s` and is generally expected to be
//! continuous, but not necessarily smooth.
class CUMO_EXPORT CSpacePath {
 public:
  virtual ~CSpacePath() = default;

  //! Return the number of configuration space coordinates for the path.
  virtual int numCSpaceCoords() const = 0;

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
  [[nodiscard]] virtual Eigen::VectorXd eval(double s) const = 0;

  //! Return the total distance accumulated along the path, where distance is computed using the
  //! L2-norm.
  //!
  //! This length is not, in general, equal to the distance between the configurations at the lower
  //! and upper bounds of the domain.
  [[nodiscard]] virtual double pathLength() const = 0;

  //! Return the minimum position of the path within the defined `domain()`.
  //!
  //! NOTE: These minimum position values are not, in general, synchronous. Instead, they represent
  //!       the minimum position independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::VectorXd minPosition() const = 0;

  //! Return the maximum position of the path within the defined `domain()`.
  //!
  //! NOTE: These maximum position values are not, in general, synchronous. Instead, they represent
  //!       the maximum position independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::VectorXd maxPosition() const = 0;
};

}  // namespace cumotion
