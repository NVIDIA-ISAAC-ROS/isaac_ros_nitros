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

#include <memory>
#include <vector>

#include "cumotion/cspace_path.h"
#include "cumotion/cspace_path_spec.h"
#include "cumotion/cumotion_export.h"

namespace cumotion {

//! Represent a path linearly interpolated through configuration space (i.e., c-space) waypoints.
//!
//! This path is parameterized by independent variable `s` and will be continuous but not
//! (in general) smooth.
class CUMO_EXPORT LinearCSpacePath : public CSpacePath {
 public:
  virtual ~LinearCSpacePath() = default;

  //! Return the number of configuration space coordinates for the path.
  [[nodiscard]] int numCSpaceCoords() const override = 0;

  //! Return the domain for the path.
  [[nodiscard]] Domain domain() const override = 0;

  //! Evaluate the path at the given `s`.
  //!
  //! A fatal error is logged if `s` is outside of the path `domain()`.
  [[nodiscard]] Eigen::VectorXd eval(double s) const override = 0;

  //! Return the total distance accumulated along the path, where distance is computed using the
  //! L2-norm.
  //!
  //! This length is not, in general, equal to the distance between the configurations at the lower
  //! and upper bounds of the domain.
  [[nodiscard]] double pathLength() const override = 0;

  //! Return the minimum position of the path within the defined `domain()`.
  //!
  //! NOTE: These minimum position values are not, in general, synchronous. Instead, they represent
  //!       the minimum position independently achieved by each coordinate.
  [[nodiscard]] Eigen::VectorXd minPosition() const override = 0;

  //! Return the maximum position of the path within the defined `domain()`.
  //!
  //! NOTE: These maximum position values are not, in general, synchronous. Instead, they represent
  //!       the maximum position independently achieved by each coordinate.
  [[nodiscard]] Eigen::VectorXd maxPosition() const override = 0;

  //! Return all of the waypoints through which the path is linearly interpolated (including the
  //! initial and final c-space configurations).
  [[nodiscard]] virtual std::vector<Eigen::VectorXd> waypoints() const = 0;
};

//! Create a `LinearCSpacePath` from a c-space path specification.
CUMO_EXPORT std::unique_ptr<LinearCSpacePath> CreateLinearCSpacePath(
    const CSpacePathSpec &cspace_path_spec);

}  // namespace cumotion
