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

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"

namespace cumotion {

//! The `CSpacePathSpec` is used to procedurally specify a `CSpacePath` from a series of
//! configuration space points.
class CUMO_EXPORT CSpacePathSpec {
 public:
  virtual ~CSpacePathSpec() = default;

  //! Return the number of configuration space coordinates for the path specification.
  [[nodiscard]] virtual int numCSpaceCoords() const = 0;

  //! Add a path to connect the previous configuration to the `waypoint`.
  //!
  //! The `waypoint` must have dimension equal to `numCSpaceCoords()` or it will be discarded.
  //!
  //! If path segment is successfully added, `true` is returned. Else, `false` is returned and an
  //! error is logged.
  virtual bool addCSpaceWaypoint(const Eigen::VectorXd &waypoint) = 0;
};

//! Create a `CSpacePathSpec` with the specified `initial_cspace_position`.
CUMO_EXPORT std::unique_ptr<CSpacePathSpec> CreateCSpacePathSpec(
    const Eigen::VectorXd &initial_cspace_position);

}  // namespace cumotion
