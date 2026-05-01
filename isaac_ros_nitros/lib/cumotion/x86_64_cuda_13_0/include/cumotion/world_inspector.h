// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES.
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
//! @brief Public interface for querying spatial relationships in a world.

#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"
#include "cumotion/pose3.h"
#include "cumotion/world.h"

namespace cumotion {

//! Interface for querying properties and spatial relationships within a world.
class CUMO_EXPORT WorldInspector {
 public:
  virtual ~WorldInspector() = default;

  //! Return whether the obstacle specified by `obstacle` is enabled in the current view of the
  //! world.
  [[nodiscard]] virtual bool isEnabled(const World::ObstacleHandle &obstacle) const = 0;

  //! Return the pose of the obstacle specified by `obstacle`.
  [[nodiscard]] virtual Pose3 pose(const World::ObstacleHandle &obstacle) const = 0;

  //! Test whether a sphere defined by `center` and `radius` is in collision with ANY enabled
  //! obstacle in the world.
  [[nodiscard]] virtual bool inCollision(const Eigen::Vector3d &center, double radius) const = 0;

  //! Test whether a sphere defined by `center` and `radius` is in collision with the obstacle
  //! specified by `obstacle`.
  [[nodiscard]] virtual bool inCollision(const World::ObstacleHandle &obstacle,
                                         const Eigen::Vector3d &center,
                                         double radius) const = 0;

  //! Compute the distance from `point` to the obstacle specified by `obstacle`.
  //!
  //! Returns distance between the `point` and `obstacle`. If the `point` intersects `obstacle`, a
  //! negative distance is returned.  The distance gradient is written to `gradient` if provided.
  virtual double distanceTo(const World::ObstacleHandle &obstacle,
                            const Eigen::Vector3d &point,
                            Eigen::Vector3d *gradient = nullptr) const = 0;

  //! Compute distances from `point` to all enabled obstacles.
  //!
  //! Distances between the `point` and each enabled obstacle are written to `distances`.
  //! If the `point` intersects an obstacle, the resulting distance will be negative. The distance
  //! gradients are written to `distance_gradients` if provided.
  //!
  //! The number of `distances` and/or `distance_gradients` that are written (i.e., the number of
  //! enabled obstacles) is returned.
  //!
  //! If the length of `distances` is less than the number of enabled obstacles, it will be resized
  //! to the number of enabled obstacles. The same applies to `distance_gradients` if provided. If
  //! the vectors lengths are larger than the number of enabled obstacles, the vectors will NOT be
  //! resized. Only the first N elements will be written to, where N is the number of enabled
  //! obstacles. The values of extra elements at the end of the input vectors will NOT be changed.
  virtual int distancesTo(const Eigen::Vector3d &point,
                          std::vector<double> &distances,
                          std::vector<Eigen::Vector3d> *distance_gradients = nullptr) const = 0;

  //! Compute distances from `point` to all enabled obstacles.
  //!
  //! Distances between the `point` and each enabled obstacle are written to `distances`, which must
  //! not be `nullptr`.
  //! If the `point` intersects an obstacle, the resulting distance will be negative. The distance
  //! gradients are written to `distance_gradients` if provided.
  //!
  //! The number of `distances` and/or `distance_gradients` that are written (i.e., the number of
  //! enabled obstacles) is returned.
  //!
  //! If the length of `distances` is less than the number of enabled obstacles, it will be resized
  //! to the number of enabled obstacles. The same applies to `distance_gradients` if provided. If
  //! the vectors lengths are larger than the number of enabled obstacles, the vectors will NOT be
  //! resized. Only the first N elements will be written to, where N is the number of enabled
  //! obstacles. The values of extra elements at the end of the input vectors will NOT be changed.
  //!
  //! A fatal error will be logged if `distances` is `nullptr`.
  //!
  //! DEPRECATED: This function overload is deprecated and will be removed in a future version. Use
  //!             the reference overload (`WorldInspector::distancesTo(const Eigen::Vector3d &point,
  //!             std::vector<double> &distances, std::vector<Eigen::Vector3d> *distance_gradients =
  //!             nullptr) const`) instead.
  virtual CUMO_DEPRECATED int distancesTo(
      const Eigen::Vector3d &point,
      std::vector<double> *distances,
      std::vector<Eigen::Vector3d> *distance_gradients = nullptr) const = 0;

  //! Compute the minimum distance from `point` to ANY enabled obstacle in the current
  //! view of the world.
  //!
  //! Returns distance between the `point` and the nearest obstacle. If the `point` is inside an
  //! obstacle, a negative distance is returned. The distance gradient is written to `gradient` if
  //! provided.
  virtual double minDistance(const Eigen::Vector3d &point,
                             Eigen::Vector3d *gradient = nullptr) const = 0;

  //! Return the number of enabled obstacles in the current view of the world.
  [[nodiscard]] virtual int numEnabledObstacles() const = 0;
};

//! Create a `WorldInspector` for a given `world_view`.
CUMO_EXPORT std::unique_ptr<WorldInspector> CreateWorldInspector(const WorldViewHandle &world_view);

}  // namespace cumotion
