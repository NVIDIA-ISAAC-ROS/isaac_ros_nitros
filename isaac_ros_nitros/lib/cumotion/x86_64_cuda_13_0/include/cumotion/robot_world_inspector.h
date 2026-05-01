// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES.
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
//! @brief Public interface for querying spatial relationships between a robot and a world.

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"
#include "cumotion/robot_description.h"
#include "cumotion/world.h"

namespace cumotion {

//! Interface for querying spatial relationships between a robot and a world.
class CUMO_EXPORT RobotWorldInspector {
 public:
  virtual ~RobotWorldInspector() = default;

  //! Clear the existing world view from `RobotWorldInspector`.
  //!
  //! All `ObstacleHandle`s will be invalid from the perspective of `RobotWorldInspector`.
  //!
  //! `inCollisionWithObstacle()` will always return `false`.
  virtual void clearWorldView() = 0;

  //! Set the internal world view used by `RobotWorldInspector`.
  virtual void setWorldView(const WorldViewHandle &world_view) = 0;

  //! Return the number of world-collision spheres in the robot representation.
  [[nodiscard]] virtual int numWorldCollisionSpheres() const = 0;

  //! Return the number of self-collision spheres in the robot representation.
  [[nodiscard]] virtual int numSelfCollisionSpheres() const = 0;

  //! Return the name of the frame associated with the world-collision sphere corresponding to
  //! `world_collision_sphere_index`.
  //!
  //! A fatal error is logged if:
  //!  1. `world_collision_sphere_index` is greater than or equal to the number of world-collision
  //!     spheres attached to the robot (i.e., `numWorldCollisionSpheres()`), *OR*
  //!  2. `world_collision_sphere_index` is negative.
  [[nodiscard]]
  virtual std::string worldCollisionSphereFrameName(int world_collision_sphere_index) const = 0;

  //! Return the name of the frame associated with the self-collision sphere corresponding to
  //! `self_collision_sphere_index`.
  //!
  //! A fatal error is logged if:
  //!  1. `self_collision_sphere_index` is greater than or equal to the number of self-collision
  //!     spheres attached to the robot (i.e., `numSelfCollisionSpheres()`), *OR*
  //!  2. `self_collision_sphere_index` is negative.
  [[nodiscard]]
  virtual std::string selfCollisionSphereFrameName(int self_collision_sphere_index) const = 0;

  //! Return the radii of all world-collision spheres on the robot.
  //!
  //! Each radius is the "effective radius", representing the sum of the "base radius" of the sphere
  //! and any specified world-collision buffer distance.
  //!
  //! The order of `sphere_radii` corresponds to the order of `sphere_positions` returned by
  //! `worldCollisionSpherePositions()`.
  //!
  //! The length of `sphere_radii` will be set to the number of world-collision spheres specified in
  //! the `RobotDescription`.
  virtual void worldCollisionSphereRadii(std::vector<double> &sphere_radii) const = 0;

  //! Return the radii of all self-collision spheres on the robot.
  //!
  //! Each radius is the "effective radius", representing the sum of the "base radius" of the sphere
  //! and any specified self-collision buffer distance.
  //!
  //! The order of `sphere_radii` corresponds to the order of `sphere_positions` returned by
  //! `selfCollisionSpherePositions()`.
  //!
  //! The length of `sphere_radii` will be set to the number of self-collision spheres specified in
  //! the `RobotDescription`.
  virtual void selfCollisionSphereRadii(std::vector<double> &sphere_radii) const = 0;

  //! Compute positions of all world-collision spheres on the robot at the specified
  //! `cspace_position`.
  //!
  //! The order of `sphere_positions` corresponds to order of `sphere_radii` returned by
  //! `worldCollisionSphereRadii()`.
  //!
  //! If the length of `sphere_positions` is less than the number of world-collision spheres
  //! specified in the `RobotDescription`, `sphere_positions` will be resized to the number of
  //! world-collision spheres. If the length of `sphere_positions` is larger than the number of
  //! world-collision spheres, the vector will NOT be resized. Only the first n elements will be
  //! written to, where n is the number of world-collision spheres.
  virtual void worldCollisionSpherePositions(
      const Eigen::VectorXd &cspace_position,
      std::vector<Eigen::Vector3d> &sphere_positions) const = 0;

  //! Compute positions of all self-collision spheres on the robot at the specified
  //! `cspace_position`.
  //!
  //! The order of `sphere_positions` corresponds to order of `sphere_radii` returned by
  //! `selfCollisionSphereRadii()`.
  //!
  //! If the length of `sphere_positions` is less than the number of self-collision spheres
  //! specified in the `RobotDescription`, `sphere_positions` will be resized to the number of
  //! self-collision spheres. If the length of `sphere_positions` is larger than the number of
  //! self-collision spheres, the vector will NOT be resized. Only the first n elements will be
  //! written to, where n is the number of self-collision spheres.
  virtual void selfCollisionSpherePositions(
      const Eigen::VectorXd &cspace_position,
      std::vector<Eigen::Vector3d> &sphere_positions) const = 0;

  //! Determine whether a specified `cspace_position` puts the robot into collision
  //! with an obstacle.
  //!
  //! If no world view is specified, `false` is returned for any `cspace_position`.
  virtual bool inCollisionWithObstacle(const Eigen::VectorXd &cspace_position) const = 0;

  //! Compute the minimum pair-wise signed distance between the set of robot spheres and the set of
  //! obstacles for the provided position in configuration space.  A positive distance implies that
  //! the obstacle and the robot are *NOT* in collision.
  //!
  //! A fatal error is logged if the `RobotWorldInspector` does not have a world view set.
  //!
  //! NOTE: A distance of zero implies that the closest robot sphere to any obstacle intersects the
  //!       obstacle at a point.
  virtual double minDistanceToObstacle(const Eigen::VectorXd &cspace_position) const = 0;

  //! Compute the signed distance between a given obstacle and collision sphere for the provided
  //! position in configuration space.  A positive distance implies that the obstacle and the sphere
  //! are *NOT* in collision.
  //!
  //! A fatal error is logged if:
  //!  - `world_collision_sphere_index` is greater than or equal to the number of world-collision
  //!    spheres attached to the robot (i.e., `numWorldCollisionSpheres()`), or it is negative.
  //!  - The `RobotWorldInspector` does not have a world view set.
  //!
  //! NOTE: If the world-collision sphere intersects the `obstacle` a distance of zero is returned.
  virtual double distanceToObstacle(const World::ObstacleHandle &obstacle,
                                    int world_collision_sphere_index,
                                    const Eigen::VectorXd &cspace_position) const = 0;

  //! Indicate whether the robot is in self-collision at the given `cspace_position`.
  //!
  //! The robot is considered to be in self-collision if any collision sphere on any frame
  //! intersects a collision sphere from another frame for which collisions have not been masked.
  virtual bool inSelfCollision(const Eigen::VectorXd &cspace_position) const = 0;

  //! Return pairs of frames for which self-collision occurs at the given `cspace_position`.
  //!
  //! Each pair of self-collision frames will be returned once in arbitrary order (e.g., the pair
  //! {"linkA", "linkB"} may be returned *OR* the pair {"linkB", "linkA"} may be returned, but
  //! not both).
  virtual std::vector<std::pair<std::string, std::string>> framesInSelfCollision(
      const Eigen::VectorXd &cspace_position) const = 0;
};

//! Create a `RobotWorldInspector` for a given `robot_description` and `world_view`.
//!
//! The `world_view` is optional. If not provided, queries related to obstacle collisions
//! (e.g., `distanceToObstacle()`) will not be functional unless a world view is set by
//! `setWorldView()` after construction.
CUMO_EXPORT std::unique_ptr<RobotWorldInspector> CreateRobotWorldInspector(
    const RobotDescription &robot_description,
    std::optional<WorldViewHandle> world_view);

}  // namespace cumotion
