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

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "cumotion/cumotion_export.h"
#include "cumotion/obstacle.h"
#include "cumotion/pose3.h"

namespace cumotion {

//! Forward declaration of `WorldViewHandle` for use by `World::addWorldView()`.
struct WorldViewHandle;

//! World represents a collection of obstacles.
class CUMO_EXPORT World {
 public:
  //! Opaque handle to an obstacle.
  struct CUMO_EXPORT ObstacleHandle {
    struct Impl;
    std::shared_ptr<Impl> impl;
  };

  virtual ~World() = default;

  //! Add `obstacle` to the world.
  //!
  //! All attributes of obstacle are copied to world and subsequent changes to `obstacle` will
  //! not be reflected in the world.
  //!
  //! If a `pose` is not provided for the `obstacle`, `Pose3::Identity()` will be used.
  //!
  //! Obstacles are automatically enabled when added.
  virtual ObstacleHandle addObstacle(const Obstacle &obstacle,
                                     std::optional<Pose3> pose = std::nullopt) = 0;

  //! Permanently remove obstacle, invalidating its handle.
  virtual void removeObstacle(const ObstacleHandle &obstacle) = 0;

  //! Enable an obstacle for the purpose of collision checks and distance evaluations.
  virtual void enableObstacle(const ObstacleHandle &obstacle) = 0;

  //! Disable an obstacle for the purpose of collision checks and distance evaluations.
  virtual void disableObstacle(const ObstacleHandle &obstacle) = 0;

  //! Set the pose of the given obstacle.
  virtual void setPose(const ObstacleHandle &obstacle, const Pose3 &pose) = 0;

  //! Set the grid values for an obstacle of type SDF using a host-resident `values` buffer.
  //!
  //! It is assumed that `values` is stored with the `z` index varying fastest and has dimensions
  //! given by the `Obstacle::Grid` associated with `obstacle`.  For example, for an obstacle with
  //! `Grid` parameters `num_voxels_x`, `num_voxels_y`, and `num_voxels_z`, the length of `values`
  //! should be `num_voxels_x * num_voxels_y * num_voxels_z`, and in the provided coordinates,
  //! adjacent elements in memory should correspond to voxels with adjacent Z coordinates, and
  //! voxels with adjacent X coordinates should be separated by `num_voxels_y * num_voxels_z`
  //! elements.
  //!
  //! `precision` specifies the floating-point type of `values`.  `Obstacle::Grid::Precision::HALF`
  //! corresponds to the `__half` data type defined in the `cuda_fp16.h` header.
  //!
  //! If the type of `obstacle` is not `Obstacle::Type::SDF`, a fatal error will be logged.
  virtual void setSdfGridValuesFromHost(const ObstacleHandle &obstacle,
                                        const void *values,
                                        Obstacle::Grid::Precision grid_precision) = 0;

  //! Set the grid values for an obstacle of type SDF using a device-resident buffer `values`.
  //!
  //! It is assumed that `values` is stored with the `z` index varying fastest and has dimensions
  //! given by the `Obstacle::Grid` associated with `obstacle`.  For example, for an obstacle with
  //! `Grid` parameters `num_voxels_x`, `num_voxels_y`, and `num_voxels_z`, the length of `values`
  //! should be `num_voxels_x * num_voxels_y * num_voxels_z`, and in the provided coordinates,
  //! adjacent elements in memory should correspond to voxels with adjacent Z coordinates, and
  //! voxels with adjacent X coordinates should be separated by `num_voxels_y * num_voxels_z`
  //! elements.
  //!
  //! `precision` specifies the floating-point type of `values`.  `Obstacle::Grid::Precision::HALF`
  //! corresponds to the `__half` data type defined in the `cuda_fp16.h` header.
  //!
  //! If the type of `obstacle` is not `Obstacle::Type::SDF`, a fatal error will be logged.
  virtual void setSdfGridValuesFromDevice(const ObstacleHandle &obstacle,
                                          const void *values,
                                          Obstacle::Grid::Precision grid_precision) = 0;

  //! Tolerances used when inspecting an SDF.
  //!
  //! Accumulation of numerical error from type-casting is handled independently of the tolerances
  //! in `SdfInspectionTolerances`.  It is expected that any SDF computed from an analytically
  //! correct distance function will pass `InspectCudaSdfAndSync()` with the default tolerances of
  //! `0.0f`.
  //!
  //! Inspection tolerances are specified in single-precision (`float`) because only single- and
  //! half- precision data for device-resident SDFs are supported.
  struct CUMO_EXPORT SdfInspectionTolerances {
    //! Numerical tolerance on checking whether an SDF voxel value is equal to its neighbor.
    //!
    //! The effective tolerance used is the combination of this tolerance and an internally computed
    //! `numerical_tolerance`.  `numerical_tolerance` is a derived upper bound on accumulated error
    //! under the assumption that SDF data was computed analytically with infinite precision, then
    //! cast to the `SDF` data-type specified by `Obstacle::Grid::device_precision`.
    //!
    //! A voxel is said to be equal to its neighbor if:
    //! `abs(voxel_value - neighbor_value) <= voxel_matches_neighbor_tolerance +
    //!                                       numerical_tolerance`.
    float voxel_matches_neighbor_tolerance = 0.0f;

    //! Numerical tolerance on checking whether an SDF voxel value is too far from a neighboring
    //! voxel.
    //!
    //! The effective tolerance used is the combination of this tolerance and an internally computed
    //! `numerical_tolerance`.  `numerical_tolerance` is a derived upper bound on accumulated error
    //! under the assumption that SDF data was computed analytically with infinite precision, then
    //! cast to the `SDF` data-type specified by `Obstacle::Grid::device_precision`.
    //!
    //! A voxel is said to be too far from its neighbor if:
    //! `abs(voxel_value - neighbor_value) > voxel_too_far_from_neighbor_tolerance +
    //!                                      numerical_tolerance`.
    float voxel_too_far_from_neighbor_tolerance = 0.0f;
  };

  //! Inspection results associated with an SDF.
  //!
  //! When `numErrors() == 0`, this does *NOT* guarantee that the SDF is globally valid.  It only
  //! shows that none of the specific error cases covered by the inspector were present.
  struct CUMO_EXPORT SdfInspectionResults {
    //! This count will be set to the number of voxels whose values match the values of `6`
    //! neighboring voxels within the specified `voxel_matches_neighbor_tolerance`.  This can only
    //! be triggered by non-boundary voxels, as boundary voxels do not have `6` neighbors.
    //!
    //! There is no geometrically consistent way this can happen in a valid distance field without
    //! voxel-scale obstacles.
    int num_voxels_matching_all_neighbors = 0;

    //! This count will be set to the number of voxels whose distance to a neighboring voxel is
    //! greater than the `voxel_size` parameter of the SDF inflated by the specified
    //! `voxel_too_far_from_neighbor_tolerance`.
    //!
    //! There is no geometrically consistent way this can happen in a valid distance field.
    int num_voxels_too_far_from_neighbors = 0;

    //! Returns the sum of all error conditions in `SdfInspectionResults`.
    [[nodiscard]] int numErrors() const {
      return num_voxels_matching_all_neighbors + num_voxels_too_far_from_neighbors;
    }
  };

  //! Inspect the data for an obstacle of type `SDF`.
  //!
  //! The returned `SdfInspectionResults` reports error-cases in the SDF data that are likely to
  //! cause issues such as undetected collision with the environment.
  //!
  //! The default `inspection_tolerances` are recommended, and only need to be replaced if there is
  //! known systematic error in the generation of SDF data within known bounds.
  //!
  //! A fatal error will be logged if:
  //!  - `obstacle` does not have type `SDF`.
  //!  - `obstacle` has not been populated with data.
  //!  - `obstacle` has been removed from the world.
  [[nodiscard]] virtual SdfInspectionResults inspectSdf(
      const ObstacleHandle &obstacle,
      std::optional<SdfInspectionTolerances> inspection_tolerances = std::nullopt) const = 0;

  //! Create a view into the world that can be used for collision checks and distance evaluations.
  //!
  //! Each world view will maintain a static view of the world until it is updated. When a
  //! world view is updated, it will reflect any changes to the world since its last update.
  virtual WorldViewHandle addWorldView() = 0;
};

//! Create an empty world with no obstacles.
CUMO_EXPORT std::shared_ptr<World> CreateWorld();

//! A handle to a view of a `cumotion::World`.
//!
//! This view can be independently updated to track updates made to a `cumotion::World` object.
//! A `WorldViewHandle` may be copied, with all copies sharing the same underlying view.
//!
//! To query spatial relationships in a world view, use `cumotion::WorldInspector`.
struct CUMO_EXPORT WorldViewHandle {
  //! Update world view such that any changes to the underlying world are reflected in this view.
  void update();

  struct Impl;
  std::shared_ptr<Impl> impl;
};

}  // namespace cumotion
