// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES.
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
//! @brief Public interface for segmenting out the contribution of a robot from a depth image
//!
//! @note This interface is experimental and may evolve in a future release.  It also lacks
//!       a corresponding Python API.

#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"
#include "cumotion/pose3.h"
#include "cumotion/robot_description.h"
#include "cumotion/vision.h"

namespace cumotion {

//! Interface class for segmenting (masking) out the contribution of a robot from a depth image.
class CUMO_EXPORT RobotSegmenter {
 public:
  enum class RobotGeometryKind {
    SELF_COLLISION_SPHERES,   //!< Use self-collision spheres for robot segmentation.
    WORLD_COLLISION_SPHERES,  //!< Use world-collision spheres for robot segmentation.
  };

  virtual ~RobotSegmenter() {}

  //! For the given `camera_pose_in_robot_frame`, `cspace_position` of the robot, and
  //! `input_depth_image`, compute a filtered `output_depth_image` and/or `output_depth_mask`.
  //!
  //! Both `output_depth_image` and `output_mask` are optional, and passing in `nullptr` for either
  //! will skip the corresponding computation.  A warning will be logged if `nullptr` is passed in
  //! for both.
  //!
  //! If not null, `output_depth_image` will be populated with the same depth values as
  //! `input_depth_image` for all pixels where the corresponding point (in three dimensions) is not
  //! contained within the robot geometry, taking into account the `additional_buffer_distance`
  //! specified during construction of the `RobotSegmenter`.  Depth values for pixels corresponding
  //! to points contained in the robot geometry will be set to zero.
  //!
  //! If not null, `output_mask` will contain a zero for all pixels where the corresponding point
  //! (in three dimensions) is contained within the robot geometry and a value larger than zero for
  //! all other pixels.  The mask is pixel-aligned with `input_depth_image`.  The particular
  //! nonzero value used depends on the scalar type of the `DepthImage`:
  //!
  //! * For a scalar type of `float`, the nonzero value is 1.0.  Note that `meters_per_unit` for
  //!   the `DepthImage` has no effect; it's the "raw" value that's set to 1.0.
  //! * For a scalar type of `uint16_t`, the nonzero value is 65,535 (the maximum representable
  //!   integer).  Again, `meters_per_unit` for the `DepthImage` has no effect.
  //!
  //! `min_depth` and `max_depth` are given in meters and influence the interpretation of depth
  //! values read from the `input_depth_image`.  A depth value outside the range
  //! (`min_depth`, `max_depth`) is assumed to correspond to a point that is not contained within
  //! the robot geometry, and such a depth value will be written to the `output_depth_image`
  //! unchanged.
  //!
  //! The current implementation requires that all depth images have `HOST` or `MANAGED` residency.
  //!
  //! A fatal error will be logged if:
  //!
  //! 1. `output_depth_image` is not null, and its width or height differs from the width or height
  //!    (respectively) of `input_depth_image`.
  //! 2. `output_mask` is not null, and its width or height differs from the width or height
  //!    (respectively) of `input_depth_image`.
  //! 3. `output_depth_image` or `output_mask` is provided as a `DepthImage` with a `const` scalar
  //!    type.
  //! 4. `input_depth_image`, `output_depth_image`, or `output_mask` have `DEVICE` residency.
  virtual void segmentDepthImage(const Pose3 &camera_pose_in_robot_frame,
                                 const Eigen::VectorXd &cspace_position,
                                 const DepthImageBase &input_depth_image,
                                 DepthImageBase *output_depth_image,
                                 DepthImageBase *output_mask,
                                 double min_depth = 0.01,
                                 double max_depth = std::numeric_limits<double>::max()) = 0;
};

//! Create a `RobotSegmenter` from a `robot_description` and `camera_intrinsics`.
//!
//! `camera_intrinsics` must be in the four-parameter "pinhole" format with zero skew.
//!
//! If provided, `additional_buffer_distance` will inflate the effective size of the robot geometry
//! (e.g., collision spheres) such that a point within that distance is considered to be contained
//! within the geometry.  This `additional_buffer_distance` augments rather than replaces any
//! per-link buffer distance specified in the XRDF for the robot.
//!
//! `additional_buffer_distance` may be negative, shrinking the effective size of the robot
//! geometry.  A given part of the geometry (e.g., a collision sphere) will be ignored altogether
//! once `additional_buffer_distance` is sufficiently negative (e.g., negative with a magnitude
//! that exceeds the radius, for the case of a sphere).
//!
//! The currently-supported values for `robot_geometry_kind` are `SELF_COLLISION_SPHERES` and
//! `WORLD_COLLISION_SPHERES`, mapping to the corresponding collision geometry types in XRDF.
//!
//! A fatal error will be logged if the `camera_intrinsics` are not in the four-parameter "pinhole"
//! format (with zero skew).
CUMO_EXPORT std::unique_ptr<RobotSegmenter>
CreateRobotSegmenter(const RobotDescription &robot_description,
                     const CameraIntrinsics &camera_intrinsics,
                     double additional_buffer_distance = 0.0,
                     RobotSegmenter::RobotGeometryKind robot_geometry_kind =
                         RobotSegmenter::RobotGeometryKind::WORLD_COLLISION_SPHERES);

}  // namespace cumotion
