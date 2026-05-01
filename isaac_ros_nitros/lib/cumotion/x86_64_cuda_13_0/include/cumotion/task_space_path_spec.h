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

#include "cumotion/cumotion_export.h"
#include "cumotion/task_space_path.h"

namespace cumotion {

//! The `TaskSpacePathSpec` is used to procedurally specify a `TaskSpacePath` from a series of
//! continuous task space segments.
//!
//! Each segment can have position that is:
//!   [1] constant,
//!   [2] linearly interpolated,
//!   [3] or defined by a circular arc.
//! For the case where position follows an arc, the arc can be defined either by a series of three
//! points, or by two endpoints and a constraint that the arc be tangent to the previous segment.
//!
//! Each segment may have orientation that is:
//!   [1] constant (w.r.t. to the fixed global frame),
//!   [2] smoothly blended via spherical linear interpolation (i.e., "slerp"), or
//!   [3] constant w.r.t. the tangent direction of the position path (i.e., "tangent orientation").
//! The "tangent orientation" specification is only relevant for path segments for which position is
//! defined by an arc (since for linear position paths, the "tangent orientation" specification
//! would reduce to a constant orientation w.r.t. the fixed global frame).
class CUMO_EXPORT TaskSpacePathSpec {
 public:
  virtual ~TaskSpacePathSpec() = default;

  //! Add a path to linearly connect the previous pose to the `target_pose`.
  //!
  //! Position will use linear interpolation and orientation will use "slerp".
  //!
  //! An optional `blend_radius` can be used to add arc segments between successive segments.
  //! This arc segment will be added *only* if the successive segments both represent a path for
  //! which position is linearly interpolated. Moreover, the `blend_radius` will be capped such that
  //! no more than half of either linear segment is replaced by the intermediate arc segment. If
  //! `blend_radius` is <= 0, then no blending will be performed.
  //!
  //! If path segment is successfully added, `true` is returned. Else, `false` is returned and an
  //! error is logged.
  virtual bool addLinearPath(const Pose3 &target_pose, double blend_radius = 0.0) = 0;

  //! Add a translation-only path to linearly connect the previous pose to the `target_position`.
  //!
  //! Position will use linear interpolation, and orientation will be constant.
  //!
  //! An optional `blend_radius` can be used to add arc segments between successive segments.
  //! This arc segment will be added *only* if the successive segments both represent a path for
  //! which position is linearly interpolated. Moreover, the `blend_radius` will be capped such that
  //! no more than half of either linear segment is replaced by the intermediate arc segment. If
  //! `blend_radius` is <= 0, then no blending will be performed.
  //!
  //! If path segment is successfully added, `true` is returned. Else, `false` is returned and an
  //! error is logged.
  virtual bool addTranslation(const Eigen::Vector3d &target_position,
                              double blend_radius = 0.0) = 0;

  //! Add a rotation-only path connecting the previous pose to the `target_rotation`.
  //!
  //! Orientation will use "slerp" for interpolation, and position will be constant.
  //!
  //! If path segment is successfully added, `true` is returned. Else, `false` is returned and an
  //! error is logged.
  virtual bool addRotation(const Rotation3 &target_rotation) = 0;

  //! Add a path to connect the previous pose to the `target_position` along a circular arc that
  //! passes through `intermediate_position`.
  //!
  //! Position will follow a "three-point arc" where the previous position and `target_position` are
  //! endpoints and `intermediate_position` is an intermediate point on the arc.
  //!
  //! If `constant_orientation` is set to `true`, the orientation will be constant throughout the
  //! arc.
  //!
  //! If `constant_orientation` is set to `false`, the orientation will remain fixed relative to the
  //! tangent direction of the arc (i.e., "tangent orientation"; this tangent orientation is defined
  //! such that if the angular distance of the arc is N radians, then the change in orientation
  //! throughout the arc will be N radians about the normal axis of the arc).
  //!
  //! If path segment is successfully added, `true` is returned. Else, `false` is returned and an
  //! error is logged.
  virtual bool addThreePointArc(const Eigen::Vector3d &target_position,
                                const Eigen::Vector3d &intermediate_position,
                                bool constant_orientation = true) = 0;

  //! Add a path to connect the previous pose to the `target_pose` along a circular arc that
  //! passes through `intermediate_position`.
  //!
  //! Position will follow a "three-point arc" where the previous position and
  //! `target_pose.translation` are endpoints and `intermediate_position` is an intermediate point
  //! on the arc.
  //!
  //! Orientation will use "slerp" for interpolation between the previous orientation and
  //! `target_pose.rotation`.
  //!
  //! If path segment is successfully added, `true` is returned. Else, `false` is returned and an
  //! error is logged.
  virtual bool addThreePointArcWithOrientationTarget(
      const Pose3 &target_pose,
      const Eigen::Vector3d &intermediate_position) = 0;

  //! Add a path to connect the previous pose to the `target_position` along a circular arc that
  //! is tangent to the previous segment
  //!
  //! Position will follow a "tangent arc" where the previous position and `target_position` are
  //! endpoints and the arc is tangent to the previous segment.
  //!
  //! It is required that at least one previous segment on the path defines a change in position for
  //! a "tangent arc" to be able to be added. If no segments have been added or only rotation
  //! segments have been added, then an error will be logged, no "tangent arc" segment will be
  //! added, and `false` will be returned.
  //!
  //! If `constant_orientation` is set to `true`, the orientation will be constant throughout the
  //! arc.
  //!
  //! If `constant_orientation` is set to `false`, the orientation will remain fixed relative to the
  //! tangent direction of the arc (i.e., "tangent orientation"; this tangent orientation is defined
  //! such that if the angular distance of the arc is N radians, then the change in orientation
  //! throughout the arc will be N radians about the normal axis of the arc).
  //!
  //! If path segment is successfully added, `true` is returned. Else, `false` is returned and an
  //! error is logged.
  virtual bool addTangentArc(const Eigen::Vector3d &target_position,
                             bool constant_orientation = true) = 0;

  //! Add a path to connect the previous pose to the `target_pose` along a circular arc that
  //! is tangent to the previous segment.
  //!
  //! Position will follow a "tangent arc" where the previous position and `target_pose.translation`
  //! are endpoints and the arc is tangent to the previous segment.
  //!
  //! It is required that at least one previous segment on the path defines a change in position for
  //! a "tangent arc" to be able to be added. If no segments have been added or only rotation
  //! segments have been added, then an error will be logged, no "tangent arc" segment will be
  //! added, and `false` will be returned.
  //!
  //! Orientation will use "slerp" for interpolation between the previous orientation and
  //! `target_pose.rotation`.
  //!
  //! If path segment is successfully added, `true` is returned. Else, `false` is returned and an
  //! error is logged.
  virtual bool addTangentArcWithOrientationTarget(const Pose3 &target_pose) = 0;

  //! Generate a continuous path between all of the procedurally added task space path segments.
  //!
  //! The lower bound of the domain of the generated path will be zero and the upper bound will be
  //! determined by task space distance.
  [[nodiscard]] virtual std::unique_ptr<TaskSpacePath> generatePath() const = 0;
};

//! Create a `TaskSpacePathSpec` with the specified `initial_pose`.
CUMO_EXPORT std::unique_ptr<TaskSpacePathSpec> CreateTaskSpacePathSpec(const Pose3 &initial_pose);

}  // namespace cumotion
