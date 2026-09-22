// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES.
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
//! @brief Public interface for querying a kinematic structure.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"
#include "cumotion/pose3.h"

namespace cumotion {

//! Class representing the mapping from configuration space to coordinate frames that are rigidly
//! attached to the kinematic structure.
class CUMO_EXPORT Kinematics {
 public:
  //! Opaque handle to a frame
  struct CUMO_EXPORT FrameHandle {
    class Impl;
    std::shared_ptr<Impl> impl;
  };

  virtual ~Kinematics() = default;

  //! Return the number of configuration space coordinates for the kinematic structure.
  [[nodiscard]] virtual int numCSpaceCoords() const = 0;

  //! Return the name of a given configuration space coordinate of the kinematic structure.
  //!
  //! A fatal error will be logged if:
  //!   1. `coord_index` < 0, *OR*
  //!   2. `coord_index` >= `numCSpaceCoords()`.
  [[nodiscard]] virtual std::string cSpaceCoordName(int coord_index) const = 0;

  //! Lower and upper limits for a configuration space coordinate.
  struct CUMO_EXPORT Limits {
    double lower;  //!< lower limit
    double upper;  //!< upper limit
  };

  //! Return the limits of a given configuration space coordinate of the kinematic structure.
  //!
  //! A fatal error will be logged if:
  //!   1. `coord_index` < 0, *OR*
  //!   2. `coord_index` >= `numCSpaceCoords()`.
  [[nodiscard]] virtual Limits cSpaceCoordLimits(int coord_index) const = 0;

  //! Determine whether a specified configuration space position is within limits for each
  //! coordinate.
  //!
  //! If `log_warnings` is set to `true` and `cspace_position` is outside limits, a warning will be
  //! logged indicating which coordinates are outside limits.
  virtual bool withinCSpaceLimits(const Eigen::VectorXd &cspace_position,
                                  bool log_warnings) const = 0;

  //! Return the velocity limit of a given configuration space coordinate of the kinematic
  //! structure.
  //!
  //! A fatal error will be logged if:
  //!   1. `coord_index` < 0, *OR*
  //!   2. `coord_index` >= `numCSpaceCoords()`.
  [[nodiscard]] virtual double cSpaceCoordVelocityLimit(int coord_index) const = 0;

  //! Return the acceleration limit of a given configuration space coordinate of the kinematic
  //! structure.
  //!
  //! A fatal error will be logged if:
  //!   1. `coord_index` < 0, *OR*
  //!   2. `coord_index` >= `numCSpaceCoords()`.
  [[nodiscard]] virtual double cSpaceCoordAccelerationLimit(int coord_index) const = 0;

  //! Return the jerk limit of a given configuration space coordinate of the kinematic structure.
  //!
  //! A fatal error will be logged if:
  //!   1. `coord_index` < 0, *OR*
  //!   2. `coord_index` >= `numCSpaceCoords()`.
  [[nodiscard]] virtual double cSpaceCoordJerkLimit(int coord_index) const = 0;

  //! Return all the frame names in the kinematic structure.
  [[nodiscard]] virtual const std::vector<std::string> &frameNames() const = 0;

  //! Return the name of the given `frame`.
  [[nodiscard]] virtual const std::string &frameName(const FrameHandle &frame) const = 0;

  //! Return a handle representing the frame with the given `frame_name`.
  //!
  //! A fatal error will be logged if `frame_name` is not a valid frame name.
  [[nodiscard]] virtual FrameHandle frame(const std::string &frame_name) const = 0;

  //! Return a handle representing the base frame of the kinematic structure.
  [[nodiscard]] virtual FrameHandle baseFrame() const = 0;

  //! Return the pose of the given `frame` with respect to `reference_frame` at the given
  //! `cspace_position`.
  [[nodiscard]] virtual Pose3 pose(const Eigen::VectorXd &cspace_position,
                                   const FrameHandle &frame,
                                   const FrameHandle &reference_frame) const = 0;

  //! Return the pose of the given `frame` with respect to the base frame (i.e., `baseFrame()`) at
  //! the given `cspace_position`.
  [[nodiscard]] virtual Pose3 pose(const Eigen::VectorXd &cspace_position,
                                   const FrameHandle &frame) const = 0;

  //! Return the position of the origin of the given `frame` with respect to `reference_frame` at
  //! the given `cspace_position`.
  [[nodiscard]] virtual Eigen::Vector3d position(const Eigen::VectorXd &cspace_position,
                                                 const FrameHandle &frame,
                                                 const FrameHandle &reference_frame) const = 0;

  //! Return the position of the origin of the given `frame` with respect to the base frame
  //! (i.e., `baseFrame()`) at the given `cspace_position`.
  [[nodiscard]] virtual Eigen::Vector3d position(const Eigen::VectorXd &cspace_position,
                                                 const FrameHandle &frame) const = 0;

  //! Return the orientation of the given `frame` with respect to `reference_frame` at the given
  //! `cspace_position`.
  [[nodiscard]] virtual Rotation3 orientation(const Eigen::VectorXd &cspace_position,
                                              const FrameHandle &frame,
                                              const FrameHandle &reference_frame) const = 0;

  //! Return the orientation of the given `frame` with respect to the base frame
  //! (i.e., `baseFrame()`) at the given `cspace_position`.
  [[nodiscard]] virtual Rotation3 orientation(const Eigen::VectorXd &cspace_position,
                                              const FrameHandle &frame) const = 0;

  //! Return the Jacobian of the origin of the given `frame` with respect to the base frame
  //! (i.e., `baseFrame()`) at the given `cspace_position`.
  //!
  //! The returned Jacobian is a 6 x N matrix where N is the `numCSpaceCoords`. Column `i` of the
  //! returned Jacobian represents the perturbation contribution to origin of `frame` from the
  //! `i`th c-space element in the coordinates of the base frame. For each column, the first three
  //! elements represent the translational portion and the last three elements represent the
  //! rotational portion.
  [[nodiscard]] virtual Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(
      const Eigen::VectorXd &cspace_position,
      const FrameHandle &frame) const = 0;

  //! Return the Jacobian of the position of the origin of the given `frame` with respect to the
  //! base frame (i.e., `baseFrame()`) at the given `cspace_position`.
  //!
  //! The returned Jacobian is a 3 x N matrix where N is the `numCSpaceCoords`. Column `i` of the
  //! returned Jacobian represents the perturbation contribution to position of the origin of
  //! `frame` from the `i`th c-space element in the coordinates of the base frame.
  [[nodiscard]] virtual Eigen::Matrix<double, 3, Eigen::Dynamic> positionJacobian(
      const Eigen::VectorXd &cspace_position,
      const FrameHandle &frame) const = 0;

  //! Return the Jacobian of the orientation of the given `frame` with respect to the base frame
  //! (i.e., `baseFrame()`) at the given `cspace_position`.
  //!
  //! The returned Jacobian is a 3 x N matrix where N is the `numCSpaceCoords`. Column `i` of the
  //! returned Jacobian represents the perturbation contribution to orientation of `frame` from the
  //! `i`th c-space element in the coordinates of the base frame.
  [[nodiscard]] virtual Eigen::Matrix<double, 3, Eigen::Dynamic> orientationJacobian(
      const Eigen::VectorXd &cspace_position,
      const FrameHandle &frame) const = 0;

  //! Return the fixed transform between `frame` and its parent frame.
  //!
  //! Internally, the `Kinematics` structure is represented as a directed rooted tree. Each frame
  //! has a single parent frame (other than the root which by definition has no parent). The edge
  //! from parent frame to child frame represents both a fixed transform (i.e., a rigid linkage) AND
  //! a "joint" transform (revolute, prismatic, or fixed). The root frame, while not having a
  //! parent, maintains a fixed transform that defines its pose relative to global coordinates.
  //!
  //! This function returns the fixed transform that precedes the "joint" transform. The returned
  //! `Pose3` is expressed relative to the parent frame. For example, if the `wrist_frame` is a
  //! child of the `elbow_frame`, the returned pose would be the pose of the wrist joint expressed
  //! in the local coordinates of the `elbow_frame`.
  //!
  //! NOTE: In general, the returned pose will NOT be the pose of the child frame expressed in the
  //! local coordinates of the parent frame, as the "joint" transform is NOT included in the
  //! returned transform, and the "joint" transform, in general, will not be identity. In order to
  //! return the full transform from parent frame to child frame for a given c-space configuration,
  //! the `pose()` function should be used with the child frame input for `frame` and the parent
  //! frame input for `reference_frame`.
  [[nodiscard]] virtual Pose3 getPrecedingFixedTransform(const FrameHandle &frame) const = 0;

  //! Set the fixed transform between `frame` and its parent frame to `transform`.
  //!
  //! Internally, the `Kinematics` structure is represented as a directed rooted tree. Each frame
  //! has a single parent frame (other than the root which by definition has no parent). The edge
  //! from parent frame to child frame represents both a fixed transform (i.e., a rigid linkage) AND
  //! a "joint" transform (revolute, prismatic, or fixed). The root frame, while not having a
  //! parent, maintains a fixed transform that defines its pose relative to global coordinates; it
  //! is valid to use this function to set that tranform to pose the base of the robot relative to
  //! the global frame.
  //!
  //! This function sets the fixed transform that precedes the "joint" transform. The input
  //! `transform` is expressed relative to the parent frame. For example, if the `wrist_frame` is a
  //! child of the `elbow_frame`, the input `transform` would define the pose of the wrist joint
  //! expressed in the local coordinates of the `elbow_frame`.
  virtual void setPrecedingFixedTransform(const FrameHandle &frame, const Pose3 &transform) = 0;
};

}  // namespace cumotion
