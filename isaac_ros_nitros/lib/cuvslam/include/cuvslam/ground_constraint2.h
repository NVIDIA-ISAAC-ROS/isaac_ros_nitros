/**
 * @file ground_constraint2.h

 * @copyright Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.\n\n
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#pragma once

#include <memory>

#include "cuvslam/cuvslam2.h"

namespace cuvslam {

/**
 * Ground constraint
 *
 * The ground constraint is a post-processing tool to constrain 3d poses from Odometry on the ground plane.
 * Because of the drifts or failures after some time, 3D poses from odometry will have roll and pitch,
 * even if the robot moves on a flat surface. To resolve it, GroundConstraint will integrate 2d deltas in each frame,
 * removing roll and pitch angles on small deltas.
 */
class CUVSLAM_API GroundConstraint {
public:
  /**
   * Construct a ground constraint integrator.
   * @param[in] world_from_ground       Ground plane pose in world frame
   * @param[in] initial_pose_on_ground  Initial pose projected on ground in world frame
   * @param[in] initial_pose_in_space   Initial unconstrained pose in world frame
   * @throws std::runtime_error on initialization failure
   */
  GroundConstraint(const Pose& world_from_ground, const Pose& initial_pose_on_ground,
                   const Pose& initial_pose_in_space);

  ~GroundConstraint();

  GroundConstraint(const GroundConstraint&) = delete;
  GroundConstraint& operator=(const GroundConstraint&) = delete;
  /**
   * Move constructor
   * @param[in] other other ground constraint
   */
  GroundConstraint(GroundConstraint&& other) noexcept;
  /**
   * Move assignment operator
   * @param[in] other other ground constraint
   * @return reference to this
   */
  GroundConstraint& operator=(GroundConstraint&& other) noexcept;

  /**
   * Add next pose in space (3D) to the integrator.
   * Removes roll and pitch after projecting to the ground plane.
   * @param[in] next_pose_in_space Next pose in world frame
   */
  void AddNextPose(const Pose& next_pose_in_space);

  /**
   * Get current pose constrained to the ground plane in world frame.
   * @return pose on the ground plane in world frame
   */
  Pose GetPoseOnGround() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};

}  // namespace cuvslam
