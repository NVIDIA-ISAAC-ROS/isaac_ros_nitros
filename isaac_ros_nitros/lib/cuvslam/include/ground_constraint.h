/**
 * @file ground_constraint.h

 * @copyright Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.\n\n
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#pragma once
#include "cuvslam.h"

#ifdef __cplusplus
extern "C" {
#endif
/// @endcond

/**
 * Ground constraint Handle
 *
 * The ground constraint is a post-processing tool to constrain 3d poses from Odometry on the ground plane.
 * Because of the drifts or failures after some time, 3D poses from odometry will have roll and pitch,
 * even if the robot moves on a flat surface. To resolve it, GroundConstraint will integrate 2d deltas in each frame,
 * removing roll and pitch angles on small deltas.
 */
typedef struct CUVSLAM_GroundConstraint* CUVSLAM_GroundConstraintHandle;

/**
 * Initialize ground constraint
 * @param[out] ground                  created ground constraint handle
 * @param[in]  world_from_ground       world from ground plane transformation
 * @param[in]  initial_pose_on_ground  initial pose on ground in world coordinate frame
                                       if pose is not belong to the ground plane it will projected
                                       and rotated with nearest angle
 * @param[in]  initial_pose_in_space   initial pose in space in world coordinate frame
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GroundConstraintCreate(CUVSLAM_GroundConstraintHandle* ground,
                                              const struct CUVSLAM_Pose* world_from_ground,
                                              const struct CUVSLAM_Pose* initial_pose_on_ground,
                                              const struct CUVSLAM_Pose* initial_pose_in_space);

/**
 * Release all resources owned by the ground constraint
 * @param[in] ground ground constraint handle
 */
CUVSLAM_API
void CUVSLAM_GroundConstraintDestroy(CUVSLAM_GroundConstraintHandle ground);

/**
 * Add new 3d position from Odometry to 2d integrator
 * @param[in]  ground              ground constraint handle
 * @param[in]  next_pose_in_space  next to previous (or to initial) pose in 3d space in world frame
 *                                 pose it expected have just small delta from previous one.
 *                                 After projection roll and pitch angles will be removed.
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GroundConstraintAddNextPose(CUVSLAM_GroundConstraintHandle ground,
                                                   const struct CUVSLAM_Pose* next_pose_in_space);

/**
 * Return current integrated 2d pose
 * @param[in]  handle          ground constraint handle
 * @param[in]  pose_on_ground  pose on the ground-plane in world frame
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GroundConstraintGetPoseOnGround(CUVSLAM_GroundConstraintHandle handle,
                                                       struct CUVSLAM_Pose* pose_on_ground);

#ifdef __cplusplus
} // extern "C"
#endif
