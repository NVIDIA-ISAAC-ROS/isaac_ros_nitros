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
//! @brief Public interface for inverse kinematics solvers.

#pragma once

#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"
#include "cumotion/kinematics.h"

namespace cumotion {

//! Configuration for solving inverse kinematics.
struct CUMO_EXPORT IkConfig {
  //! Construct configuration with default values likely to be appropriate for most 6- and 7-dof
  //! robotic arms.
  IkConfig() = default;

  //! The maximum number of c-space positions that will be used as seeds (i.e., initial positions)
  //! for cyclic coordinate descent.
  //!
  //! The inverse kinematic solver will terminate when a valid c-space configuration is found OR
  //! the number of attempted descents reaches `max_num_descents`.
  //!
  //! Must be positive.
  //!
  //! Default: 100
  int max_num_descents = 100;

  //! Optional parameter to set the initial c-space configurations for each descent.
  //!
  //! These `cspace_seeds` will be used to attempt cyclic coordinate descent in the order
  //! provided, terminating before all `cspace_seeds` are tried if a valid `cspace_position` is
  //! found.
  //!
  //! If the number of attempted descents is greater than the number of provided `cspace_seeds`,
  //! random starting configurations will be generated (see `irwin_hall_sampling_order` for
  //! details).
  //!
  //! It is valid to provide no `cspace_seeds`. In this case all initial configurations will be
  //! randomly sampled.
  std::vector<Eigen::VectorXd> cspace_seeds = {};

  //! If the number of attempted descents exceeds the provided number of `cspace_seeds` (including
  //! the case where no `cspace_seeds` are provided), then random c-space configurations will be
  //! sampled for each descent.
  //!
  //! Each c-space coordinate value will be sampled from an Irwin-Hall distribution spanning the
  //! joint limits. The distribution mean will always be set to the midpoint between upper and
  //! lower joint limits.
  //!
  //! Setting `irwin_hall_sampling_order` to 1 corresponds to a uniform distribution, setting to
  //! 2 corresponds to a triangular distribution, and higher values approximate a normal
  //! distribution.
  //!
  //! See https://en.wikipedia.org/wiki/Irwin%E2%80%93Hall_distribution for reference, noting that
  //! the internal distribution is scaled s.t. all sampled values will be within joint limits.
  //!
  //! Must be positive.
  //!
  //! Default: 2
  int irwin_hall_sampling_order = 2;

  //! Seed for random number generator used for Irwin-Hall sampling for initial c-space positions.
  //!
  //! Must be non-negative.
  //!
  //! Default: 123456
  int sampling_seed = 123456;

  //! Maximum position error (L2-norm) of the task space pose for a successful IK solution.
  //!
  //! Must be positive.
  //!
  //! Default: 1e-3
  double position_tolerance = 1e-3;

  //! Maximum orientation error of the task space pose for a successful IK solution.
  //!
  //! For each axis in task space, error is defined as the L2-norm of the difference between the
  //! axis direction and the target pose axis direction.
  //!
  //! Must be positive.
  //!
  //! Default: 0.01
  double orientation_tolerance = 0.01;

  //================================================================================================
  // Parameters for cyclic coordinate descent (CCD) algorithm.

  //! The number of iterations used for cyclic coordinate descent from each initial c-space
  //! position (i.e., seed).
  //!
  //! Smaller values will decrease the amount of computation time per descent, but may require
  //! more descents to converge to valid c-space configuration.
  //!
  //! CCD will be disabled for values less than 1.
  //!
  //! Default: 10
  int ccd_max_iterations = 10;

  //! Each descent can terminate early if the L2-norm of the change to c-space coordinates is less
  //! than the `descent_termination_delta`.
  //!
  //! Must be positive.
  //!
  //! Default: 1e-1
  double ccd_descent_termination_delta = 1e-1;

  //! Weight for the relative importance of position error term when performing
  //! cyclic coordinate descent.
  //!
  //! Must be non-negative.
  //!
  //! Default: 1.0
  double ccd_position_weight = 1.0;

  //! Weight for the relative importance of orientation error term when performing
  //! cyclic coordinate descent.
  //!
  //! Must be non-negative.
  //!
  //! Default: 0.05
  double ccd_orientation_weight = 0.05;

  //! Number of samples used to identify valid three-point bracket for numerical optimization of
  //! c-space updates.
  //!
  //! NOTE: This parameter is *ONLY* used when more than one active upstream joint is controlled by
  //! a single c-space coordinate.
  //!
  //! For the "many-to-one joint-to-cspace" case, quadratic fit search (QFS) is used to approximate
  //! an optimal c-space update during each iteration of cyclic coordinate descent. In order to
  //! find a valid three-point bracket for QFS, N points are sampled to attempt to find a region
  //! with minimum error. This sampling is incorporated into the numerical solver since, in
  //! general, the error function will not be unimodal within the domain bounded by the c-space
  //! position limits.
  //!
  //! Must be greater than one.
  //!
  //! Default: 10
  int ccd_bracket_search_num_uniform_samples = 10;

  //================================================================================================
  // Parameters for Broyden-Fletcher-Goldfarb-Shanno (BFGS) solver.

  //! The number of iterations used for Broyden-Fletcher-Goldfarb-Shanno (BFGS) descent from each
  //! initial c-space position (i.e., seed).
  //!
  //! Smaller values will decrease the amount of computation time per descent, but may require
  //! more descents to converge to valid c-space configuration.
  //!
  //! Must be positive.
  //!
  //! Default: 60
  int bfgs_max_iterations = 60;

  //! The BFGS solver will terminate if the L2-norm of the error function gradient at the current
  //! best c-space position is less than `bfgs_gradient_norm_termination`.
  //!
  //! Low values for `bfgs_gradient_norm_termination` will increase solver accuracy, while high
  //! values will decrease solve times for a given problem.
  //!
  //! Must be positive.
  //!
  //! Default: 1e-6
  double bfgs_gradient_norm_termination = 1e-6;

  //! The BFGS solver is implemented as a two-step process, with a coarse solve used to identify
  //! whether convergence to a valid c-space position is likely.
  //!
  //! For the coarse solve, the `bfgs_gradient_norm_termination` is multiplied by the
  //! `bfgs_gradient_norm_termination_coarse_scale_factor` such that the optimization will end
  //! early.
  //!
  //! Must be greater than or equal to 1.
  //!
  //! Default: 1e7
  double bfgs_gradient_norm_termination_coarse_scale_factor = 1e7;

  //! The error function for BFGS descent optionally includes a cost-term for avoiding c-space
  //! coordinate values near position limits.
  //!
  //! If set to `ENABLE`, this cost term will always be turned on. If set to `DISABLE`, this cost
  //! term will always be turned off. Setting to `AUTO` will turn this cost term on for systems
  //! with 7 or more degrees-of-freedom (DoF), while leaving it off for systems with 6 or fewer
  //! DoF. NOTE: DoF is defined as the numer of c-space coordinates upstream from the target frame.
  //!
  //! Setting to `AUTO` is likely to provide desirable results for most common robots. Redundant
  //! mechanisms with less than 7-DoF are a potential exception. E.g., a 4-Dof robot restricted to
  //! planar motion may benefit from enabling a cost term to avoid position limits.
  enum class CSpaceLimitBiasing {
    AUTO,
    ENABLE,
    DISABLE,
  };

  //! See documentation for `CSpaceLimitBiasing`.
  //!
  //! Default: AUTO
  CSpaceLimitBiasing bfgs_cspace_limit_biasing = CSpaceLimitBiasing::AUTO;

  //! Define the region near c-space limits which will induce penalties when c-space limit biasing
  //! is active.
  //!
  //! The region is defined as a fraction of the c-space span for each c-space coordinate.
  //! For example, imagine a 2d system for which the limits of the first coordinate are [-5, 5] and
  //! the limits of the second coordinate are [0, 2]. If the `cspace_limit_penalty_region` is set
  //! to 0.1, then the size of each penalty region for first coordinate will be:
  //!   0.1 * (5 - (-5)) = 1
  //! Similarly, the size of each penalty region for the second coordinate will be:
  //!   0.1 * (2 - 0) = 0.2
  //! This means that for the first coordinate, c-space limit errors will be active from [-5, -4]
  //! and [4, 5]. For the second coordinate, c-space limit errors will be active from [0, 0.2] and
  //! [1.8, 2]
  //!
  //! Must be in the range [0, 0.5].
  //!
  //! Default: 0.01
  double bfgs_cspace_limit_penalty_region = 0.01;

  //! Relative weight applied to c-space limit error (if active).
  //!
  //! Must be non-negative.
  //!
  //! Default: 1.0
  double bfgs_cspace_limit_biasing_weight = 1.0;

  //! Weight for the relative importance of position error term when performing BFGS descent.
  //!
  //! Must be non-negative.
  //!
  //! Default: 1.0
  double bfgs_position_weight = 1.0;

  //! Weight for the relative importance of orientation error term when performing BFGS descent.
  //!
  //! Must be non-negative.
  //!
  //! Default: 100.0
  double bfgs_orientation_weight = 100.0;
};

//! Results from solving inverse kinematics.
struct CUMO_EXPORT IkResults {
  //! Indicate whether a valid `cspace_position` within the tolerances specified in the
  //! `IkConfig` has been found.
  bool success;

  //! If `success` is set to `true`, `cspace_position` will contain a valid joint configuration
  //! that corresponds to the `target_pose` passed to the IK solver within the tolerances
  //! specified in the `IkConfig`.
  Eigen::VectorXd cspace_position;

  //! Position error (L2-norm) of the task space pose corresponding to the returned
  //! `cspace_position`
  //!
  //! If `success` is set to `true`, the `position_error` will be less than or equal to the
  //! `position_tolerance` set in the `IkConfig`.
  double position_error;

  //! X-axis orientation error (L2-norm) of the task space pose corresponding to the returned
  //! `cspace_position`
  //!
  //! If `success` is set to `true`, the `x_axis_orientation_error` will be less than or equal to
  //! the `orientation_tolerance` set in the `IkConfig`.
  double x_axis_orientation_error;

  //! Y-axis orientation error (L2-norm) of the task space pose corresponding to the returned
  //! `cspace_position`
  //!
  //! If `success` is set to `true`, the `y_axis_orientation_error` will be less than or equal to
  //! the `orientation_tolerance` set in the `IkConfig`.
  double y_axis_orientation_error;

  //! Z-axis orientation error (L2-norm) of the task space pose corresponding to the returned
  //! `cspace_position`
  //!
  //! If `success` is set to `true`, the `z_axis_orientation_error` will be less than or equal to
  //! the `orientation_tolerance` set in the `IkConfig`.
  double z_axis_orientation_error;

  //! The number of unique c-space positions that were used for attempting cyclic coordinate
  //! descent prior to the IK solver terminating.
  //!
  //! If `success` is set to `true`, this will be the number
  //! of descents that were attempted in order to get a `cspace_position` within the provided
  //! tolerance. If `success` is set to `false`, the number of descents will be set to the
  //! `max_num_descents` set in the `IkConfig`.
  int num_descents;
};

//! Attempt to solve inverse kinematics.
//!
//! A `target_pose` is provided for the task space `target_frame` and the solver attempts to find
//! a corresponding set of c-space coordinates. A valid c-space configuration must not exceed
//! joint limits set in the kinematic structure and must be the target pose tolerances specified
//! within `config`.
//!
//! If the `target_frame` is selected such that some of the c-space coordinates do not affect the
//! pose of the target frame, these "downstream" coordinates will be ignored by the IK solver. For
//! the case where the successful descent was started from a seed provided in `config.cspace_seeds`,
//! the "downstream" coordinates will not be altered from the provided seed. For the case where
//! a random seed is used, the "downstream" coordinates will be returned with values between their
//! respective c-space position limits. This behavior is intended to support cases such as branched
//! kinematic structures (i.e., to solve for palm pose of a robotic system with an arm and multiple
//! fingers) or a redundant serial linkage for which it is desired to set a pose of a frame not
//! rigidly fixed to the end effector.
//!
//! This implementation of cyclic coordinate descent (CCD) is based on "A Combined Optimization
//! Method for Solving the Inverse Kinematics Problem of Mechanical Manipulators" (1991) by
//! Wang et al.
//! Ref: http://web.cse.ohio-state.edu/~parent.1/classes/788/Sp06/ReferenceMaterial/IK/WC91.pdf
//!
//! As described by Wang et al., this "combined" method uses cyclic coordinate descent (CCD) as a
//! quick method to find a feasible c-space position that is in the neighborhood of an exact
//! solution to the inverse kinematics problem. These nearby c-space positions are then used to
//! "warm start" the Broyden-Fletcher-Goldfarb-Shanno (BFGS) algorithm. Given a point sufficiently
//! near an exact solution, the BFGS algorithm is able to leverage an estimate of curvature to
//! rapidly converge to a very accurate approximation of the exact solution.
//!
//! Concise explanations of the CCD algorithm with interactive demonstrations are available at:
//! [1] https://zalo.github.io/blog/inverse-kinematics/
//! [2] http://rodolphe-vaillant.fr/?e=114
CUMO_EXPORT IkResults SolveIk(const Kinematics &kinematics,
                              const Pose3 &target_pose,
                              const Kinematics::FrameHandle &target_frame,
                              const IkConfig &config);

}  // namespace cumotion
