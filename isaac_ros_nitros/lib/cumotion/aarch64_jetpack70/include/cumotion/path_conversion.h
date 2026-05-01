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

#include "cumotion/composite_path_spec.h"
#include "cumotion/cumotion_export.h"
#include "cumotion/ik_solver.h"
#include "cumotion/kinematics.h"
#include "cumotion/linear_cspace_path.h"

namespace cumotion {

//! Configuration parameters for converting a `TaskSpacePathSpec` into a series of c-space
//! configurations.
struct CUMO_EXPORT TaskSpacePathConversionConfig {
  //! Create default set of configuration parameters for converting a `TaskSpacePathSpec` into a
  //! series of c-space configurations.
  //!
  //! Default parameters are expected to work well for converting most task space paths to c-space
  //! paths.
  //!
  //! If a tighter tolerance for adherence to the task space path is desired, then the
  //! `min_position_deviation` and/or `max_position_deviation` can be decreased (at the expense of
  //! more computational cost and more c-space waypoints which will likely lead to trajectories with
  //! longer time-spans). Conversely, tolerances can be loosened for faster path conversion
  //! and/or (likely) faster trajectories.
  //!
  //! Beyond (optionally) adjusting position deviations, it is unlikely that other parameters will
  //! need to be modified from the provided default values.
  TaskSpacePathConversionConfig() = default;

  //! For each c-space waypoint that is generated, the position deviation between the desired task
  //! space path and a task space mapping of a straight-line interpolation in c-space is
  //! approximated. The minimum position deviation places a lower bound of deviation required to add
  //! a c-space waypoint.
  //!
  //! While it is somewhat unintuitive that the deviation could be *too* small, this
  //! minimum deviation is used to control the minimum spacing between c-space configurations along
  //! the task space path domain. A (relatively) sparse series of c-space waypoints is desirable for
  //! trajectory generation; if the minimum deviation is arbitrarily small then the c-space points
  //! will be (in general) too close together to generate a time-optimal trajectory. Generation of
  //! excessive c-space waypoints will also be computationally expensive and is, in general, best
  //! avoided.
  //!
  //! `min_position_deviation` must be positive and less than `max_position_deviation`.
  //!
  //! Default value is 0.001.
  double min_position_deviation = 0.001;

  //! For each c-space waypoint that is generated, the position deviation between the desired task
  //! space path and a task space mapping of a straight-line interpolation in c-space is
  //! approximated. The maximum position deviation places an upper bound of deviation allowed to add
  //! a c-space waypoint.
  //!
  //! `max_position_deviation` must be positive and greater than `min_position_deviation`.
  //!
  //! Default value is 0.003.
  double max_position_deviation = 0.003;

  //! Initial step size in the domain value 's' used to sample poses from the task space path to be
  //! converted to c-space waypoints.
  //!
  //! The 's' step size will be adaptively updated throughout the path conversion and the default
  //! value for initialization is generally recommended.
  //!
  //! `initial_s_step_size` must be positive.
  //!
  //! Default value is 0.05.
  double initial_s_step_size = 0.05;

  //! Initial step size "delta" that is used to adaptively adjust the 's' step size; 's' is the
  //! domain value 's' used to sample poses from the task space path to be converted to c-space
  //! waypoints.
  //!
  //! The 's' step size "delta" will be adaptively updated throughout the path conversion and the
  //! default value for initialization is generally recommended.
  //!
  //! `initial_s_step_size_delta` must be positive.
  //!
  //! Default value is 0.005.
  double initial_s_step_size_delta = 0.005;

  //! Minimum allowable interval in domain value 's' that can separate poses from the task space
  //! path to be converted to c-space waypoints.
  //!
  //! The minimum 's' step size serves to limit the number of c-space configurations that can be
  //! returned in the converted path. Specifically, the upper bound for the number of returned
  //! c-space configurations is ("span of the task space path domain" / min_s_step_size) + 1.
  //!
  //! `min_s_step_size` must be positive.
  //!
  //! Default value is 1e-5.
  double min_s_step_size = 1e-5;

  //! Minimum allowable 's' step size "delta" used to adaptively update the 's' step size.
  //!
  //! The `min_s_step_size_delta` serves to limit wasted iterations when (minimal) progress is being
  //! made towards path conversion. If `min_s_step_size_delta` is reached during the search for any
  //! c-space waypoint, then path conversion will fail.
  //!
  //! The default value is generally recommended.
  //!
  //! `min_s_step_size_delta` must be positive.
  //!
  //! Default value is 1e-5.
  double min_s_step_size_delta = 1e-5;

  //! Maximum number of iterations to search for each c-space waypoint.
  //!
  //! If `max_iterations` is reached for any c-space waypoint, then path conversion will fail.
  //!
  //! `max_iterations` must be positive.
  //!
  //! Default value is 50.
  int max_iterations = 50;

  //! "alpha" is used to exponentially scale the 's' step size "delta" to speed convergence when the
  //! 's' step size is being successively increased or successively decreased. When an increase is
  //! followed by a decrease, or vice versa, "alpha" is used to decrease the 's' step size "delta"
  //! to reduce overshoot.
  //!
  //! The default value is generally recommended.
  //!
  //! `alpha` must be greater than 1.
  //!
  //! Default value is 1.4.
  double alpha = 1.4;
};

//! Convert a `composite_path_spec` into a linear c-space path.
//!
//! The mapping from c-space to task space is defined by `kinematics` for the given `control_frame`.
//!
//! If non-default configuration parameters for the path conversion process are desired, then
//! `task_space_path_conversion_config` can (optionally) be specified.
//!
//! If non-default configuration parameters for the inverse kinematics (IK) solver are desired,
//! then `ik_config` can optionally be specified.
CUMO_EXPORT std::unique_ptr<LinearCSpacePath> ConvertCompositePathSpecToCSpace(
    const CompositePathSpec &composite_path_spec,
    const Kinematics &kinematics,
    const Kinematics::FrameHandle &control_frame,
    const TaskSpacePathConversionConfig &task_space_path_conversion_config =
        TaskSpacePathConversionConfig(),
    const IkConfig &ik_config = IkConfig());

//! Convert a `task_space_path_spec` into a linear c-space path.
//!
//! Inverse kinematics will be used to convert the initial task space pose of `task_space_path_spec`
//! to a c-space position. If a particular c-space solution is desired, this can be set in
//! `ik_config.cspace_seeds`. If the specified c-space does *not* correspond to the initial task
//! space pose, it will simply be used as a warm start and the IK solver will proceed to search for
//! an appropriate c-space position.
//!
//! The mapping from c-space to task space is defined by `kinematics` for the given `control_frame`.
//!
//! If non-default configuration parameters for the path conversion process are desired, then
//! `task_space_path_conversion_config` can (optionally) be specified.
//!
//! If non-default configuration parameters for the inverse kinematics (IK) solver are desired,
//! then `ik_config` can optionally be specified.
CUMO_EXPORT std::unique_ptr<LinearCSpacePath> ConvertTaskSpacePathSpecToCSpace(
    const TaskSpacePathSpec &task_space_path_spec,
    const Kinematics &kinematics,
    const Kinematics::FrameHandle &control_frame,
    const TaskSpacePathConversionConfig &task_space_path_conversion_config =
        TaskSpacePathConversionConfig(),
    const IkConfig &ik_config = IkConfig());

}  // namespace cumotion
