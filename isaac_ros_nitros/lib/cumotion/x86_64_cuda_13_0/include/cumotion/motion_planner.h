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
//! @brief Public interface to cuMotion's global motion planning implementation.

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"
#include "cumotion/robot_description.h"
#include "cumotion/world.h"

namespace cumotion {

//! Configuration parameters for a `MotionPlanner`.
class CUMO_EXPORT MotionPlannerConfig {
 public:
  virtual ~MotionPlannerConfig() = default;

  //! Indicate lower and upper limits for a coordinate.
  struct CUMO_EXPORT Limit {
    double lower;
    double upper;
  };

  //! Specify the value for a given parameter.
  //!
  //! The required `ParamValue` constructor for each param is detailed in the
  //! documentation for `setParam()`.
  struct CUMO_EXPORT ParamValue {
    //! Create `ParamValue` from `int`.
    ParamValue(int value);  // NOLINT Allow implicit conversion
    //! Create `ParamValue` from `double`.
    ParamValue(double value);  // NOLINT Allow implicit conversion
    //! Create `ParamValue` from `Eigen::Vector3d`.
    ParamValue(const Eigen::Vector3d &value);  // NOLINT Allow implicit conversion
    //! Create `ParamValue` from `std::vector<double>`.
    ParamValue(const std::vector<double> &value);  // NOLINT Allow implicit conversion
    //! Create `ParamValue` from `bool`.
    ParamValue(bool value);  // NOLINT Allow implicit conversion
    //! Create `ParamValue` from `const char*`.
    ParamValue(const char *value);  // NOLINT Allow implicit conversion
    //! Create `ParamValue` from `std::string`.
    ParamValue(const std::string &value);  // NOLINT Allow implicit conversion
    //! Create `ParamValue` from `std::vector<Limit>`.
    ParamValue(const std::vector<Limit> &value);  // NOLINT Allow implicit conversion

    struct Impl;
    std::shared_ptr<Impl> impl;
  };

  //! Set the value of the parameter.
  //!
  //! `setParam` returns `true` if the param has been successfully updated and `false` if an error
  //! has occurred (e.g., invalid parameter).
  //!
  //! The following parameters can be set for `MotionPlanner`:
  //!
  //! `seed` [`int`]
  //!   - Used to initialize random sampling.
  //!   - `seed` must be positive.
  //!   - Default: 1234
  //!
  //! `step_size` [`double`]
  //!   - Step size for tree extension and (optionally) for discretization of `interpolated_path`
  //!     in `Results`
  //!   - It is assumed that a straight path connecting two valid c-space configurations with
  //!     separation distance <= `step_size` is a valid edge, where separation distance is defined
  //!     as the L2-norm of the difference between the two configurations.
  //!   - `step_size` must be positive.
  //!   - Default: 0.05
  //!
  //! `max_iterations` [`int`]
  //!   - Maximum number of iterations of tree extensions that will be attempted.
  //!   - If `max_iterations` is reached without finding a valid path, the `Results` will
  //!     indicate `path_found` is `false` and `path` will be an empty vector.
  //!   - `max_iterations` must be positive.
  //!   - Default: 10,000
  //!
  //! `max_sampling` [`int`]
  //!   - Maximum number of configurations used for sampling in the environment setup.
  //!   - `max_sampling` must be positive.
  //!   - Default: 10'000
  //!
  //! `distance_metric_weights` [`std::vector<double>`]
  //!   - When selecting a node for tree extension, the closest node is defined using a weighted,
  //!     squared L2-norm:
  //!       distance = (q0 - q1)^T * W * (q0 - q1)
  //!       where q0 and q1 represent two configurations and W is a diagonal matrix formed from
  //!       `distance_metric_weights`.
  //!   - The length of the `distance_metric_weights` must be equal to the number of c-space
  //!     coordinates for the robot and each weight must be positive.
  //!   - NOTE: In general, it is recommended to use a vector of ones for `distance_metric_weights`
  //!           (i.e., unweighted squared L2-norm). Doing so enables significant performance
  //!           optimization for nearest neighbor searches during geometric planning. Non-unity
  //!           weights should only be considered if the benefits of the weighted distance metric
  //!           outweigh the cost of more expensive nearest neighbor searches.
  //!   - Default: Vector of ones with length set by the number of c-space coordinates in
  //!     `robot_description`.
  //!
  //! `tool_frame_name` [`std::string`]
  //!   - Indicate the name (from URDF) of the frame to be used for task space planning.
  //!   - With current implementation, setting a `tool_frame_name` that is not found in the
  //!     kinematics will throw an exception rather than failing gracefully.
  //!
  //! `task_space_limits` [`std::vector<Limit>`]
  //!   - Task space limits define a bounding box used for sampling task space when planning
  //!     a path to a task space target.
  //!   - The specified `task_space_limits` vector should be length 3, corresponding to the xyz
  //!     dimensions of the bounding box.
  //!   - Each upper limit must be >= the corresponding lower limit.
  //!   - Default: Lower limits default to -1 and upper limits to 1.
  //!
  //! `enable_self_collision_checking` [`bool`]
  //!  - Set to `true` to enable self-collision checking during motion planning.
  //!  - When enabled, configurations that result in self-collision (collision between robot
  //!    self-collision spheres) will be considered invalid during path planning.
  //!  - Default: `true`
  //!
  //! `enable_cuda_tree` [`bool`]
  //!   - Set to `true` to enable use of CUDA for storing explored configurations and performing
  //!     nearest neighbor look-up, or `false` to disable usage of CUDA.
  //!   - When `enable_cuda_tree` is set to `true`, CUDA will only be used when
  //!     `distance_metric_weights` is not equal to `Eigen::VectorXd::Ones(cspace_dimension)`.
  //!   - If set to `true` from the `false` state, default values for `cuda_tree_params` will be
  //!     used.
  //!   - If set to `false` from the `true` state, current values for `cuda_tree_params` will be
  //!     discarded (i.e., if returned to `true`, default rather than previous values will be used).
  //!   - NOTE: this parameter does NOT need to be set in the YAML configuration file. The presence
  //!     of a `cuda_tree_params` with corresponding parameters in the configuration file indicates
  //!     that `enable_cuda_tree` should be set to `true`.
  //!   - Default: `true`
  //!
  //! `cuda_tree_params/max_num_nodes` [`int`]
  //!   - DEPRECATED
  //!   - The number of configurations that can be stored in CUDA-accelerated trees is no longer
  //!     bound by a preset value. Growth is now limited by `max_iterations`, rather than the
  //!     coupling of `max_iterations` and `cuda_tree_params/max_num_nodes`.
  //!   - A deprecation warning is logged when `cuda_tree_params/max_num_nodes` is set via
  //!     `setParam()` or when present in a configuration file. It is recommended to *not*
  //!     provide a value for this parameter, as it will be ignored.
  //!
  //! `cuda_tree_params/max_buffer_size` [`int`]
  //!   - Maximum number of valid configurations that are buffered on CPU prior to transferring to
  //!     GPU.
  //!   - A default value of 30 is recommended for most use-cases.
  //!   - `cuda_tree_params/max_buffer_size` must be positive.
  //!   - Default: 30
  //!
  //! `cuda_tree_params/num_nodes_cpu_gpu_crossover` [`int`]
  //!   - Number of valid explored configurations added to tree prior to copying all configurations
  //!     to GPU and using GPU for nearest neighbor lookup.
  //!   - A default value of 3000 is recommended for most use-cases.
  //!   - `cuda_tree_params/num_nodes_cpu_gpu_crossover` must be positive.
  //!   - Default: 3000
  //!
  //!  NOTE: For all of the `cuda_tree_params` listed above, the best values for optimal performance
  //!        will depend on the number of c-space coordinates, the system hardware (e.g., CPU, GPU,
  //!        and memory configuration), and software versions (e.g., CUDA, NVIDIA GPU driver, and
  //!        cuMotion versions). The provided default recommendations are chosen to be appropriate
  //!        for most expected use-cases.
  //!
  //! `cspace_planning_params/exploration_fraction` [`double`]
  //!   - The c-space planner uses RRT-Connect to try to find a path to a c-space target.
  //!   - RRT-Connect attempts to iteratively extend two trees (one from the initial configuration
  //!     and one from the target configuration) until the two trees can be connected. The
  //!     configuration to which a tree is extended can be either a random sample
  //!     (i.e., exploration) or a node on the tree to which connection is desired
  //!     (i.e., exploitation). The `exploration_fraction` controls the fraction of steps that are
  //!     exploration steps. It is generally recommended to set `exploration_fraction` in range
  //!     [0.5, 1), where 1 corresponds to a single initial exploitation step followed by only
  //!     exploration steps. Values of between [0, 0.5) correspond to more exploitation than
  //!     exploration and are not recommended. If a value outside range [0, 1] is provided, a
  //!     warning is logged and the value is clamped to range [0, 1].
  //!   - A default value of 0.5 is recommended as a starting value for initial testing with a given
  //!     system.
  //!   - Default: 0.5
  //!
  //! `task_space_planning_params/translation_target_zone_tolerance` [`double`]
  //!   - A configuration has reached the task space translation target when task space position has
  //!     an L2 Norm within `translation_target_zone_tolerance` of the target.
  //!   - It is assumed that a valid configuration within the target tolerance can be moved directly
  //!     to the target configuration using an inverse kinematics solver and linearly stepping
  //!     towards the solution.
  //!   - In general, it is recommended that the size of the translation target zone be on the same
  //!     order of magnitude as the translational distance in task-space corresponding to moving the
  //!     robot in configuration space by one step with an L2 norm of `step_size`.
  //!   - Must be > 0.
  //!   - Default: 0.05
  //!
  //! `task_space_planning_params/orientation_target_zone_tolerance` [`double`]
  //!   - A configuration has reached the task space pose target when task space orientation is
  //!     within `orientation_target_zone_tolerance` radians and an L2 norm translation
  //!     within `translation_target_zone_tolerance` of the target.
  //!   - It is assumed that a valid configuration within the target tolerances can be moved
  //!     directly to the target configuration using an inverse kinematics solver and linearly
  //!     stepping towards the solution.
  //!   - In general, it is recommended that the size of the orientation target zone be on the same
  //!     order of magnitude as the rotational distance in task-space corresponding to moving the
  //!     robot in configuration space by one step with an L2 norm of `step_size`.
  //!   - Must be > 0.
  //!   - Default: 0.09
  //!
  //! `task_space_planning_params/translation_target_final_tolerance` [`double`]
  //!   - Once a path is found that terminates within `translation_target_zone_tolerance`, an IK
  //!     solver is used to find a configuration space solution corresponding to the task space
  //!     target. This solver terminates when the L2-norm of the corresponding task space position
  //!     is within `translation_target_final_tolerance` of the target.
  //!   - Note: This solver assumes that if a c-space configuration within
  //!     `translation_target_zone_tolerance` is found then this c-space configuration can be
  //!     moved linearly in c-space to the IK solution. If this assumption is NOT met, the returned
  //!     path will not reach the task space target within the `translation_target_final_tolerance`
  //!     and an error is logged.
  //!   - The recommended default value is 1e-4, but in general this value should be set to a
  //!     positive value that is considered "good enough" precision for the specific system.
  //!   - Default: 1e-4
  //!
  //! `task_space_planning_params/orientation_target_final_tolerance` [`double`]
  //!   - For pose targets, once a path is found that terminates within
  //!     `orientation_target_zone_tolerance` and `translation_target_zone_tolerance` of the target,
  //!     an IK solver is used to find a configuration space solution corresponding to the task
  //!     space target. This solver terminates when the L2-norm of the corresponding task space
  //!     position is within `orientation_target_final_tolerance` and
  //!     `translation_target_final_tolerance` of the target.
  //!   - Note: This solver assumes that if a c-space configuration within the target zone
  //!     tolerances is found then this c-space configuration can be moved linearly in c-space to
  //!     the IK solution. If this assumption is NOT met, the returned path will not reach the task
  //!     space target within the final target tolerances and an error is logged.
  //!   - The recommended default value is 1e-4, but in general this value should be set to a
  //!     positive value that is considered "good enough" precision for the specific system.
  //!   - Default: 0.005
  //!
  //! `task_space_planning_params/translation_gradient_weight` [`double`]
  //!   - For pose targets, computed translation and orientation gradients are linearly weighted by
  //!     `translation_gradient_weight` and `orientation_gradient_weight` to compute a combined
  //!     gradient step when using the Jacobian Transpose method to guide tree expansion
  //!     towards a task space target.
  //!   - The default value is recommended as a starting value for initial testing with a given
  //!     system.
  //!   - Must be > 0.
  //!   - Default: 1.0
  //!
  //! `task_space_planning_params/orientation_gradient_weight` [`double`]
  //!   - For pose targets, computed translation and orientation gradients are linearly weighted by
  //!     `translation_gradient_weight` and `orientation_gradient_weight` to compute a combined
  //!     gradient step when using the Jacobian Transpose method to guide tree expansion
  //!     towards a task space target.
  //!   - The default value is recommended as a starting value for initial testing with a given
  //!     system.
  //!   - Must be > 0.
  //!   - Default: 0.125
  //!
  //! `task_space_planning_params/nn_translation_distance_weight` [`double`]
  //!   - For pose targets, nearest neighbor distances are computed by linearly weighting
  //!     translation and orientation distance by `nn_translation_distance_weight` and
  //!     `nn_orientation_distance_weight`.
  //!   - Nearest neighbor search is used to select the node from which the tree of valid
  //!     configurations will be expanded.
  //!   - The default value is recommended as a starting value for initial testing with a given
  //!     system.
  //!   - Must be > 0.
  //!   - Default: 1.0
  //!
  //! `task_space_planning_params/nn_orientation_distance_weight` [`double`]
  //!   - For pose targets, nearest neighbor distances are computed by linearly weighting
  //!     translation and orientation distance by `nn_translation_distance_weight` and
  //!     `nn_orientation_distance_weight`.
  //!   - Nearest neighbor search is used to select the node from which the tree of valid
  //!     configurations will be expanded.
  //!   - The default value is recommended as a starting value for initial testing with a given
  //!     system.
  //!   - Must be > 0.
  //!   - Default: 0.125
  //!
  //! `task_space_planning_params/task_space_exploitation_fraction` [`double`]
  //!   - Fraction of iterations for which tree is extended towards target position in task space.
  //!   - Must be in range [0, 1]. Additionally, the sum of `task_space_exploitation_fraction` and
  //!     `task_space_exploration_fraction` must be <= 1.
  //!   - The default value is recommended as a starting value for initial testing with a given
  //!     system.
  //!   - Default: 0.4
  //!
  //! `task_space_planning_params/task_space_exploration_fraction` [`double`]
  //!   - Fraction of iterations for which tree is extended towards random position in task space.
  //!   - Must be in range [0, 1]. Additionally, the sum of `task_space_exploitation_fraction` and
  //!     `task_space_exploration_fraction` must be <= 1.
  //!   - The default value is recommended as a starting value for initial testing with a given
  //!     system.
  //!   - Default: 0.1
  //!
  //!  NOTE: The remaining fraction beyond `task_space_exploitation_fraction` and
  //!        `task_space_exploration_fraction` is a `cspace_exploration_fraction` that is
  //!        implicitly defined as:
  //!          1 - (`task_space_exploitation_fraction` + `task_space_exploration_fraction`
  //!        In general, easier path searches will take less time with higher exploitation fraction
  //!        while more difficult searches will waste time if the exploitation fraction is too high
  //!        and benefit from greater combined exploration fraction.
  //!
  //! `task_space_planning_params/max_extension_substeps_away_from_target` [`int`]
  //!  - Maximum number of Jacobian transpose gradient descent substeps that may be taken
  //!    while the end effector is away from the task-space target.
  //!  - The threshold for nearness is determined by the
  //!    `extension_substep_target_region_scale_factor` parameter.
  //!  - The default value is recommended as a starting value for initial testing with a given
  //!    system.
  //!  - Must be >= 0.
  //!  - Default: 6
  //!
  //! `task_space_planning_params/max_extension_substeps_near_target` [`int`]
  //!  - Maximum number of Jacobian transpose gradient descent substeps that may be taken
  //!    while the end effector is near the task-space target.
  //!  - The threshold for nearness is determined by the
  //!    `extension_substep_target_region_scale_factor` parameter.
  //!  - The default value is recommended as a starting value for initial testing with a given
  //!    system.
  //!  - Must be >= 0.
  //!  - Default: 50
  //!
  //! `task_space_planning_params/extension_substep_target_region_scale_factor` : [`double`]
  //!  - A scale factor used to determine whether the end effector is close enough to the target
  //!    to change the amount of gradient descent substeps allowed when adding a node in RRT.
  //!  - The `max_extension_substeps_near_target` parameter is used when the distance
  //!    (i.e., L2 norm) between the end effector and target position is less than
  //!    `extension_substep_target_region_scale_factor` * `x_zone_target_tolerance`.
  //!  - Must be greater than or equal to 1.0; a value of 1.0 effectively disables the
  //!   `max_extension_substeps_near_target` parameter.
  //!  - The default value is recommended as a starting value for initial testing with a given
  //!    system.
  //!  - Default: 2.0
  //!
  //! `task_space_planning_params/unexploited_nodes_culling_scalar` [`double`]
  //!  - Scalar controlling culling of unexploited nodes during task-space planning.
  //!  - Must be >= 0.0.
  //!  - Default: 1.0
  //!
  //! `task_space_planning_params/gradient_substep_size` [`double`]
  //!  - Size of each gradient-descent substep when following the Jacobian Transpose direction.
  //!  - Must be > 0.0.
  //!  - Default: 0.025
  virtual bool setParam(const std::string &param_name, ParamValue value) = 0;
};

//! Load a set of `MotionPlanner` configuration parameters from file.
//!
//! These parameters are combined with `robot_description`, `tool_frame_name`, and `world_view` to
//! create a configuration for motion planning.
//!
//! A fatal error will be logged if `motion_planner_config_file` is missing any required parameters.
//! If any provided parameter values are invalid:
//! - When recoverable, they will be clipped to their expected value range with a warning on calling
//!   `CreateMotionPlanner()`.
//! - When not recoverable, a fatal error will be logged.
//!
//! A configuration will *NOT* be created if:
//!   1. `robot_description` is invalid, *OR*
//!   2. `tool_frame_name` is not a valid frame in `robot_description`.
//! In the case of failure, a non-fatal error will be logged and a `nullptr` will be returned.
CUMO_EXPORT std::unique_ptr<MotionPlannerConfig> CreateMotionPlannerConfigFromFile(
    const std::filesystem::path &motion_planner_config_file,
    const RobotDescription &robot_description,
    const std::string &tool_frame_name,
    const WorldViewHandle &world_view);

//! Use default parameters to create a configuration for motion planning.
//!
//! See `MotionPlannerConfig::setParam()` for the default parameter values.
//!
//! A configuration will *NOT* be created if:
//!   1. `robot_description` is invalid, *OR*
//!   2. `tool_frame_name` is not a valid frame in `robot_description`.
//!
//! In the case of failure, a non-fatal error will be logged and a `nullptr` will be returned.
CUMO_EXPORT std::unique_ptr<MotionPlannerConfig> CreateDefaultMotionPlannerConfig(
    const RobotDescription &robot_description,
    const std::string &tool_frame_name,
    const WorldViewHandle &world_view);

//! Interface class for planning collision-free paths for robotic manipulators.
//! Supports both configuration space (i.e., joint position) targets and task space (e.g., end
//! effector position) targets.
class CUMO_EXPORT MotionPlanner {
 public:
  virtual ~MotionPlanner() = default;

  //! Results from a path search.
  struct CUMO_EXPORT Results {
    //! Indicate whether a collision-free path was found.
    //! If `false`, `path` and `interpolated_path` will be empty.
    bool path_found;

    //! Minimal representation of collision-free path.
    //! Each vector represents a knot in configuration space, where successive knots can be linearly
    //! interpolated in configuration space to generate a collision-free path.
    std::vector<Eigen::VectorXd> path;

    //! Interpolation of `path` such that successive knots are (in general) one `step_size` apart
    //! from each other (where distance is defined as L2-norm in c-space).
    //! NOTE: Each straight segment from `path` above are interpolated individually, where the
    //!       distance between the last two returned knots for each segment will be less than one
    //!       `step_size` from each other.
    //! The `interpolated_path` will only be populated if `generate_interpolated_path` is set to
    //! `true` when generating a path plan.
    std::vector<Eigen::VectorXd> interpolated_path;
  };

  //! Attempt to find a path from initial configuration space position (`q_initial`) to a
  //! configuration space target (`q_target`).
  //! `generate_interpolated_path` determines whether `interpolated_path` will be populated in
  //! `Results`.
  virtual Results planToCSpaceTarget(const Eigen::VectorXd &q_initial,
                                     const Eigen::VectorXd &q_target,
                                     bool generate_interpolated_path = false) = 0;

  //! Attempt to find a path from initial configuration space position (`q_initial`) to a task
  //! space translation target (`translation_target`).
  //! `generate_interpolated_path` determines whether `interpolated_path` will be populated in
  //! `Results`.
  virtual Results planToTranslationTarget(const Eigen::VectorXd &q_initial,
                                          const Eigen::Vector3d &translation_target,
                                          bool generate_interpolated_path = false) = 0;

  //! Attempt to find a path from initial configuration space position (`q_initial`) to a task
  //! space pose target (`pose_target`).
  //! `generate_interpolated_path` determines whether `interpolated_path` will be populated in
  //! `Results`.
  virtual Results planToPoseTarget(const Eigen::VectorXd &q_initial,
                                   const Pose3 &pose_target,
                                   bool generate_interpolated_path = false) = 0;

  //! Reset all internal state so that subsequent planning calls produce the same results as they
  //! did immediately after construction, given the same inputs.
  virtual void reset() = 0;
};

//! Create a `MotionPlanner` with the given `config`.
CUMO_EXPORT std::unique_ptr<MotionPlanner> CreateMotionPlanner(const MotionPlannerConfig &config);

}  // namespace cumotion
