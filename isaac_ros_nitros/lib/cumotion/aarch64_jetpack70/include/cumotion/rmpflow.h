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
//! @brief Public interface to cuMotion's RMPflow implementation

#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"
#include "cumotion/robot_description.h"
#include "cumotion/rotation3.h"
#include "cumotion/world.h"

namespace cumotion {

//! Interface class for loading and manipulating RMPflow parameters.
//! WARNING: This interface may change in a future release.
class CUMO_EXPORT RmpFlowConfig {
 public:
  virtual ~RmpFlowConfig() = default;

  //! Get the value of a parameter, given a "param_name" string of the form
  //! "<rmp_name>/<parameter_name>"
  [[nodiscard]] virtual double getParam(const std::string &param_name) const = 0;

  //! Set the value of the parameter.
  virtual void setParam(const std::string &param_name, double value) = 0;

  //! Get the names and values of all parameters.  The two vectors will be overwritten if not empty.
  virtual void getAllParams(std::vector<std::string> &names, std::vector<double> &values) const = 0;

  //! Set all parameters at once.  The vectors "names" and "values" must have the same size.
  //! The parameter corresponding to names[i] will be set to the value given by values[i].
  virtual void setAllParams(const std::vector<std::string> &names,
                            const std::vector<double> &values) = 0;

  //! Set the world view that will be used for obstacle avoidance. All enabled obstacles in
  //! `world_view` will be avoided by the RMPflow policy.
  virtual void setWorldView(const WorldViewHandle &world_view) = 0;
};

//! Load a set of RMPflow parameters from file, and combine with a robot description to create a
//! configuration object for consumption by CreateRmpFlow().  The "end_effector_frame" should
//! correspond to a link name as specified in the original URDF or added as a frame in the XRDF used
//! to create the robot description. All enabled obstacles in `world_view` will be avoided by the
//! RMPflow policy.
//!
//! DEPRECATED: This function is deprecated and will be removed in a future version.
//!             Use the variant of `CreateRmpFlowConfigFromFile()` that does *not* include
//!             `end_effector_frame` instead. The end effector frame can then be added using
//!             `RmpFlow::addTargetFrame()`.
CUMO_DEPRECATED_EXPORT std::unique_ptr<RmpFlowConfig> CreateRmpFlowConfigFromFile(
    const std::filesystem::path &rmpflow_config_file,
    const RobotDescription &robot_description,
    const std::string &end_effector_frame,
    const WorldViewHandle &world_view);

//! Load a set of RMPflow parameters from file, and combine with a robot description to create a
//! configuration object for consumption by CreateRmpFlow(). All enabled obstacles in `world_view`
//! will be avoided by the RMPflow policy.
CUMO_EXPORT std::unique_ptr<RmpFlowConfig> CreateRmpFlowConfigFromFile(
    const std::filesystem::path &rmpflow_config_file,
    const RobotDescription &robot_description,
    const WorldViewHandle &world_view);

//! Load a set of RMPflow parameters from string, and combine with a robot description to create a
//! configuration object for consumption by CreateRmpFlow().  The "end_effector_frame" should
//! correspond to a link name as specified in the original URDF or added as a frame in the XRDF used
//! to create the robot description. All enabled obstacles in `world_view` will be avoided by the
//! RMPflow policy.
//!
//! DEPRECATED: This function is deprecated and will be removed in a future version.
//!             Use the variant of `CreateRmpFlowConfigFromMemory()` that does *not* include
//!             `end_effector_frame` instead. The end effector frame can then be added using
//!             `RmpFlow::addTargetFrame()`.
CUMO_DEPRECATED_EXPORT std::unique_ptr<RmpFlowConfig> CreateRmpFlowConfigFromMemory(
    const std::string &rmpflow_config,
    const RobotDescription &robot_description,
    const std::string &end_effector_frame,
    const WorldViewHandle &world_view);

//! Load a set of RMPflow parameters from string, and combine with a robot description to create a
//! configuration object for consumption by CreateRmpFlow(). All enabled obstacles in `world_view`
//! will be avoided by the RMPflow policy.
CUMO_EXPORT std::unique_ptr<RmpFlowConfig> CreateRmpFlowConfigFromMemory(
    const std::string &rmpflow_config,
    const RobotDescription &robot_description,
    const WorldViewHandle &world_view);

//! Interface class for building and evaluating a motion policy in the RMPflow framework
class CUMO_EXPORT RmpFlow {
 public:
  virtual ~RmpFlow() = default;

  //! Parameters to configure task-space RMPs.
  //!
  //! For detailed description of all parameters, see RMPflow documentation
  //! (https://nvidia-isaac.github.io/cumotion/concepts/rmpflow.html) and associated tuning guide
  //! (https://nvidia-isaac.github.io/cumotion/concepts/rmpflow_tuning_guide.html). This
  //! guide provides a mathematical framework for the target RMP and defines variables used
  //! to document each of the parameters in `TargetRmpConfig`.
  struct CUMO_EXPORT TargetRmpConfig {
    //! Create a `TargetRmpConfig` using default parameters.
    TargetRmpConfig();

    //! Create a `TargetRmpConfig` using parameters from `rmpflow_config`.
    explicit TargetRmpConfig(const RmpFlowConfig &rmpflow_config);

    //! Parameters to configure a task-space position RMP.
    struct CUMO_EXPORT PositionConfig {
      //! Position gain.
      //!
      //! Units: m/s^2
      //! Must be positive.
      double accel_p_gain{80.0};

      //! Damping gain.
      //!
      //! Units: s^-1
      //! Must be positive.
      double accel_d_gain{120.0};

      //! Length scale controlling transition between constant acceleration region far from target
      //! and linear region near target.
      //!
      //! Units: m
      //! Must be positive.
      double accel_norm_eps{0.075};

      //! Length scale of the Gaussian controlling blending between `S` and `I`.
      //!
      //! Units: m
      //! Must be positive.
      double metric_alpha_length_scale{0.05};

      //! Controls the minimum contribution of the isotropic `M_near` term to the metric (inertia
      //! matrix).
      //!
      //! Units: dimensionless
      //! Must be positive.
      double min_metric_alpha{0.01};

      //! Metric scalar for the isotropic `M_near` contribution to the metric (inertia matrix).
      //!
      //! Units: dimensionless
      //! Must be positive.
      double max_metric_scalar{10000.0};

      //! Metric scalar for the directional `M_far` contribution to the metric (inertia matrix).
      //!
      //! Units: dimensionless
      //! Must be positive.
      double min_metric_scalar{2500.0};

      //! Scale factor controlling the strength of boosting near the target.
      //!
      //! Units: dimensionless
      //! Must be positive.
      double proximity_metric_boost_scalar{20.0};

      //! Length scale of the Gaussian controlling boosting near the target.
      //!
      //! Units: m
      //! Must be positive.
      double proximity_metric_boost_length_scale{0.02};
    };

    //! Parameters to configure a task-space orientation RMP.
    struct CUMO_EXPORT OrientationConfig {
      //! Position gain.
      //!
      //! Units: s^-2
      //! Must be positive.
      double accel_p_gain{200.0};

      //! Damping gain.
      //!
      //! Units: s^-1
      //! Must be positive.
      double accel_d_gain{40.0};

      //! Priority weight relative to other RMPs.
      //!
      //! Units: dimensionless
      //! Must be positive.
      double metric_scalar{10.0};

      //! Scale factor controlling the strength of boosting near the position target.
      //!
      //! Units: dimensionless
      //! Must be positive.
      double proximity_metric_boost_scalar{3000.0};

      //! Length scale of the Gaussian controlling boosting near the position target.
      //!
      //! Units: m
      //! Must be positive.
      double proximity_metric_boost_length_scale{0.05};
    };

    //! Parameters to configure damping for task-space RMPs.
    struct CUMO_EXPORT DampingConfig {
      //! Nonlinear damping gain.
      //!
      //! Units: m^-1
      //! Must be positive.
      double accel_d_gain{30.0};

      //! Priority weight relative to other RMPs.
      //!
      //! Units: (m/s)^-1
      //! Must be positive.
      double metric_scalar{50.0};

      //! Additional inertia.
      //!
      //! Units: dimensionless
      //! Must be positive.
      double inertia{100.0};
    };

    //! Parameters to configure a task-space position RMP.
    PositionConfig position_config;

    //! Parameters to configure a task-space orientation RMP.
    OrientationConfig orientation_config;

    //! Parameters to configure damping for task-space RMPs.
    DampingConfig damping_config;
  };

  //! Return the number of task-space target frames.
  //!
  //! This count includes both "active" target frames (i.e., frames with position and/or orientation
  //! targets set) as well as "inactive" target frames (i.e., frames for which no position or
  //! orientation targets have been set). This count does *not* include any target frames that have
  //! been removed.
  [[nodiscard]] virtual int numTargetFrames() const = 0;

  //! Return the names of all frames with task-space targets.
  //!
  //! These names include both "active" target frames (i.e., frames with position and/or orientation
  //! targets set) as well as "inactive" target frames (i.e., frames for which no position or
  //! orientation targets have been set). These names do *not* include any target frames that have
  //! been removed.
  [[nodiscard]] virtual std::vector<std::string> targetFrameNames() const = 0;

  //! Enable a task-space target for the frame corresponding to `frame_name`.
  //!
  //! The target may be configured using the optional `config`. If `config` is not provided, then
  //! configuration parameters are taken from the `RmpFlowConfig` originally used to create the
  //! `RmpFlow` object.
  //!
  //! NOTE: The task-space target will not be active until a position and/or orientation target
  //!       is set.
  //!
  //! A fatal error will be logged if:
  //!   1. `frame_name` is not a valid frame in the `RobotDescription` used to construct `RmpFlow`,
  //!   2. `frame_name` is already in use as a target frame (i.e., this frame has been added and
  //!      not removed). This includes the `end_effector_frame` (optionally) used to construct the
  //!      `RmpflowConfig` used create this `RmpFlow` instance, *OR*
  //!   3. Any parameter in `config` is invalid.
  virtual void addTargetFrame(const std::string &frame_name,
                              std::optional<TargetRmpConfig> config = std::nullopt) = 0;

  //! Remove the task-space target for the frame corresponding to `frame_name`.
  //!
  //! A fatal error will be logged if:
  //!   1. `frame_name` does not correspond to a task-space target frame.
  virtual void removeTargetFrame(const std::string &frame_name) = 0;

  //! Set both a position and orientation target for `frame_name`.
  //!
  //! A fatal error will be logged if:
  //!   1. `frame_name` does not correspond to a task-space target frame.
  virtual void setPoseTarget(const std::string &frame_name, const Pose3 &pose) = 0;

  //! Clear both the position and orientation target for `frame_name`.
  //!
  //! This function will be a no-op if neither a position nor an orientation target is active for
  //! `frame_name`.
  //!
  //! A fatal error will be logged if:
  //!   1. `frame_name` does not correspond to a task-space target frame.
  virtual void clearPoseTarget(const std::string &frame_name) = 0;

  //! Set a position target for `frame_name`.
  //!
  //! A fatal error will be logged if:
  //!   1. `frame_name` does not correspond to a task-space target frame.
  virtual void setPositionTarget(const std::string &frame_name,
                                 const Eigen::Vector3d &position) = 0;

  //! Clear the position target for `frame_name`.
  //!
  //! This function will be a no-op if a position target is not active for `frame_name`.
  //!
  //! A fatal error will be logged if:
  //!   1. `frame_name` does not correspond to a task-space target frame.
  virtual void clearPositionTarget(const std::string &frame_name) = 0;

  //! Set an orientation target for `frame_name`.
  //!
  //! A fatal error will be logged if:
  //!   1. `frame_name` does not correspond to a task-space target frame.
  virtual void setOrientationTarget(const std::string &frame_name,
                                    const Rotation3 &orientation) = 0;

  //! Clear the orientation target for `frame_name`.
  //!
  //! This function will be a no-op if an orientation target is not active for `frame_name`.
  //!
  //! A fatal error will be logged if:
  //!   1. `frame_name` does not correspond to a task-space target frame.
  virtual void clearOrientationTarget(const std::string &frame_name) = 0;

  //! Set an end-effector position attractor.
  //!
  //! The origin of the end effector frame will be driven towards the specified position.
  //!
  //! The "end-effector" is defined as the first task-space target (i.e., the first frame name
  //! returned from `targetFrameNames()`). A fatal error will be logged if there are no target
  //! frames (i.e., `numTargetFrames()` == 0).
  //!
  //! DEPRECATED: This function is deprecated and will be removed in a future release.
  //!             Use `setPositionTarget()` instead.
  CUMO_DEPRECATED virtual void setEndEffectorPositionAttractor(const Eigen::Vector3d &position) = 0;

  //! Clear end-effector position attractor.
  //!
  //! The RMP driving the origin of the end effector frame towards a particular position will be
  //! deactivated.
  //!
  //! The "end-effector" is defined as the first task-space target (i.e., the first frame name
  //! returned from `targetFrameNames()`). A fatal error will be logged if there are no target
  //! frames (i.e., `numTargetFrames()` == 0).
  //!
  //! DEPRECATED: This function is deprecated and will be removed in a future release.
  //!             Use `clearPositionTarget()` instead.
  CUMO_DEPRECATED virtual void clearEndEffectorPositionAttractor() = 0;

  //! Set an end-effector orientation attractor.
  //!
  //! The orientation of the end effector frame will be driven towards the specified orientation.
  //!
  //! The "end-effector" is defined as the first task-space target (i.e., the first frame name
  //! returned from `targetFrameNames()`). A fatal error will be logged if there are no target
  //! frames (i.e., `numTargetFrames()` == 0).
  //!
  //! DEPRECATED: This function is deprecated and will be removed in a future release.
  //!             Use `setOrientationTarget()` instead.
  CUMO_DEPRECATED virtual void setEndEffectorOrientationAttractor(const Rotation3 &orientation) = 0;

  //! Clear end-effector orientation attractor.
  //!
  //! The RMPs driving the orientation of the end effector frame towards a particular orientation
  //! will be deactivated.
  //!
  //! The "end-effector" is defined as the first task-space target (i.e., the first frame name
  //! returned from `targetFrameNames()`). A fatal error will be logged if there are no target
  //! frames (i.e., `numTargetFrames()` == 0).
  //!
  //! DEPRECATED: This function is deprecated and will be removed in a future release.
  //!             Use `clearOrientationTarget()` instead.
  CUMO_DEPRECATED virtual void clearEndEffectorOrientationAttractor() = 0;

  //! Set an attractor in generalized coordinates (configuration space).
  //!
  //! The c-space coordinates will be biased towards the specified configuration.
  //!
  //! NOTE:  Unlike the end effector attractors, there is always an active c-space attractor (either
  //!        set using `setCSpaceAttractor()` or using the default value loaded from the robot
  //!        description).
  virtual void setCSpaceAttractor(const Eigen::VectorXd &cspace_position) = 0;

  //! Compute configuration-space acceleration from motion policy, given input state. This takes
  //! into account the current c-space and/or task-space targets, as well as any
  //! currently-enabled obstacles.
  virtual void evalAccel(const Eigen::VectorXd &cspace_position,
                         const Eigen::VectorXd &cspace_velocity,
                         Eigen::Ref<Eigen::VectorXd> cspace_accel) const = 0;

  //! Compute configuration-space force and metric from motion policy, given input state. This takes
  //! into account the current c-space and/or task-space targets, as well as any
  //! currently-enabled obstacles.
  virtual void evalForceAndMetric(const Eigen::VectorXd &cspace_position,
                                  const Eigen::VectorXd &cspace_velocity,
                                  Eigen::Ref<Eigen::VectorXd> cspace_force,
                                  Eigen::Ref<Eigen::MatrixXd> cspace_metric) const = 0;
};

//! Create an instance of the RmpFlow interface from an RMPflow configuration.
CUMO_EXPORT std::unique_ptr<RmpFlow> CreateRmpFlow(const RmpFlowConfig &config);

}  // namespace cumotion
