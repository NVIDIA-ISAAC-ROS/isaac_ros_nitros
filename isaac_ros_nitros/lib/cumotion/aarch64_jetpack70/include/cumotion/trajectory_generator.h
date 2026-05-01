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
#include <string>
#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"
#include "cumotion/kinematics.h"
#include "cumotion/trajectory.h"

namespace cumotion {

//! Configure a trajectory generator that can compute smooth* trajectories. A trajectory generation
//! problem is specified by:
//!
//!   1. Bound constraints (position, velocity, and acceleration),
//!   2. Intermediate position waypoints, and
//!   3. Limits on some combination of position, velocity, acceleration and jerk.
//!
//! Times for domain bounds and intermediate waypoints are *not* specified. It is assumed that a
//! time-optimal trajectory that respects the constraints (with the definition of time-optimality
//! being implementation-specific) will be generated.
//!
//! * The definition of "smooth" is implementation-specific. Currently, `CSpaceTrajectoryGenerator`
//!   uses a single implementation that interpolates through waypoints using a series of cubic
//!   splines. For this implementation, "smooth" is defined as smooth velocity, continuous
//!   acceleration, and bounded jerk.
class CUMO_EXPORT CSpaceTrajectoryGenerator {
 public:
  virtual ~CSpaceTrajectoryGenerator() = default;

  //! Return the number of configuration space coordinates for the trajectory generator.
  [[nodiscard]] virtual int numCSpaceCoords() const = 0;

  //! Set position limits.
  //!
  //! A fatal error is logged if either `min_position` or `max_position` does not have a length
  //! matching the expected number of c-space coordinates, or if any coordinate of
  //! `max_position` is not greater than the corresponding coordinate of `min_position`.
  virtual void setPositionLimits(const Eigen::VectorXd &min_position,
                                 const Eigen::VectorXd &max_position) = 0;

  //! Set velocity magnitude limits.
  //!
  //! A fatal error is logged if `max_velocity` does not have a length matching the expected number
  //! of c-space coordinates, or if any coordinate of `max_velocity` is negative.
  //!
  //! WARNING: The current implementation using a series of cubic splines requires velocity limits
  //!          to be specified. This restriction may be lifted in future versions of the
  //!          `CSpaceTrajectoryGenerator`.
  virtual void setVelocityLimits(const Eigen::VectorXd &max_velocity) = 0;

  //! Set acceleration magnitude limits.
  //!
  //! A fatal error is logged if `max_acceleration` does not have a length matching the expected
  //! number of c-space coordinates, or if any coordinate of `max_acceleration` is negative.
  virtual void setAccelerationLimits(const Eigen::VectorXd &max_acceleration) = 0;

  //! Set jerk magnitude limits.
  //!
  //! A fatal error is logged if `max_jerk` does not have a length matching the expected
  //! number of c-space coordinates, or if any coordinate of `max_jerk` is negative.
  virtual void setJerkLimits(const Eigen::VectorXd &max_jerk) = 0;

  //! Attempt to generate a time-optimal trajectory passing through the specified `waypoints` with
  //! the specified constraints.
  //!
  //! If a trajectory cannot be generated, `nullptr` is returned.
  [[nodiscard]] virtual std::unique_ptr<Trajectory> generateTrajectory(
      const std::vector<Eigen::VectorXd> &waypoints) const = 0;

  //! Interpolation modes used by `interpolateTrajectory()`.
  enum class InterpolationMode {
    //! Linear interpolation between c-space positions.
    //!
    //! The resulting trajectory will have continuous (but not smooth) position and discontinuous
    //! segments of constant velocity. Acceleration and jerk will be returned as zero (including at
    //! velocity discontinuities where values are technically undefined).
    LINEAR,
    //! Piecewise solution with a cubic spline between adjacent c-space positions.
    //!
    //! The resulting trajectory will have smooth position and velocity, continuous (but not smooth)
    //! acceleration, and bounded (but discontinuous) jerk.
    CUBIC_SPLINE
  };

  //! Attempt to interpolate a trajectory passing through the specified `waypoints` at the specified
  //! `times`.
  //!
  //! Interpolation will fail if:
  //! 1. The number of specified `positions` does not match the number of specified `times`;
  //! 2. There is not at least one position specified for `LINEAR` interpolation mode or at least
  //!    two positions specified for `CUBIC_SPLINE` interpolation mode;
  //! 3. Not all `positions` have the same number of c-space coordinates;
  //! 4. Not all `times` are strictly increasing; *OR*
  //! 5. The interpolated trajectory does not satisfy specified limits on c-space position,
  //!    velocity, acceleration, or jerk.
  //!
  //! If a trajectory cannot be generated due to invalid input (items 1-4 above), a fatal error is
  //! logged. If failure is due to c-space limits (item 5 above), then `nullptr` is returned.
  [[nodiscard]] virtual std::unique_ptr<Trajectory> generateTimeStampedTrajectory(
      const std::vector<Eigen::VectorXd> &waypoints,
      const std::vector<double> &times,
      InterpolationMode interpolation_mode = InterpolationMode::CUBIC_SPLINE) const = 0;

  //================================================================================================
  // Interface for setting implementation-specific solver parameters.
  // NOTE: Default values are likely to be sufficient for most use-cases.

  //! Specify the value for a given parameter.
  //!
  //! The required `SolverParamValue` constructor for each parameter is detailed in the
  //! documentation for `setSolverParam()`.
  struct CUMO_EXPORT SolverParamValue {
    //! Create `SolverParamValue` from `int`.
    SolverParamValue(int value);  // NOLINT Allow implicit conversion
    //! Create `SolverValue` from `double`.
    SolverParamValue(double value);  // NOLINT Allow implicit conversion
    //! Create `SolverParamValue` from `const char*`.
    SolverParamValue(const char *value);  // NOLINT Allow implicit conversion
    //! Create `SolverParamValue` from `std::string`.
    SolverParamValue(const std::string &value);  // NOLINT Allow implicit conversion

    struct Impl;
    std::shared_ptr<Impl> impl;
  };

  //! Set the value of the solver parameter.
  //!
  //! Currently, `CSpaceTrajectoryGenerator` uses a single solver implementation based on computing
  //! a series of cubic splines. The following parameters can be set for this piecewise cubic spline
  //! solver:
  //!
  //! `max_segment_iterations` [`int`]
  //!   - The first step towards finding a time-optimal trajectory using a series of cubic
  //!     splines is to iteratively compute an optimal span for each spline segment.
  //!   - Setting a relatively high `max_segment_iterations` will, in general, result in shorter
  //!     trajectory time spans, but will tend to take longer to converge.
  //!   - This step can be skipped completely by setting `max_segment_iterations` to zero.
  //!   - `max_segment_iterations` must be non-negative. Additionally at least one of
  //!     `max_segment_iterations` and `max_aggregate_iterations` must be non-zero.
  //!   - Default value is 5.
  //!
  //! `max_aggregate_iterations` [`int`]
  //!   - The second step towards finding a time-optimal trajectory using a series of cubic splines
  //!     is to iteratively compute an optimal span for the entire trajectory.
  //!   - Relying more heavily on this second step (i.e., setting `max_segment_iterations` to a
  //!     a relatively low value) will, in general, require less iterations (and thus less time) to
  //!     converge, but the generated trajectories will tend to have longer time spans.
  //!   - This step can be skipped completely by setting `max_aggregate_iterations` to zero.
  //!   - `max_aggregate_iterations` must be non-negative. Additionally at least one of
  //!     `max_segment_iterations` and `max_aggregate_iterations` must be non-zero.
  //!   - Default value is 50.
  //!
  //! `convergence_dt` [`double`]
  //!   - The search for optimal time values will terminate if the maximum change to any time value
  //!     during a given iteration is less than the `convergence_dt`.
  //!   - `convergence_dt` must be positive.
  //!   - Default value is 0.001.
  //!
  //! `max_dilation_iterations` [`int`]
  //!   - After the segment-wise and/or aggregate time-optimal search has converged or reached
  //!     maximum iterations, the resulting set of splines will be tested to see if any derivative
  //!     limits are exceeded.
  //!   - If any derivative limits are exceeded, the splines will be iteratively scaled in time to
  //!     reduce the maximum achieved derivative. This process will repeat until no derivative
  //!     limits are exceeded (success) or `max_dilation_iterations` are reached (failure).
  //!   - `max_dilation_iterations` must be non-negative.
  //!   - Default value is 100.
  //!
  //! `dilation_dt` [`double`]
  //!   - For the iterative dilation step described in `max_dilation_iterations` documentation, the
  //!     `dilation_dt` is the "epsilon" value added to the span of the trajectory that exceeds
  //!     derivative limits.
  //!   - `dilation_dt` must be positive.
  //!   - Default value is 0.001.
  //!
  //! `min_time_span` [`double`]
  //!   - Specify the minimum allowable time span between adjacent waypoints/endpoints.
  //!   - `min_time_span` must be positive.
  //!   - Default value is 0.01.
  //!
  //! `time_split_method` [`string`]
  //!   - Specify the `TimeSplitMethod` for the initial distribution of time values that will be
  //!     used to iteratively search for time-optimal values.
  //!   - Valid settings are `uniform`, `chord_length` and `centripetal`.
  [[nodiscard]]
  virtual bool setSolverParam(const std::string &param_name, SolverParamValue value) = 0;
};

//! Create a `CSpaceTrajectoryGenerator` with the specified number of configuration space
//! coordinates.
//!
//! Position and derivative limits, bound constraints, and intermediate waypoints can be added
//! after construction.
CUMO_EXPORT std::unique_ptr<CSpaceTrajectoryGenerator> CreateCSpaceTrajectoryGenerator(
    int num_cspace_coords);

//! Create a `CSpaceTrajectoryGenerator` with the specified `kinematics`.
//!
//! The `kinematics` will be used to specify the number of configuration space coordinates, position
//! limits, and any available derivative limits.
//!
//! Position and derivative limits may be added or overwritten after construction. Bound
//! constraints and intermediate waypoints can be added after construction.
CUMO_EXPORT std::unique_ptr<CSpaceTrajectoryGenerator> CreateCSpaceTrajectoryGenerator(
    const Kinematics &kinematics);

}  // namespace cumotion
