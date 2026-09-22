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

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"

namespace cumotion {

//! Represent a path through configuration space (i.e., "c-space") parameterized by time.
class CUMO_EXPORT Trajectory {
 public:
  virtual ~Trajectory() = default;

  //! Return the number of configuration space coordinates for the kinematic structure.
  [[nodiscard]] virtual int numCSpaceCoords() const = 0;

  //! Indicates the continuous range of time values, `t`, for which the trajectory and its
  //! derivatives are defined.
  struct CUMO_EXPORT Domain {
    //! Minimum value of time defining domain.
    double lower;

    //! Maximum value of time defining domain.
    double upper;

    //! Convenience function to return the span of time values included in domain.
    [[nodiscard]] double span() const { return upper - lower; }
  };

  //! Return the domain for the trajectory.
  [[nodiscard]] virtual Domain domain() const = 0;

  //! Evaluate the trajectory at the given `time`.
  virtual void eval(double time,
                    Eigen::VectorXd *cspace_position,
                    Eigen::VectorXd *cspace_velocity = nullptr,
                    Eigen::VectorXd *cspace_acceleration = nullptr,
                    Eigen::VectorXd *cspace_jerk = nullptr) const = 0;

  //! Evaluate specified trajectory derivative at the given `time` and return value.
  //!
  //! The default `derivative_order` is the "zeroth derivative" (which is simply the c-space
  //! position of the trajectory). Setting `derivative_order` to 1 will output the c-space velocity,
  //! with higher `derivative_order`s corresponding to higher derivatives.
  //!
  //! Trajectories are expected to support at least the third derivative (i.e., "jerk").
  //!
  //! The default implementation internally calls the above `eval()` function and will log a fatal
  //! error if the `derivative_order` is not in range [0, 3].
  [[nodiscard]] virtual Eigen::VectorXd eval(double time, int derivative_order = 0) const;

  //! Return the minimum position for each c-space coordinate within the defined `domain()`.
  //!
  //! NOTE: These minimum position values are not, in general, synchronous. Instead, they represent
  //!       the minimum position independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::VectorXd minPosition() const = 0;

  //! Return the maximum position for each c-space coordinate within the defined `domain()`.
  //!
  //! NOTE: These maximum position values are not, in general, synchronous. Instead, they represent
  //!       the maximum position independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::VectorXd maxPosition() const = 0;

  //! Return the maximum magnitude of velocity for each c-space coordinate within the defined
  //! `domain()`.
  //!
  //! NOTE: These maximum magnitude values are not, in general, synchronous. Instead, they represent
  //!       the maximum magnitudes independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::VectorXd maxVelocityMagnitude() const = 0;

  //! Return the maximum magnitude of acceleration for each c-space coordinate within the defined
  //! `domain()`.
  //!
  //! NOTE: These maximum magnitude values are not, in general, synchronous. Instead, they represent
  //!       the maximum magnitudes independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::VectorXd maxAccelerationMagnitude() const = 0;

  //! Return the maximum magnitude of jerk for each c-space coordinate within the defined
  //! `domain()`.
  //!
  //! NOTE: These maximum magnitude values are not, in general, synchronous. Instead, they represent
  //!       the maximum magnitudes independently achieved by each coordinate.
  [[nodiscard]] virtual Eigen::VectorXd maxJerkMagnitude() const = 0;
};

}  // namespace cumotion
