// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES.
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
//! @brief Public interface for loading and accessing a robot description

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"

namespace cumotion {

//! Forward declaration of Kinematics from `cumotion/kinematics.h`.
class Kinematics;

//! Class encapsulating all semantic and kinematic properties of a robot
class CUMO_EXPORT RobotDescription {
 public:
  virtual ~RobotDescription() = default;

  //! Return the number of actuated joints for the robot.
  [[nodiscard]] virtual int numCSpaceCoords() const = 0;

  //! Return the name of a given joint of the robot.
  [[nodiscard]] virtual std::string cSpaceCoordName(int coord_index) const = 0;

  //! Return default joint positions for the robot.
  //!
  //! Returned vector will have length equal to `numCSpaceCoords()`.
  [[nodiscard]] virtual Eigen::VectorXd defaultCSpaceConfiguration() const = 0;

  //! Return a copy of robot kinematics.
  [[nodiscard]] virtual std::unique_ptr<Kinematics> kinematics() const = 0;

  //! Return the names of all tool frames (if any) specified in the robot description.
  [[nodiscard]] virtual std::vector<std::string> toolFrameNames() const = 0;
};

//! Load a robot description from an XRDF (`robot_xrdf`) and a URDF (`robot_urdf`).
//!
//! It is recommended that `robot_xrdf` and `robot_urdf` be specified as absolute filepaths.
//! Relative paths will be resolved using the same rules as the `std::filesystem::path` type.
//!
//! The "Extended Robot Description Format" (XRDF) is documented at:
//! https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/manipulation/xrdf.html
//!
//! A fatal error will be logged if:
//! - `robot_xrdf` is not a valid file path,
//! - `robot_urdf` is not a valid file path,
//! - `robot_xrdf` cannot be successfully parsed, *OR*
//! - `robot_urdf` cannot be successfully parsed.
[[nodiscard]] CUMO_EXPORT std::unique_ptr<RobotDescription> LoadRobotFromFile(
    const std::filesystem::path &robot_xrdf,
    const std::filesystem::path &robot_urdf);

//! Load a robot description from the contents of an XRDF ("robot_xrdf") and the contents
//! of a URDF ("robot_urdf").
//!
//! The "Extended Robot Description Format" (XRDF) is documented at:
//! https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/manipulation/xrdf.html
[[nodiscard]] CUMO_EXPORT std::unique_ptr<RobotDescription> LoadRobotFromMemory(
    const std::string &robot_xrdf,
    const std::string &robot_urdf);

}  // namespace cumotion
