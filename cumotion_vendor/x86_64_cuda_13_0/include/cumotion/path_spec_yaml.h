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

#include <filesystem>
#include <memory>
#include <string>

#include "cumotion/composite_path_spec.h"
#include "cumotion/cspace_path_spec.h"
#include "cumotion/cumotion_export.h"
#include "cumotion/task_space_path_spec.h"

namespace cumotion {

//! Load a `TaskSpacePathSpec` from file with absolute path `task_space_path_spec_file`.
//!
//! The `task_space_spec_file` is expected to correspond to a YAML file specifying a task space
//! path. This path specification must include:
//!   [1] an initial task space pose, and
//!   [2] a series of path segments emanating from this initial pose.
//!
//! The initial pose uses the key "initial pose" and is expected to have the following format:
//!
//!   initial pose:
//!     position: [#, #, #]
//!     orientation: {w: #, xyz: [#, #, #]}
//!
//! where each "#" represents a floating point value. Position must be a 3d-vector and orientation
//! is represented by a quaternion.
//!
//! NOTE: Standard YAML formatting is allowed, wherein:
//!   * mapped keys can be included in any order (e.g., "position" and "orientation" can be
//!     included in either order),
//!   * mapped keys can be included on a single comma-separated line with curly braces, or
//!     expanded to multiple lines. E.g., "orientation" can be formatted as:
//!        orientation: {w: #, xyz: [#, #, #]}
//!     or equivalently as:
//!        orientation:
//!          w: #
//!          xyz: [#, #, #]
//!   * sequences can be included in a single comma-separated line with square brackets, or
//!     expanded into multiple lines with preceding dashes. E.g., "position" can be formatted as:
//!        position: [#, #, #]
//!     or equivalently as:
//!        position:
//!          - #
//!          - #
//!          - #
//!
//! The path segments are specified as a sequence under the key "path specs". Each path
//! specification must specify both a "position mode" and a "orientation mode".
//!
//! The "position mode" must be:
//!   "linear",
//!   "constant",
//!   "three_point_arc", or
//!   "tangent_arc".
//! The "orientation mode" must be:
//!   "slerp",
//!   "constant", or
//!   "tangent".
//!
//! WARNING: Not all combinations of "position mode" and "orientation mode" are compatible.
//! Specifically, a "linear" position mode is *NOT* compatible with a "tangent" orientation mode.
//! A "constant" position mode is *ONLY* compatible with a "slerp" orientation mode. The two arc
//! position modes ("three_point_arc" and "tangent_arc") are compatible with all orientation modes.
//!
//! Each (valid) combination of position and orientation mode requires specific input data:
//!
//! [1] Linear path:
//!     - "linear" position mode with "slerp" orientation mode.
//!     - Requires "target pose" and optional "blend radius".
//!     - See `TaskSpacePathSpec::addLinearPath()` in `task_space_path_spec.h` for more
//!       documentation.
//!     - Format:
//!           - position mode: linear
//!             orientation mode: slerp
//!             target pose:
//!               position: [#, #, #]
//!               orientation: {w: #, xyz: [#, #, #]}
//!             blend radius: #
//! [2] Translation path:
//!     - "linear" position mode with "constant" orientation mode.
//!     - Requires "target position" and optional "blend radius".
//!     - See `TaskSpacePathSpec::addTranslation()` in `task_space_path_spec.h` for more
//!       documentation.
//!     - Format:
//!           - position mode: linear
//!             orientation mode: constant
//!             target position: [#, #, #]
//!             blend radius: #
//! [3] Rotation path:
//!     - "constant" position mode with "slerp" orientation mode.
//!     - Requires "target orientation" .
//!     - See `TaskSpacePathSpec::addRotation()` in `task_space_path_spec.h` for more
//!       documentation.
//!     - Format:
//!           - position mode: constant
//!             orientation mode: slerp
//!             target orientation: {w: #, xyz: [#, #, #]}
//! [4] Three-point arc with constant orientation:
//!     - "three_point_arc" position mode with "constant" orientation mode.
//!     - Requires "target position" and "intermediate position".
//!     - See `TaskSpacePathSpec::addThreePointArc()` in `task_space_path_spec.h` for more
//!       documentation.
//!     - Format:
//!           - position mode: three_point_arc
//!             orientation mode: constant
//!             target position: [#, #, #]
//!             intermediate position: [#, #, #]
//! [5] Three-point arc with tangent orientation:
//!     - "three_point_arc" position mode with "tangent" orientation mode.
//!     - Requires "target position" and "intermediate position".
//!     - See `TaskSpacePathSpec::addThreePointArc()` in `task_space_path_spec.h` for more
//!       documentation.
//!     - Format:
//!           - position mode: three_point_arc
//!             orientation mode: tangent
//!             target position: [#, #, #]
//!             intermediate position: [#, #, #]
//! [6] Three-point arc with orientation target:
//!     - "three_point_arc" position mode with "slerp" orientation mode.
//!     - Requires "target pose" and "intermediate position".
//!     - See `TaskSpacePathSpec::addThreePointArcWithOrientationTarget()` in
//!       `task_space_path_spec.h` for more documentation.
//!     - Format:
//!           - position mode: three_point_arc
//!             orientation mode: slerp
//!             target pose:
//!               position: [#, #, #]
//!               orientation: {w: #, xyz: [#, #, #]}
//!             intermediate position: [#, #, #]
//! [7] Tangent arc with constant orientation:
//!     - "tangent_arc" position mode with "constant" orientation mode.
//!     - Requires "target position".
//!     - See `TaskSpacePathSpec::addTangentArc()` in `task_space_path_spec.h` for more
//!       documentation.
//!     - Format:
//!           - position mode: tangent_arc
//!             orientation mode: constant
//!             target position: [#, #, #]
//! [8] Tangent arc with tangent orientation:
//!     - "tangent_arc" position mode with "tangent" orientation mode.
//!     - Requires "target position".
//!     - See `TaskSpacePathSpec::addTangentArc()` in `task_space_path_spec.h` for more
//!       documentation.
//!     - Format:
//!           - position mode: tangent_arc
//!             orientation mode: tangent
//!             target position: [#, #, #]
//! [9] Tangent arc with orientation target:
//!     - "tangent_arc" position mode with "slerp" orientation mode.
//!     - Requires "target pose".
//!     - See `TaskSpacePathSpec::addTangentArcWithOrientationTarget()` in
//!       `task_space_path_spec.h` for more documentation.
//!     - Format:
//!           - position mode: tangent_arc
//!             orientation mode: slerp
//!             target pose:
//!               position: [#, #, #]
//!               orientation: {w: #, xyz: [#, #, #]}
//!
//! If "initial pose" or "path specs" are unable to be parsed, `nullptr` will be returned. If any
//! "path specs" fail to be parsed, they will be discarded and a warning will be logged, but any
//! other valid "path specs" will continue to be added to the returned `TaskSpacePathSpec`.
CUMO_EXPORT std::unique_ptr<TaskSpacePathSpec> LoadTaskSpacePathSpecFromFile(
    const std::filesystem::path &task_space_path_spec_file);

//! Load a `TaskSpacePathSpec` from the contents of a YAML file (`task_space_path_spec_yaml`).
//!
//! See `LoadTaskSpacePathSpecFromFile()` documentation for detailed description of required YAML
//! format.
CUMO_EXPORT std::unique_ptr<TaskSpacePathSpec> LoadTaskSpacePathSpecFromMemory(
    const std::string &task_space_path_spec_yaml);

//! Export `task_space_path_spec` as a string.
//!
//! The returned string will be in YAML format as documented in `LoadTaskSpacePathSpecFromFile()`.
CUMO_EXPORT std::string ExportTaskSpacePathSpecToMemory(
    const TaskSpacePathSpec &task_space_path_spec);

//! Load a `CSpacePathSpec` from file with absolute path `cspace_path_spec_file`.
//!
//! The `cspace_spec_file` is expected to correspond to a YAML file specifying a c-space path.
//! This path specification must include:
//!   [1] an initial c-space position, and
//!   [2] a series of c-space waypoints following this initial c-space position.
//!
//! The initial c-space position uses the key "initial c-space position" and is expected to have the
//! following format:
//!
//!   initial c-space position: [#, #, ... , #]
//!
//! where each "#" represents a floating point value. The number of elements in this vector sets the
//! expected number of c-space coordinates for the `CSpacePathSpec`.
//!
//! The c-space waypoints are specified as a sequence under the key "waypoints", with the following
//! format:
//!
//!   waypoints:
//!     - [#, #, ... , #]
//!     - [#, #, ... , #]
//!     - [#, #, ... , #]
//!
//! where the number of waypoints is variable. Each waypoint must have the same number of
//! c-space coordinates as the "initial c-space position", or it will be discarded and a warning
//! will be logged.
//!
//! If "initial c-space position" or "waypoints" are unable to be parsed, `nullptr` will be
//! returned. If any "waypoints" fail to be parsed, they will be discarded and a warning will be
//! logged, but any other valid "waypoints" will continue to be added to the returned
//! `CSpacePathSpec`.
CUMO_EXPORT std::unique_ptr<CSpacePathSpec> LoadCSpacePathSpecFromFile(
    const std::filesystem::path &cspace_path_spec_file);

//! Load a `CSpacePathSpec` from the contents of a YAML file (`cspace_path_spec_yaml`).
//!
//! See `LoadCSpacePathSpecFromFile()` documentation for detailed description of required YAML
//! format.
CUMO_EXPORT std::unique_ptr<CSpacePathSpec> LoadCSpacePathSpecFromMemory(
    const std::string &cspace_path_spec_yaml);

//! Export `cspace_path_spec` as a string.
//!
//! The returned string will be in YAML format as documented in `LoadCSpacePathSpecFromFile()`.
CUMO_EXPORT std::string ExportCSpacePathSpecToMemory(const CSpacePathSpec &cspace_path_spec);

//! Load a `CompositePathSpec` from file with absolute path `composite_path_spec_file`.
//!
//! The `composite_path_spec_file` is expected to correspond to a YAML file specifying a path
//! composed of task space and/or c-space segments. This path specification must include:
//!   [1] an initial c-space position, and
//!   [2] a series of task-space and/or c-space path specifications following this initial
//!       c-space position.
//!
//! The initial c-space position uses the key "initial c-space position" and is expected to have the
//! following format:
//!
//!   initial c-space position: [#, #, ... , #]
//!
//! where each "#" represents a floating point value. The number of elements in this vector sets the
//! expected number of c-space coordinates for the `CSpacePathSpec`.
//!
//! The series of path specifications uses the key "path specs" and each spec is required to be
//! labelled as either "c-space" or "task-space". In addition to including the full
//! specification for the task space or c-space path (see `LoadTaskSpacePathSpecFromFile()` and
//! `LoadCSpacePathSpecFromFile()` for details), each path specification must specify a
//! "transition mode" which must be "skip", "free", or "linear_task_space". These modes correspond
//! to the `CompositePathSpec::TransitionMode` defined in `composite_path_spec.h`.
//!
//! An example specification may look like:
//!
//! initial c-space position: [#, #, ... , #]
//! path specs:
//!   - c-space:
//!       transition mode: skip
//!       initial c-space position: [#, #, ... , #]
//!       waypoints:
//!         - [#, #, ... , #]
//!         - [#, #, ... , #]
//!         - ...
//!         - [#, #, ... , #]
//!   - task-space:
//!       transition mode: linear_task_space
//!       initial pose:
//!         position: [#, #, #]
//!         orientation: { w: #, xyz: [#, #, #] }
//!       path specs:
//!         - position mode: linear
//!           orientation mode: constant
//!           target position: [#, #, #]
//!           blend radius: 0.0
//!         - ...
//!   - ...
//!
//! where any combination of task space and c-space path specifications may be included. The order
//! of the path specifications in the YAML file dictates the order in which they are added to the
//! `CompositePathSpec`.
//!
//! If "initial c-space position" or "path specs" are unable to be parsed, `nullptr` will be
//! returned. If any "path specs" fail to be parsed, they will be discarded and a warning will be
//! logged, but any other valid "path specs" will continue to be added to the returned
//! `CompositePathSpec`.
CUMO_EXPORT std::unique_ptr<CompositePathSpec> LoadCompositePathSpecFromFile(
    const std::filesystem::path &composite_path_spec_file);

//! Load a `CompositePathSpec` from the contents of a YAML file (`composite_path_spec_yaml`).
//!
//! See `LoadCompositePathSpecFromFile()` documentation for detailed description of required YAML
//! format.
CUMO_EXPORT std::unique_ptr<CompositePathSpec> LoadCompositePathSpecFromMemory(
    const std::string &composite_path_spec_yaml);

//! Export `composite_path_spec` as a string.
//!
//! The returned string will be in YAML format as documented in `LoadCompositePathSpecFromFile()`.
CUMO_EXPORT std::string ExportCompositePathSpecToMemory(
    const CompositePathSpec &composite_path_spec);

}  // namespace cumotion
