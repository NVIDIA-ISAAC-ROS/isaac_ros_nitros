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

#include "Eigen/Core"

#include "cumotion/cspace_path_spec.h"
#include "cumotion/cumotion_export.h"
#include "cumotion/task_space_path_spec.h"

namespace cumotion {

//! The `CompositePathSpec` is used to procedurally composite `CSpacePathSpec` and
//! `TaskSpacePathSpec` segments into a single path specification.
class CUMO_EXPORT CompositePathSpec {
 public:
  virtual ~CompositePathSpec() = default;

  //! Return the number of configuration space coordinates for the path specification.
  [[nodiscard]] virtual int numCSpaceCoords() const = 0;

  //! Return the number of path specifications contained in the `CompositePathSpec`.
  [[nodiscard]] virtual int numPathSpecs() const = 0;

  //! Indicate whether a path specification is a `TaskSpacePathSpec` (i.e., `TASK_SPACE`) or a
  //! `CSpacePathSpec` (i.e., `CSPACE`).
  enum class PathSpecType {
    TASK_SPACE,
    CSPACE
  };

  //! Given a `path_spec_index` in range [0, `numPathSpecs()`), return the type of the corresponding
  //! path specification.
  //!
  //! A fatal error will be logged if `path_spec_index` is not in range [0, `numPathSpecs()`).
  //!
  //! The `path_spec_index` corresponds to the order in which path specifications are added to the
  //! `CompositePathSpec`.
  [[nodiscard]] virtual PathSpecType pathSpecType(int path_spec_index) const = 0;

  //! Return a `TaskSpacePathSpec` at the given `path_spec_index`.
  //!
  //! If the `path_spec_index` is invalid (i.e., not in range `[0, `numPathSpecs()`)) *OR* the
  //! `path_spec_index` does not correspond to a `TaskSpacePathSpec`, then `nullptr` will be
  //! returned and an error will be logged.
  [[nodiscard]]
  virtual std::unique_ptr<TaskSpacePathSpec> taskSpacePathSpec(int path_spec_index) const = 0;

  //! Return a `CSpacePathSpec` at the given `path_spec_index`.
  //!
  //! If the `path_spec_index` is invalid (i.e., not in range `[0, `numPathSpecs()`)) *OR* the
  //! `path_spec_index` does not correspond to a `CSpacePathSpec`, then `nullptr` will be
  //! returned and an error will be logged.
  [[nodiscard]]
  virtual std::unique_ptr<CSpacePathSpec> cSpacePathSpec(int path_spec_index) const = 0;

  //! Specify the transition preceding a `TaskSpacePathSpec` or `CSpacePathSpec`.
  enum class TransitionMode {
    //! Skip either the initial task space pose of the `TaskSpacePathSpec` or the initial c-space
    //! configuration of the `CSpacePathSpec`.
    //!
    //! For a `TaskSpacePathSpec`, the first task space path segment in the appended
    //! `TaskSpacePathSpec` will instead originate from the current task space pose of the
    //! `CompositePathSpec`.
    //!
    //! For a `CSpacePathSpec`, The first c-space waypoint of the added `CSpacePathSpec` will be
    //! added directly after the current c-space configuration of the `CompositePathSpec`.
    SKIP,

    //! Add a path from the current pose of the `CompositePathSpec` to the initial task space pose
    //! of the `TaskSpacePathSpec` or the initial c-space configuration of the `CSpacePathSpec`,
    //! with no restrictions on the form of the path.
    FREE,

    //! Add a path that is linear in task space from the current task space pose of the
    //! `CompositePathSpec` to the initial task space pose of the `TaskSpacePathSpec`.
    //!
    //! *WARNING* This mode is *ONLY* available for adding a `TaskSpacePathSpec`
    //! (via `addTaskSpacePathSpec()`). Usage with `addCSpacePathSpec()` will result in an error
    //! and the `CSpacePathSpec` will *NOT* be added.
    LINEAR_TASK_SPACE
  };

  //! Given a `path_spec_index` in range [0, `numPathSpecs()`), return the corresponding transition
  //! mode.
  //!
  //! A fatal error will be logged if `path_spec_index` is not in range [0, `numPathSpecs()`).
  //!
  //! The `path_spec_index` corresponds to the order in which path specifications are added to the
  //! `CompositePathSpec`.
  [[nodiscard]] virtual TransitionMode transitionMode(int path_spec_index) const = 0;

  //! Add a task space `path_spec` to the `CompositePathSpec` with the specified `transition_mode`.
  //!
  //! Returns `true` if path specification is successfully added. Else, returns `false`.
  virtual bool addTaskSpacePathSpec(const TaskSpacePathSpec &path_spec,
                                    TransitionMode transition_mode) = 0;

  //! Add a c-space `path_spec` to the `CompositePathSpec` with the specified `transition_mode`.
  //!
  //! `path_spec` will be discarded (with logged error) if it does not have the same number of
  //! c-space coordinates as the `CompositePathSpec` (i.e., `numCSpaceCoords()`).
  //!
  //! `path_spec` will be discarded (with logged error) if the `transition_mode` is invalid
  //! (i.e., `LINEAR_TASK_SPACE`).
  //!
  //! Returns `true` if path specification is successfully added. Else, returns `false`.
  virtual bool addCSpacePathSpec(const CSpacePathSpec &path_spec,
                                 TransitionMode transition_mode) = 0;
};

//! Create a `CompositePathSpec` with the specified `initial_cspace_position`.
CUMO_EXPORT std::unique_ptr<CompositePathSpec> CreateCompositePathSpec(
    const Eigen::VectorXd &initial_cspace_position);

}  // namespace cumotion
