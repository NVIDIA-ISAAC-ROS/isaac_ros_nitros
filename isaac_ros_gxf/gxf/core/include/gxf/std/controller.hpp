// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVIDIA_GXF_STD_CONTROLLER_HPP_
#define NVIDIA_GXF_STD_CONTROLLER_HPP_

#include "gxf/core/component.hpp"
#include "gxf/std/scheduling_condition.hpp"

namespace nvidia {
namespace gxf {

/// @brief The status returned by controller to executor deciding termination
/// policy
/// - GXF_EXECUTE_SUCCESS: executor resumes execution
/// - GXF_EXECUTE_FAILURE_REPEAT: codelet fails and executor repeatedly execute
/// this entity
/// - GXF_EXECUTE_FAILURE_DEACTIVATE: codelet fails and executor deactivate this
/// entity
/// - GXF_EXECUTE_FAILURE: codelet fails and executor deactivates all the
/// entities and stops the entire graph
typedef enum {
  GXF_EXECUTE_SUCCESS = 0,
  GXF_EXECUTE_FAILURE_REPEAT = 1,
  GXF_EXECUTE_FAILURE_DEACTIVATE = 2,
  GXF_EXECUTE_FAILURE = 3,
} gxf_execution_status_t;

// Type to represent codelet::tick() result for behavior tree parent and
// execution status for executor to use termination policy
struct gxf_controller_status_t {
  entity_state_t behavior_status;
  gxf_execution_status_t exec_status;
  gxf_controller_status_t(entity_state_t b_status,
                          gxf_execution_status_t e_status)
      : behavior_status(b_status), exec_status(e_status) {}
  gxf_controller_status_t() {
    behavior_status = GXF_BEHAVIOR_SUCCESS;
    exec_status = GXF_EXECUTE_SUCCESS;
  }
};

// Interface for controlling entity's termination policy and entity's execution
// status
class Controller : public Component {
 public:
  virtual ~Controller() = default;

  // Return a struct encapsulating the determined behavior status and execution
  // status by the controller given the result of codelet's tick()
  virtual gxf_controller_status_t control(gxf_uid_t eid,
                                          Expected<void> code) = 0;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_CONTROLLER_HPP_
