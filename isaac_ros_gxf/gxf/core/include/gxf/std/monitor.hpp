/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_GXF_STD_MONITOR_HPP_
#define NVIDIA_GXF_STD_MONITOR_HPP_

#include "gxf/core/component.hpp"

namespace nvidia {
namespace gxf {

// Interface for monitoring entities during runtime
class Monitor : public Component {
 public:
  virtual ~Monitor() = default;

  // Callback for after an entity executes
  //         eid - ID of entity that finished execution
  //   timestamp - execution timestamp
  //        code - execution result
  virtual gxf_result_t on_execute_abi(gxf_uid_t eid, uint64_t timestamp, gxf_result_t code) = 0;

  // C++ API wrapper
  Expected<void> onExecute(gxf_uid_t eid, uint64_t timestamp, gxf_result_t code);
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_MONITOR_HPP_
