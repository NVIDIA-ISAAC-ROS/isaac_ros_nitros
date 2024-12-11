// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <memory>
#include <vector>
#include "gxf/core/gxf.h"
#include "gxf/std/resources.hpp"
#include "gxf/std/system.hpp"

namespace nvidia {
namespace gxf {

class EntityExecutor;  // forward declaration

// An interface which extends the nvidia::gxf::System interface to create schedulers
// which can execute codelets.

class Scheduler : public System {
 public:
  virtual gxf_result_t prepare_abi(EntityExecutor* executor) = 0;
};

}  // namespace gxf
}  // namespace nvidia
