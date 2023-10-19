// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <string>

#include "engine/gems/serialization/json.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

// Stores the Sop message and provides convenience views to it.
struct JsonMessageParts {
  // The message entity
  gxf::Entity entity;
  // Handle to the Sop instance for sight visualization
  gxf::Handle<::nvidia::isaac::Json> json;
  // Timestamp
  gxf::Handle<gxf::Timestamp> timestamp;
};

// Creates message entity and attaches a Sop instance to it to be populated later for sight
// visualization.
gxf::Expected<JsonMessageParts> CreateJsonMessage(const char* tag_name, gxf_context_t context,
                                                  bool activate = true);

}  // namespace isaac
}  // namespace nvidia
