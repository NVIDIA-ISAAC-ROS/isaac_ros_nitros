// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "gems/gxf_helpers/common_expected_macro.hpp"
#include "gxf/core/expected.hpp"

// This customizes the expected macro, s.t. it can be used with gxf_result_t.
namespace nvidia {
template <>
struct ExpectedMacroConfig<gxf_result_t> {
  constexpr static gxf_result_t DefaultSuccess() { return GXF_SUCCESS; }
  constexpr static gxf_result_t DefaultError() { return GXF_FAILURE; }
  static std::string Name(gxf_result_t result) { return GxfResultStr(result); }
};

// For back-compatibility we define an alias to the original name of the macro when it was gxf
// specific.
#define GXF_RETURN_IF_ERROR RETURN_IF_ERROR
#define GXF_UNWRAP_OR_RETURN UNWRAP_OR_RETURN

}  // namespace nvidia
