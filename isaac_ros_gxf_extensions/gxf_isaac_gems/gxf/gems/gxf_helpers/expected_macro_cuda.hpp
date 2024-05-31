// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "gxf/core/expected_macro.hpp"

namespace nvidia::expected_macro {

template <>
struct IsStatus<cudaError_t> : std::true_type {};

template <>
constexpr bool IsValid<cudaError_t>(const cudaError_t& status) {
  return status == cudaSuccess;
}

template <>
constexpr cudaError_t DefaultError<cudaError_t>() {
  return cudaErrorUnknown;
}

template <>
struct StatusName<cudaError_t> {
  static std::string Get(cudaError_t status) { return std::string(cudaGetErrorString(status)); }
};

}  // namespace nvidia::expected_macro
