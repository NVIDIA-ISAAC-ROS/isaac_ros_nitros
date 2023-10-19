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

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "gxf/core/expected.hpp"

namespace nvidia {
namespace gxf {
namespace optimizer {

template<typename T>
using UniquePointerMap = std::map<gxf_uid_t, std::unique_ptr<T>>;

using UIDList = std::vector<gxf_uid_t>;
using UIDSet = std::set<gxf_uid_t>;
using UIDStringMap = std::map<gxf_uid_t, std::string>;
using StringMap = std::map<std::string, std::string>;
using StringList = std::vector<std::string>;
using StringSet = std::set<std::string>;

constexpr char kAnyDataType[] = "any";
constexpr char kToBeDeterminedDataType[] = "tbd";
constexpr char kUndefinedDataType[] = "undefined";

// Factors for the cost of graph (COG)
enum class gxf_cog_factor_t : std::int32_t {
  THROUGHPUT = 0,
  LATENCY,
  POWER,
  ACCURACY,
};

// Get the gxf_cog_factor_t enum value from a COG factor's string.
Expected<gxf_cog_factor_t> ParseCOGFactorString(std::string factor_str);

// Get the string of a gxf_cog_factor_t factor.
Expected<const char*> GetCOGFactorString(const gxf_cog_factor_t factor);

}  // namespace optimizer
}  // namespace gxf
}  // namespace nvidia
