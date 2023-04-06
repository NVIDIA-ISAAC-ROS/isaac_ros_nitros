/*
Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
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
