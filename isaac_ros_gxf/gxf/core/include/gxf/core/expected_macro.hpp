// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "gxf/core/common_expected_macro.hpp"
#include "gxf/core/expected.hpp"
#include "magic_enum.hpp"  // NOLINT(build/include)

//////////////////////////////////////////
// Configuration for using bool as status:
//////////////////////////////////////////
namespace nvidia::expected_macro {

// This section contains the configuration to use the expected macro with
// different return types like bool, int, std::optional (and in the future std::expected).
template <>
struct IsStatus<bool> : std::true_type {};

template <>
constexpr bool DefaultError<bool>() {
  return false;
}

template <>
struct StatusName<bool> {
  static std::string Get(bool status) { return status ? "Success" : "Failure"; }
};

/////////////////////////////////////////
// Configuration for using int as status:
/////////////////////////////////////////
template <>
struct IsStatus<int> : std::true_type {};

template <>
constexpr bool IsValid<int>(const int& status) {
  return status == 0;
}

template <>
constexpr int DefaultError<int>() {
  return 1;
}

template <>
struct StatusName<int> {
  static std::string Get(int status) { return std::to_string(status); }
};

//////////////////////////////////////////
// Configuration for using enum as status:
//////////////////////////////////////////
template <typename EnumStatus>
struct StatusName<EnumStatus, typename std::enable_if_t<std::is_enum_v<EnumStatus>>> {
  static std::string Get(EnumStatus status) { return std::string(magic_enum::enum_name(status)); }
};

////////////////////////////////////////////////////////
// Configuration for using std::optional as unwrappable:
////////////////////////////////////////////////////////
template <typename Value>
struct IsUnwrappable<std::optional<Value>> : std::true_type {};

template <typename Value>
struct StatusType<std::optional<Value>> {
  using Type = bool;
};

template <typename Value>
struct StatusValue<std::optional<Value>> {
  static constexpr bool Get(const std::optional<Value>& optional) { return optional.has_value(); }
};

template <typename Value>
struct InvalidUnwrappable<std::optional<Value>, bool> {
  static std::optional<Value> Get(bool status) { return std::nullopt; }
};

///////////////////////////////////////////////////////////
// Configuration for using nvidia::Expected as unwrappable:
///////////////////////////////////////////////////////////
template <typename Value, typename Status>
struct IsUnwrappable<::nvidia::Expected<Value, Status>> : std::true_type {};

template <typename Value, typename Status>
struct StatusType<::nvidia::Expected<Value, Status>> {
  using Type = Status;
};

template <typename Value, typename Status>
struct StatusValue<::nvidia::Expected<Value, Status>> {
  static constexpr Status Get(const ::nvidia::Expected<Value, Status>& expected) {
    return expected.error();
  }
};

template <typename Value, typename Status>
struct InvalidUnwrappable<::nvidia::Expected<Value, Status>, Status> {
  static ::nvidia::Expected<Value, Status> Get(Status status) {
    return ::nvidia::Unexpected<Status>(status);
  }
};

// This customizes the expected macro, s.t. it can be used with gxf_result_t.
template <>
struct IsStatus<gxf_result_t> : std::true_type {};

template <>
constexpr bool IsValid<gxf_result_t>(const gxf_result_t& status) {
  return status == GXF_SUCCESS;
}

template <>
constexpr gxf_result_t DefaultError<gxf_result_t>() {
  return GXF_FAILURE;
}

template <>
struct StatusName<gxf_result_t> {
  static std::string Get(gxf_result_t status) { return GxfResultStr(status); }
};

// For back-compatibility we define an alias to the original name of the macro when it was gxf
// specific.
#define GXF_RETURN_IF_ERROR RETURN_IF_ERROR
#define GXF_UNWRAP_OR_RETURN UNWRAP_OR_RETURN
}  // namespace nvidia::expected_macro
