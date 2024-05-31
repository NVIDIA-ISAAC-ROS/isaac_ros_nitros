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

#include "gems/core/expected.hpp"
#include "gxf/core/expected_macro.hpp"

// This customizes the expected macro, s.t. it can be used with ::nvidia::isaac::Expected.
namespace nvidia::expected_macro {

//////////////////////////////////////////////////////////////////
// Configuration for using nvidia::isaac::Expected as unwrappable:
//////////////////////////////////////////////////////////////////
template <typename Value, typename Status>
struct IsUnwrappable<::nvidia::isaac::Expected<Value, Status>> : std::true_type {};

template <typename Value, typename Status>
struct StatusType<::nvidia::isaac::Expected<Value, Status>> {
  using Type = Status;
};

template <typename Value, typename Status>
struct StatusValue<::nvidia::isaac::Expected<Value, Status>> {
  static constexpr Status Get(const ::nvidia::isaac::Expected<Value, Status>& expected) {
    return expected.error();
  }
};

template <typename Value, typename Status>
struct InvalidUnwrappable<::nvidia::isaac::Expected<Value, Status>, Status> {
  static ::nvidia::isaac::Expected<Value, Status> Get(Status status) {
    return ::nvidia::isaac::Unexpected<Status>(status);
  }
};

}  // namespace nvidia::expected_macro
