// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdlib>
#include <string>

#include "engine/core/logger.hpp"

// Prints a panic message and aborts the program
#define PANIC(...)                                                                                 \
  {                                                                                                \
    ::nvidia::isaac::logger::Log(__FILE__, __LINE__, ::nvidia::isaac::logger::Severity::PANIC, __VA_ARGS__);       \
    std::abort();                                                                                  \
  }

// Checks if an expression evaluates to true. If not prints a panic message and aborts the program.
#define ASSERT(expr, ...)                                                                          \
  if (!(expr)) {                                                                                   \
    ::nvidia::isaac::logger::Log(__FILE__, __LINE__, ::nvidia::isaac::logger::Severity::PANIC, __VA_ARGS__);       \
    std::abort();                                                                                  \
  }

// Asserts that A == B. If not prints a panic message and aborts the program.
#define ISAAC_ASSERT_EQ(exp_a, exp_b)                                                              \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    ASSERT(_va == _vb, "Assert failed: %s == %s.", std::to_string(_va).c_str(),                    \
           std::to_string(_vb).c_str());                                                           \
  }

// Asserts that A != B. If not prints a panic message and aborts the program.
#define ISAAC_ASSERT_NE(exp_a, exp_b)                                                              \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    ASSERT(_va != _vb, "Assert failed: %s != %s.", std::to_string(_va).c_str(),                    \
           std::to_string(_vb).c_str());                                                           \
  }

// Asserts that A > B. If not prints a panic message and aborts the program.
#define ISAAC_ASSERT_GT(exp_a, exp_b)                                                              \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    ASSERT(_va > _vb, "Assert failed: %s > %s.", std::to_string(_va).c_str(),                      \
           std::to_string(_vb).c_str());                                                           \
  }

// Asserts that A >= B. If not prints a panic message and aborts the program.
#define ISAAC_ASSERT_GE(exp_a, exp_b)                                                              \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    ASSERT(_va >= _vb, "Assert failed: %s >= %s.", std::to_string(_va).c_str(),                    \
           std::to_string(_vb).c_str());                                                           \
  }

// Asserts that A > B. If not prints a panic message and aborts the program.
#define ISAAC_ASSERT_LT(exp_a, exp_b)                                                              \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    ASSERT(_va < _vb, "Assert failed: %s > %s.", std::to_string(_va).c_str(),                      \
           std::to_string(_vb).c_str());                                                           \
  }

// Asserts that A <= B. If not prints a panic message and aborts the program.
#define ISAAC_ASSERT_LE(exp_a, exp_b)                                                              \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    ASSERT(_va <= _vb, "Assert failed: %s <= %s.", std::to_string(_va).c_str(),                    \
           std::to_string(_vb).c_str());                                                           \
  }

// Asserts that abs(A - B) < threshold. If not prints a panic message and aborts the program.
#define ISAAC_ASSERT_NEAR(exp_a, exp_b, exp_threshold)                                             \
  {                                                                                                \
    const auto _vdiff = exp_a - exp_b;                                                             \
    const auto _vthreshold = exp_threshold;                                                        \
    ASSERT(std::abs(_vdiff) <= _vthreshold,                                                        \
           "Assert failed: difference between %s and %s as |%s| <= %s.",                           \
           std::to_string(exp_a).c_str(), std::to_string(exp_b).c_str(),                           \
           std::to_string(_vdiff).c_str(), std::to_string(_vthreshold).c_str());                   \
  }
