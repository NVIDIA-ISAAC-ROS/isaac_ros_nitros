// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_COMMON_ASSERT_HPP_
#define NVIDIA_COMMON_ASSERT_HPP_

#include <cstdlib>
#include <cstring>
#include <string>

#include "common/backtrace.hpp"
#include "common/logger.hpp"

// Prints a panic message and aborts the program
#define GXF_PANIC(...)                                                                             \
  {                                                                                                \
    GXF_LOG_PANIC(__VA_ARGS__);                                                                    \
    PrettyPrintBacktrace();                                                                        \
    std::exit(1);                                                                                  \
  }

// Checks if an expression evaluates to true. If not prints a panic message and aborts the program.
#define GXF_ASSERT(expr, ...)                                                                      \
  if (!(expr)) {                                                                                   \
    GXF_LOG_PANIC(__VA_ARGS__);                                                                    \
    PrettyPrintBacktrace();                                                                        \
    std::exit(1);                                                                                  \
  }

#define GXF_ASSERT_SUCCESS(expr)                                                                   \
  {                                                                                                \
    const auto _result = (expr);                                                                   \
    if (_result != 0) {                                                                            \
      GXF_LOG_PANIC("GXF operation failed: %s", GxfResultStr(_result));                            \
      PrettyPrintBacktrace();                                                                      \
      std::exit(1);                                                                                \
    }                                                                                              \
  }                                                                                                \

// Asserts that A == true. If not prints a panic message and aborts the program.
#define GXF_ASSERT_TRUE(exp_a)                                                                     \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    GXF_ASSERT(_va == true, "Assert failed: %s == true.", std::to_string(_va).c_str());            \
  }

// Asserts that A == false. If not prints a panic message and aborts the program.
#define GXF_ASSERT_FALSE(exp_a)                                                                    \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    GXF_ASSERT(_va == false, "Assert failed: %s == false.", std::to_string(_va).c_str());          \
  }

// Asserts that A == B. If not prints a panic message and aborts the program.
#define GXF_ASSERT_EQ(exp_a, exp_b)                                                                \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    GXF_ASSERT(_va == _vb, "Assert failed: %s == %s.", std::to_string(_va).c_str(),                \
               std::to_string(_vb).c_str());                                                       \
  }

// Asserts that A == B for two strings. If not prints a panic message and aborts the program.
#define GXF_ASSERT_STREQ(exp_a, exp_b)                                                             \
  {                                                                                                \
    const char* _va = (exp_a);                                                                     \
    const char* _vb = (exp_b);                                                                     \
    GXF_ASSERT(std::strcmp(_va, _vb) == 0, "Assert failed: %s == %s.", _va, _vb);                  \
  }

// Asserts that A != B. If not prints a panic message and aborts the program.
#define GXF_ASSERT_NE(exp_a, exp_b)                                                                \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    GXF_ASSERT(_va != _vb, "Assert failed: %s != %s.", std::to_string(_va).c_str(),                \
               std::to_string(_vb).c_str());                                                       \
  }

// Asserts that A > B. If not prints a panic message and aborts the program.
#define GXF_ASSERT_GT(exp_a, exp_b)                                                                \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    GXF_ASSERT(_va > _vb, "Assert failed: %s > %s.", std::to_string(_va).c_str(),                  \
               std::to_string(_vb).c_str());                                                       \
  }

// Asserts that A >= B. If not prints a panic message and aborts the program.
#define GXF_ASSERT_GE(exp_a, exp_b)                                                                \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    GXF_ASSERT(_va >= _vb, "Assert failed: %s >= %s.", std::to_string(_va).c_str(),                \
               std::to_string(_vb).c_str());                                                       \
  }

// Asserts that A > B. If not prints a panic message and aborts the program.
#define GXF_ASSERT_LT(exp_a, exp_b)                                                                \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    GXF_ASSERT(_va < _vb, "Assert failed: %s > %s.", std::to_string(_va).c_str(),                  \
               std::to_string(_vb).c_str());                                                       \
  }

// Asserts that A <= B. If not prints a panic message and aborts the program.
#define GXF_ASSERT_LE(exp_a, exp_b)                                                                \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    GXF_ASSERT(_va <= _vb, "Assert failed: %s <= %s.", std::to_string(_va).c_str(),                \
               std::to_string(_vb).c_str());                                                       \
  }

// Asserts that abs(A - B) <= abs_error. If not prints a panic message and aborts the program.
#define GXF_ASSERT_NEAR(exp_a, exp_b, exp_abs_error)                                               \
  {                                                                                                \
    const auto _va = exp_a;                                                                        \
    const auto _vb = exp_b;                                                                        \
    const auto _verror = exp_abs_error;                                                            \
    GXF_ASSERT(std::abs(_va - _vb) <= _verror, "Assert failed: abs(%s - %s) <= %s.",               \
               std::to_string(_va).c_str(), std::to_string(_vb).c_str(),                           \
               std::to_string(_verror).c_str());                                                   \
  }

#endif  // NVIDIA_COMMON_ASSERT_HPP_
