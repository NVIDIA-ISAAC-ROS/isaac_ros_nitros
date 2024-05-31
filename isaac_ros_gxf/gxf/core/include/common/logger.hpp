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
#ifndef NVIDIA_COMMON_LOGGER_HPP_
#define NVIDIA_COMMON_LOGGER_HPP_

#include <cstdarg>
#include <cstdio>
#include <vector>

#include "gxf/logger/gxf_logger.hpp"

#define GXF_LOG_LEVEL_PANIC 0
#define GXF_LOG_LEVEL_ERROR 1
#define GXF_LOG_LEVEL_WARNING 2
#define GXF_LOG_LEVEL_INFO 3
#define GXF_LOG_LEVEL_DEBUG 4
#define GXF_LOG_LEVEL_VERBOSE 5

// clang-format off

// Set the default active logging level to VERBOSE(5) if not defined
#if !defined(GXF_LOG_ACTIVE_LEVEL)
#  define GXF_LOG_ACTIVE_LEVEL 5  // GXF_LOG_LEVEL_VERBOSE
#endif

// Define GXF_LOG_ACTIVE_LEVEL before including `common/logger.hpp` to control the logging level
// at compile time. This allows you to skip logging at certain levels.
//
// Example:
//     #define GXF_LOG_ACTIVE_LEVEL 2
//     #include "common/logger.hpp"
//     ...
//
// With this setting, logging will occur only at the WARNING(2), ERROR(1), and PANIC(0) levels.
//
// You can define GXF_LOG_ACTIVE_LEVEL in your build system. For instance, in CMake, use:
//
//     target_compile_definitions(my_target PRIVATE GXF_LOG_ACTIVE_LEVEL=2)
//
// This sets the active logging level to WARNING(2) for the target `my_target`.
//
// Alternatively, define GXF_LOG_ACTIVE_LEVEL at compile time by passing `-DGXF_LOG_ACTIVE_LEVEL=2`
// directly to the compiler.
//
// In the Bazel build system, set this in your build configuration as follows:
//
//     cc_binary(
//         name = "my_binary",
//         srcs = ["my_binary.cc"],
//         copts = ["-DGXF_LOG_ACTIVE_LEVEL=2"],
//     )
//
// This sets the active logging level to WARNING(2) for the target `my_binary`.
//
// Or, when using a Bazel build command:
//     bazel build --copt=-DGXF_LOG_ACTIVE_LEVEL=3 //path:to_your_target
//
// This sets the active logging level to INFO(3) for the target `//path:to_your_target`.

// Logs a verbose message
#if GXF_LOG_ACTIVE_LEVEL >= GXF_LOG_LEVEL_VERBOSE
#  define GXF_LOG_VERBOSE(...) \
      ::nvidia::Log(__FILE__, __LINE__, ::nvidia::Severity::VERBOSE, __VA_ARGS__)
#else
#  define GXF_LOG_VERBOSE(...) (void)0
#endif

// Logs a debug message
#if GXF_LOG_ACTIVE_LEVEL >= GXF_LOG_LEVEL_DEBUG
#  define GXF_LOG_DEBUG(...) \
      ::nvidia::Log(__FILE__, __LINE__, ::nvidia::Severity::DEBUG, __VA_ARGS__)
#else
#  define GXF_LOG_DEBUG(...) (void)0
#endif

// Logs an informational message
#if GXF_LOG_ACTIVE_LEVEL >= GXF_LOG_LEVEL_INFO
#  define GXF_LOG_INFO(...) \
      ::nvidia::Log(__FILE__, __LINE__, ::nvidia::Severity::INFO, __VA_ARGS__)
#else
#  define GXF_LOG_INFO(...) (void)0
#endif

// Logs a warning
#if GXF_LOG_ACTIVE_LEVEL >= GXF_LOG_LEVEL_WARNING
#  define GXF_LOG_WARNING(...) \
      ::nvidia::Log(__FILE__, __LINE__, ::nvidia::Severity::WARNING, __VA_ARGS__)
#else
#  define GXF_LOG_WARNING(...) (void)0
#endif

// Logs an error
#if GXF_LOG_ACTIVE_LEVEL >= GXF_LOG_LEVEL_ERROR
#  define GXF_LOG_ERROR(...) \
      ::nvidia::Log(__FILE__, __LINE__, ::nvidia::Severity::ERROR, __VA_ARGS__)
#else
#  define GXF_LOG_ERROR(...) (void)0
#endif

// Logs a panic
#if GXF_LOG_ACTIVE_LEVEL >= GXF_LOG_LEVEL_PANIC
#  define GXF_LOG_PANIC(...) \
      ::nvidia::Log(__FILE__, __LINE__, ::nvidia::Severity::PANIC, __VA_ARGS__)
#else
#  define GXF_LOG_PANIC(...) (void)0
#endif

// clang-format on

namespace nvidia {

// Function which is used for logging. It can be changed to intercept the logged messages.
// Additional arguments can be supplied via LoggingFunctionArg.
extern void (*LoggingFunction)(const char* file, int line, Severity severity,
                               const char* log, void* arg);
extern void* LoggingFunctionArg;

// Default implementation of the logging function which prints to console
void DefaultConsoleLogging(const char* file, int line, Severity severity,
                           const char* log, void* arg);

// Redirects the output for a given log severity.
void Redirect(std::FILE* file, Severity severity = Severity::ALL);

// Sets the log severity from the environment variable.
// If the environment variable is not set or invalid, the severity is not changed.
// Returns true if the environment variable is accessible and the severity was updated from it.
bool SetSeverityFromEnv(const char* env_name = kGxfLogEnvName);

// Returns the log severity from the environment variable.
// If the environment variable is not set or invalid, returns Severity::COUNT.
// If 'error_code' is not null, it will be set to 1 if the environment variable is set but invalid,
// or to 0 otherwise.
Severity GetSeverityFromEnv(const char* env_name = kGxfLogEnvName, int* error_code = nullptr);

// Sets global log severity thus effectively disabling all logging with lower severity
void SetSeverity(Severity severity);

// Returns global log severity
Severity GetSeverity();

// Converts the message and argument into a string and pass it to LoggingFunction.
template<typename... Args>
void Log(const char* file, int line, Severity severity, const char* txt, ...) __attribute__((format(printf, 4, 5))); // NOLINT

template<typename... Args>
void Log(const char* file, int line, Severity severity, const char* txt, ...) {
  va_list args1;
  va_start(args1, txt);
  va_list args2;
  va_copy(args2, args1);
  std::vector<char> buf(1 + std::vsnprintf(NULL, 0, txt, args1));
  va_end(args1);
  auto buf_data = buf.data();
  auto buf_size = buf.size();
  std::vsnprintf(buf_data, buf_size, txt, args2);
  va_end(args2);

  logger::GxfLogger& logger = logger::GlobalGxfLogger::instance();
  logger.log(file, line, nullptr, static_cast<int>(severity), buf_data);
}

}  // namespace nvidia

#endif  // NVIDIA_COMMON_LOGGER_HPP_
