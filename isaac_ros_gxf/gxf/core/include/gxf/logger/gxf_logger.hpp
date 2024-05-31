// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef COMMON_LOGGER_GXF_LOGGER_HPP
#define COMMON_LOGGER_GXF_LOGGER_HPP

#include <cstdio>
#include <memory>

#include "gxf/logger/logger.hpp"

namespace nvidia {

// Environment variable for log level override
constexpr const char* kGxfLogEnvName = "GXF_LOG_LEVEL";

// Indicates the level of severity for a log message
enum class Severity {
  // A utility case which can be used for 'SetSeverity' to disable all severity levels.
  NONE = -2,
  // A utility case which can be used for 'Redirect' and 'SetSeverity'
  ALL = -1,
  // The five different log severities in use from most severe to least severe.
  PANIC = 0,  // Need to start at 0
  ERROR,
  WARNING,
  INFO,
  DEBUG,
  VERBOSE,
  // A utility case representing the number of log levels used internally.
  COUNT
};

// The output severity which might limit certain levels of severity
struct SeverityContainer {
  Severity r = Severity::INFO;

  // Sets the default severity from the environment variable if it is set.
  SeverityContainer();
};

/// Namespace for the NVIDIA logger functionality.
namespace logger {

/// A logger that utilizes the GXF logging system.
///
/// The GXF logger supports six log levels:
///   PANIC(0), ERROR(1), WARNING(2), INFO(3), DEBUG(4), and VERBOSE(5).
///
/// If no logger or log function is provided, a default logger implementation (DefaultGxfLogger) is
/// used. The default logger leverages the existing functionality of the GXF logging system, which
/// employs a global/singleton SeverityContainer to control the logging level, and a global
/// `nvidia::LoggingFunction` to log messages.
/// The initial logging level is determined by the environment variable `GXF_LOG_LEVEL` if it is
/// set. Otherwise, the default logging level is INFO(3).
class GxfLogger : public Logger {
 public:
  /// Constructs a logger with the provided logger or log function.
  ///
  /// If neither a logger nor a log function is provided, a default GXF logger will be instantiated.
  ///
  /// @param logger The logger to use (default: nullptr).
  /// @param func The log function to use (default: nullptr).
  explicit GxfLogger(const std::shared_ptr<ILogger>& logger = nullptr,
                     const LogFunction& func = nullptr);
};

/// A logger that facilitates access to the global GXF logger.
///
/// The singleton GXF logger can be accessed via the instance() method.
/// This class also provides a method to set the log severity from an environment variable.
/// The log severity can be retrieved from the environment variable using GetSeverityFromEnv().
class GlobalGxfLogger {
 public:
  /// Retrieve the global/singleton GXF logger instance.
  static GxfLogger& instance();

  /// Set the log severity based on the value of an environment variable.
  ///
  /// If the environment variable is not set or invalid, the severity remains unchanged.
  /// @param env_name The name of the environment variable to use.
  /// @return true if the environment variable is accessible and the severity was updated from it.
  static bool SetSeverityFromEnv(const char* env_name);

  /// Return the log severity based on the value of an environment variable.
  ///
  /// If the environment variable is not set or invalid, returns Severity::COUNT.
  /// If 'error_code' is not null, it will be set to 1 if the environment variable is set but
  /// invalid, or to 0 otherwise.
  ///
  /// @param env_name The name of the environment variable to use.
  /// @param error_code The error code to set if the environment variable is invalid.
  /// @return The log severity from the environment variable.
  static Severity GetSeverityFromEnv(const char* env_name, int* error_code);
};

}  // namespace logger

}  // namespace nvidia

#endif /* COMMON_LOGGER_GXF_LOGGER_HPP */
