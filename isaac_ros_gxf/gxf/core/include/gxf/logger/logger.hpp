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

#ifndef COMMON_LOGGER_LOGGER_HPP
#define COMMON_LOGGER_LOGGER_HPP

#include <cstdarg>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace nvidia {

/// Namespace for the NVIDIA logger functionality.
namespace logger {

using LogFunction = std::function<void(const char* file, int line, const char* function_name,
                                       int level, const char* message, void* arg)>;

/// Interface for custom logger implementations.
///
/// ILogger provides an interface for implementing custom logging mechanisms.
/// This interface allows the integration of various logging systems with the Logger class.
/// Users can define their own logging behavior by implementing these methods.
class ILogger {
 public:
  /// Destructor for ILogger.
  virtual ~ILogger() = default;

  /// Log a message.
  ///
  /// This method logs a message based on the provided details such as file name, line number,
  /// function/module name, log level, message, and an optional argument.
  ///
  /// @param file The name of the source file where the log request originated.
  /// @param line The line number in the source file.
  /// @param name The name of the function/module making the log request.
  /// @param level The log level.
  /// @param message The log message.
  /// @param arg An optional argument for additional log information (default: nullptr).
  virtual void log(const char* file, int line, const char* name, int level, const char* message,
                   void* arg = nullptr) = 0;

  /// Set the log message pattern.
  ///
  /// Set the pattern used for formatting log messages.
  ///
  /// @param pattern The pattern string for formatting log messages.
  virtual void pattern(const char* pattern) = 0;

  /// Get the current log message pattern.
  ///
  /// Retrieve the currently set pattern used for formatting log messages.
  ///
  /// @return The currently set pattern string.
  virtual const char* pattern() const = 0;

  /// Set the log level.
  ///
  /// Set the log level, determining the priority of messages that should be logged.
  ///
  /// @param level The log level.
  virtual void level(int level) = 0;

  /// Get the current log level.
  ///
  /// Retrieve the currently set log level.
  ///
  /// @return The currently set log level.
  virtual int level() const = 0;

  /// Redirect log output.
  ///
  /// Redirect log output to the specified file or other output for the given log level.
  ///
  /// @param level The log level for which output redirection is applied.
  /// @param output A pointer to the file or other output to redirect log output.
  virtual void redirect(int level, void* output) = 0;

  /// Get the current redirection for a specified log level.
  ///
  /// Retrieve the file/output pointer to which log output is currently being redirected for the
  /// specified log level. If no redirection is set for the given level, returns nullptr.
  ///
  /// @param level The log level for which the redirection is queried.
  /// @return A pointer to the file/output where log output is redirected, or nullptr if no
  /// redirection is set.
  virtual void* redirect(int level) const = 0;
};

/// Logger class for logging messages.
///
/// The Logger class provides a flexible and customizable logging system. It supports a variety of
/// output targets, such as strings, standard error streams, files, or even network endpoints, and
/// allows for logging at different levels of severity. This versatility makes it suitable for a
/// wide range of applications and use cases.
///
/// A key feature of the Logger is its ability to be specialized. Users can create a custom logger
/// specialization (for example, targeting a string, standard error, a file, or a network endpoint)
/// according to their specific needs. Once created, this specialized logger can be configured and
/// used consistently across various components of an application.
///
/// In addition to its flexibility in terms of output and configuration, the Logger class supports
/// both singleton and regular class usage patterns. This provides further versatility in how
/// logging functionality can be integrated into an application.
///
/// The following examples illustrate how to use the Logger in both singleton and regular class
/// contexts.
///
/// ```cpp
/// class SingletonLogger : public Logger {
///  public:
///   static SingletonLogger& instance() {
///     static SingletonLogger instance;
///     return instance;
///   }
///
///  private:
///   SingletonLogger() {
///     // Set default logger
///     logger_ = std::make_shared<MyLogger>();
///     func_ = nullptr;
///
///     // Set level
///     level(logger_->level());
///
///     // Set pattern
///     pattern(logger_->pattern());
///
///     // Set sinks
///     for (int severity = kNumSeverity - 1; severity >= 0; --severity) {
///       redirect(severity, s_sinks[severity]);
///     }
///   }
/// };
/// // ...
/// SingletonLogger::instance().log(__FILE__, __LINE__, __FUNCTION__, 0, "Hello World!");
/// ```
///
/// The following example shows how to use Logger as a regular class:
///
/// ```cpp
/// class TestLogger : public Logger {
///  public:
///   static std::shared_ptr<TestLogger> create() { return std::make_shared<TestLogger>(); }
/// };
/// // ...
/// std::shared_ptr<TestLogger> logger = TestLogger::create();
/// logger->log(__FILE__, __LINE__, __FUNCTION__, 0, "Hello World!");
/// ```
class Logger {
 public:
  /// Construct a logger instance.
  Logger() = default;

  /// Construct a logger instance with the custom logger.
  ///
  /// @param logger A shared pointer to the custom logger.
  explicit Logger(const std::shared_ptr<ILogger>& logger);

  /// Construct a logger instance with the custom log function.
  ///
  /// @param func The custom log function.
  explicit Logger(const LogFunction& func);

  /// Construct a logger instance with the custom logger or log function.
  ///
  /// @param logger A shared pointer to the custom logger.
  /// @param func The custom log function.
  Logger(const std::shared_ptr<ILogger> logger, const LogFunction& func);

  /// Log a message.
  ///
  /// Log a message using either a custom logger set via logger() or func().
  /// The custom logger set via func() has priority over the one set via logger(). If neither is
  /// set, no logging occurs.
  ///
  /// @param file The name of the source file where the log request originated.
  /// @param line The line number in the source file.
  /// @param name The name of the function/module making the log request.
  /// @param level The log level.
  /// @param message The log message.
  void log(const char* file, int line, const char* name, int level, const char* message);

  /// Set a custom logger.
  ///
  /// Register a custom logger to be used for logging messages. The custom logger must implement
  /// the ILogger interface.
  ///
  /// @param custom_logger A shared pointer to the custom logger.
  void logger(std::shared_ptr<ILogger> custom_logger);

  /// Get the current custom logger.
  ///
  /// Retrieve the currently set custom logger.
  ///
  /// @return A shared pointer to the currently set custom logger.
  std::shared_ptr<ILogger> logger() const;

  /// Set a custom log function.
  ///
  /// Register a custom log function and an argument to be used for logging messages. This function
  /// has higher priority over the custom logger set via logger().
  /// If func_arg is not nullptr, func_arg will be passed to log_func as an argument ('arg'
  /// parameter).
  ///
  /// @param log_func The custom log function.
  /// @param func_arg An opaque pointer to auxiliary data needed by log_func.
  void func(LogFunction log_func, void* func_arg = nullptr);

  /// Get the current custom log function.
  ///
  /// Retrieve the currently set custom log function.
  ///
  /// @return The currently set custom log function.
  LogFunction func() const;

  /// Get the current custom log function argument.
  ///
  /// Retrieve the currently set custom log function argument.
  ///
  /// @return The currently set custom log function argument.
  void* arg() const;

  /// Set the log message pattern.
  ///
  /// Set the pattern used for formatting log messages.
  ///
  /// @param pattern The pattern string for formatting log messages.
  void pattern(const char* pattern);

  /// Get the current log message pattern.
  ///
  /// Retrieve the currently set pattern used for formatting log messages.
  ///
  /// @return The currently set pattern string.
  const char* pattern() const;

  /// Set the log level.
  ///
  /// Set the log level, determining the priority of messages that should be logged.
  ///
  /// @param level The log level.
  void level(int level);

  /// Get the current log level.
  ///
  /// Retrieve the currently set log level.
  ///
  /// @return The currently set log level.
  int level() const;

  /// Redirect log output.
  ///
  /// Redirect log output to the specified file or other output for the given log level.
  ///
  /// @param level The log level for which output redirection is applied.
  /// @param output A pointer to the file or other output to redirect log output.
  void redirect(int level, void* output);

  /// Get the current redirection for a specified log level.
  ///
  /// Retrieve the file/output pointer to which log output is currently being redirected for the
  /// specified log level. If no redirection is set for the given level, returns nullptr.
  ///
  /// @param level The log level for which the redirection is queried.
  /// @return A pointer to the file/output where log output is redirected, or nullptr if no
  /// redirection is set.
  void* redirect(int level) const;

 protected:
  std::shared_ptr<ILogger> logger_;  ///< shared pointer to the logger
  LogFunction func_;                 ///< log function
  void* func_arg_ = nullptr;         ///< log function argument

  std::string pattern_;       ///< log pattern
  int level_ = 0;             ///< log level
  std::vector<void*> sinks_;  ///< log sinks
};

}  // namespace logger

}  // namespace nvidia

#endif /* COMMON_LOGGER_LOGGER_HPP */
