/*
Copyright (c) 2018-2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdarg>
#include <cstdio>
#include <vector>

// Logs a debug message
#define LOG_DEBUG(...) \
  ::isaac::logger::Log(__FILE__, __LINE__, ::isaac::logger::Severity::DEBUG, __VA_ARGS__)

// Logs an informational message
#define LOG_INFO(...) \
  ::isaac::logger::Log(__FILE__, __LINE__, ::isaac::logger::Severity::INFO, __VA_ARGS__)

// Logs a warning
#define LOG_WARNING(...) \
  ::isaac::logger::Log(__FILE__, __LINE__, ::isaac::logger::Severity::WARNING, __VA_ARGS__)

// Logs an error
#define LOG_ERROR(...) \
  ::isaac::logger::Log(__FILE__, __LINE__, ::isaac::logger::Severity::ERROR, __VA_ARGS__)

namespace isaac {
namespace logger {

// Indicates the level of severity for a log message
enum class Severity {
  // A utility case which can be used for `SetSeverity` to disable all severity levels.
  NONE = -2,
  // A utility case which can be used for `Redirect` and `SetSeverity`
  ALL = -1,
  // The five different log severities in use from most severe to least severe.
  PANIC = 0,  // Need to start at 0
  ERROR,
  WARNING,
  INFO,
  DEBUG,
  // A utility case representing the number of log levels used internally.
  COUNT
};

// Function which is used for logging. It can be changed to intercept the logged messages.
extern void (*LoggingFunction)(const char* file, int line, Severity severity, const char* log);

// Redirects the output for a given log severity.
void Redirect(std::FILE* file, Severity severity = Severity::ALL);

// Redirects the output for a given file log severity up to and including a given log severity.
void RedirectFileLog(std::FILE* file, Severity severity = Severity::DEBUG);

// Sets global log severity thus effectively disabling all logging with lower severity
void SetSeverity(Severity severity);

// Converts the message and argument into a string and pass it to LoggingFunction.
template<typename... Args>
void Log(const char* file, int line, Severity severity, const char* txt, ...) {
  va_list args1;
  va_start(args1, txt);
  va_list args2;
  va_copy(args2, args1);
  std::vector<char> buf(1 + std::vsnprintf(NULL, 0, txt, args1));
  va_end(args1);
  std::vsnprintf(buf.data(), buf.size(), txt, args2);
  va_end(args2);
  LoggingFunction(file, line, severity, buf.data());
}

// Helper datastructure for Singleton
struct SeverityContainer {
  Severity r = Severity::DEBUG;
};

}  // namespace logger
}  // namespace isaac
