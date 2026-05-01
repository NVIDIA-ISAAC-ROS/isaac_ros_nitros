// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES.
//                         All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

//! @file
//! @brief Shared interface functions for the cuMotion library.

#pragma once

#include <exception>
#include <memory>
#include <ostream>
#include <string>

#include "cumotion/cumotion_export.h"
#include "cumotion/version.h"

namespace cumotion {

//! @brief Get cuMotion library version as a `std::string`.
//!
//! Also see the `CUMOTION_VERSION_STRING` macro, which serves a similar purpose.  The two should
//! always agree.  If they don't, it indicates a mismatch between the installed cuMotion headers
//! and installed cuMotion libraries.
CUMO_EXPORT std::string VersionString();

// On Windows, `wingdi.h` (included transitively via `windows.h` and certain CUDA headers)
// contains a `#define ERROR 0` that collides with the `LogLevel::ERROR` enum value.
// The following temporarily undefines the macro.
#ifdef _WIN32
#  pragma push_macro("ERROR")
#  undef ERROR
#endif

//! @brief Logging levels, ordered from least to most verbose.
enum class CUMO_EXPORT LogLevel {
  FATAL,    //!< Logging level for nonrecoverable errors (minimum level, so always enabled).
  ERROR,    //!< Logging level for recoverable errors.
  WARNING,  //!< Logging level for warnings, indicating possible cause for concern.
  INFO,     //!< Logging level for informational messages.
  VERBOSE   //!< Logging level for highly verbose informational messages.
};

#ifdef _WIN32
#  pragma pop_macro("ERROR")
#endif

//! @brief Base class for user-defined logger that allows custom handling of log messages, warnings,
//! and errors.
//!
//! Such a custom logger may be installed via `SetLogger()`.
class CUMO_EXPORT Logger {
 public:
  virtual ~Logger() = default;

  //! Return an `ostream` suitable for logging messages at the given `log_level`.
  virtual std::ostream &log(LogLevel log_level) = 0;

  //! Perform any required post-processing of individual log messages.  This function is called
  //! after the message has been logged to the `std::ostream` returned by `log()` but before
  //! `handleFatalError()` is called (if applicable).
  virtual void finalizeLogMessage([[maybe_unused]] LogLevel log_level) {
    // Do nothing by default.
  }

  //! Handle fatal errors.  This callback will be called after any associated error message has
  //! been written to the ostream returned by `log(FATAL)`.  Because fatal errors are
  //! non-recoverable, this function must ensure that any active cuMotion objects are discarded.
  virtual void handleFatalError() = 0;
};

//! @brief Exception thrown by the default logger for fatal errors.
//!
//! This is the only exception thrown by cuMotion, so clients may avoid exceptions completely by
//! installing a custom logger.
class CUMO_EXPORT FatalException : public std::exception {
 public:
  explicit FatalException(const std::string &message) : message_(message) {}

  [[nodiscard]] const char *what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

//! @brief Suppress output for all log messages with associated verbosity higher than `log_level`.
//!
//! Messages suppressed in this way incur very low overhead, so it's best to take advantage of
//! this facility even if a custom `Logger` is provided.
//!
//! Until `SetLogLevel()` is called, the default log level is `WARNING`.  The lowest supported
//! `log_level` is `FATAL`, since it is not possible to supress fatal errors.
CUMO_EXPORT void SetLogLevel(LogLevel log_level);

//! @brief Install a custom logger, derived from the above `Logger` class.
//!
//! If `logger` is a null pointer, then the default logger is reenabled.  The default logger
//! directs all output to stdout and throws a `FatalException` in the event of a fatal error.
CUMO_EXPORT void SetLogger(std::shared_ptr<Logger> logger = nullptr);

//! Set color/style used by the default logger for messages of a given `log_level`.  The `style`
//! string may contain one or more ANSI control sequences (e.g., enabling fatal error messages to
//! be rendered in bold with a red foreground color).
//!
//! For convenience, a selection of common control sequences is provided in
//! include/cumotion/text_style.h
CUMO_EXPORT void SetDefaultLoggerTextStyle(LogLevel log_level, const std::string &style);

//! Set prefix used by the default logger for all messages. The `prefix` string is logged after
//! the `style` string set for each `log_level` by `SetDefaultLoggerTextStyle()` and prior to the
//! content of the logged message.
//!
//! The default prefix is an empty string.
CUMO_EXPORT void SetDefaultLoggerPrefix(const std::string &prefix);

}  // namespace cumotion
