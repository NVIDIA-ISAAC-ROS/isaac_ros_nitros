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

#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "common/expected.hpp"

// Concatenates its two arguments.
#define EXPECTED_MACRO_INTERNAL_CONCAT(a, b) EXPECTED_MACRO_INTERNAL_CONCAT_IMPL(a, b)
#define EXPECTED_MACRO_INTERNAL_CONCAT_IMPL(a, b) a##b

// Converts its argument to a string at compile time.
#define EXPECTED_MACRO_INTERNAL_TO_STRING(x) EXPECTED_MACRO_INTERNAL_TO_STRING_IMPL(x)
#define EXPECTED_MACRO_INTERNAL_TO_STRING_IMPL(x) #x

// Gets the current location in the source code in the format "file:line".
#define EXPECTED_MACRO_INTERNAL_FILE_LINE() __FILE__ ":" EXPECTED_MACRO_INTERNAL_TO_STRING(__LINE__)

// Helper to support logging a default message and an optional custom message.
#define EXPECTED_MACRO_INTERNAL_LOG_IN_EXPECT_MACRO(expression_result, expression_string, ...) \
  nvidia::expected_macro::LogHelper(__FILE__, __LINE__, expression_result, expression_string,      \
                                ##__VA_ARGS__);

// Helper to check the return type of the expression used in RETURN_IF_ERROR.
#define EXPECTED_MACRO_INTERNAL_CHECK_EXPRESSION_IS_RESULT(expression_result) \
  static_assert(                                                                                   \
      nvidia::expected_macro::IsStatus<decltype(expression_result)>::value ||              \
      nvidia::expected_macro::IsUnwrappable<decltype(expression_result)>::value ,          \
      EXPECTED_MACRO_INTERNAL_FILE_LINE()                                                          \
          ": RETURN_IF_ERROR can only be used with expressions that return a result type. You "    \
          "can register a type as a result type unwrappable with one of"                           \
          "`template <> struct IsUnwrappable<MyType> : std::true_type {};` or "                    \
          "`template <> struct IsStatus<MyType> : std::true_type {}`. ");

// Helper to check the return type of the expression used in UNWRAP_OR_RETURN.
#define EXPECTED_MACRO_INTERNAL_CHECK_EXPRESSION_IS_UNWRAPPABLE(expression_result) \
  static_assert(                                                                                   \
      nvidia::expected_macro::IsUnwrappable<decltype(expression_result)>::value,           \
      EXPECTED_MACRO_INTERNAL_FILE_LINE()                                                          \
          ": UNWRAP_OR_RETURN can only be used with expressions that return an unwrappable type. " \
          "You can register a type as being unwrappable with "                                     \
          "`template <> struct IsUnwrappable<MyType> : std::true_type {};`. For expressions "      \
          "returning a status instead of an unwrappable use `RETURN_IF_ERROR` instead.");

// Evaluates an expression that returns a result type. If the returned result contains an error it
// returns the error. This macro can only be used in functions that also return a result type.
//
// Per default the macro already creates an error message that includes the evaluated expression. If
// needed an optional string can be passed that will be appended to the default error message. It is
// also possible to use format specifiers to customize the string.
//
// It is also possible to pass the Severity used for logging as an additional argument. This is
// required if using a custom error message.
//
// Example:
// Expected<void> DoSomething();
// Expected<void> DoAnotherThing();
//
// Expected<void> foo(){
//   RETURN_IF_ERROR(DoSomething());
//   RETURN_IF_ERROR(DoAnotherThing(), Severity::WARNING);
//   RETURN_IF_ERROR(DoAnotherThing(), Severity::WARNING, "Custom error message.");
// }
#define RETURN_IF_ERROR(expression, ...)                                                    \
  ({                                                                                      \
    auto maybe_result = (expression);                                                       \
    EXPECTED_MACRO_INTERNAL_CHECK_EXPRESSION_IS_RESULT(maybe_result)                        \
    if (!nvidia::expected_macro::IsValid(maybe_result)) {                           \
      EXPECTED_MACRO_INTERNAL_LOG_IN_EXPECT_MACRO(maybe_result, #expression, ##__VA_ARGS__) \
      return nvidia::expected_macro::ProxyFactory::FromStatusOrUnwrappable(maybe_result);   \
    }                                                                                       \
  })

// Evaluates an expression that returns a type that can be casted to a boolean. If the expression is
// false it returns an error. This macro can only be used in functions returning a result type.
//
// The difference to RETURN_IF_ERROR is that this macro will always cast the expression to a
// boolean. Thus if your expression returns a status that can be more than just true/false you will
// loose the additional information. In those cases RETURN_IF_ERROR is preferred. Use this macro
// only to check preconditions like a < b, or c != nullptr.
//
// Per default the macro already creates an error message that includes the evaluated expression. If
// needed an optional string can be passed that will be appended to the default error message. It is
// also possible to use format specifiers to customize the string.
//
// It is also possible to pass the Severity used for logging as an additional argument. This is
// required if using a custom error message.
//
// Example:
// Expected<void> foo(){
//   RETURN_IF_FALSE(1 > 2);
//   RETURN_IF_FALSE(1 > 2, Severity::WARNING);
//   RETURN_IF_FALSE(1 > 2, Severity::WARNING, "1 is not bigger than 2");
// }
#define RETURN_IF_FALSE(expression, ...)                                              \
  do {                                                                                \
    const bool result = static_cast<bool>(expression);                                \
    if (!result) {                                                                    \
      EXPECTED_MACRO_INTERNAL_LOG_IN_EXPECT_MACRO(result, #expression, ##__VA_ARGS__) \
      return nvidia::expected_macro::ProxyFactory::FromStatusOrUnwrappable(result);   \
    }                                                                                 \
  } while (0)

// Evaluates an expression that returns an unwrappable type. If the returned type contains an error
// it returns the error, else it unwraps the value. This macro can only be used in functions
// returning a result type.
//
// Per default the macro already creates an error message that includes the evaluated expression. If
// needed an optional string can be passed that will be appended to the default error message. It is
// also possible to use format specifiers to customize the string.
//
// It is also possible to pass the Severity used for logging as an additional argument.
//
// Note that this macro uses expression-statements (i.e. the ({ }) surrounding the macro) which are
// a non-standard functionality. However they are present in almost all compilers. We currently only
// know of MSVC that does not support this.
//
// Example:
// Expected<std::string> GetString();
// Expected<std::string> GetAnotherString();
//
// Expected<int> CountCombinedStringLength(){
//   const std::string str1 = UNWRAP_OR_RETURN(GetString());
//   std::string str2;
//   str2 = UNWRAP_OR_RETURN(GetAnotherString(), "This should not fail. Str1 has value %s.",
//       str1.c_str());
//   const std::string str3 = UNWRAP_OR_RETURN(GetAnotherString(), Severity::WARNING);
//   const std::string str4 = UNWRAP_OR_RETURN(GetAnotherString(), Severity::WARNING,
//       "Custom error message");
//   return str1.size() + str2.size() + str3.size() + str4.size();
// }
#define UNWRAP_OR_RETURN(expression, ...)                                                   \
  ({                                                                                        \
    auto maybe_result = (expression);                                                       \
    EXPECTED_MACRO_INTERNAL_CHECK_EXPRESSION_IS_UNWRAPPABLE(maybe_result)                    \
    if (!nvidia::expected_macro::IsValid(maybe_result)) {                                       \
      EXPECTED_MACRO_INTERNAL_LOG_IN_EXPECT_MACRO(maybe_result, #expression, ##__VA_ARGS__) \
      return nvidia::expected_macro::ProxyFactory::FromUnwrappable(maybe_result);             \
    }                                                                                       \
    std::move(maybe_result.value());                                                        \
  })

// All functions in the following namespace have to be specialized for a given result type in order
// to enable the macro for the type.
namespace nvidia::expected_macro {

// Type trait to check if a type can be used as a status.
template <typename /*Type*/>
struct IsStatus : std::false_type {};

// Type trait to check if a type can be used as an unwrappable.
template <typename /*Type*/>
struct IsUnwrappable : std::false_type {};

// The value that represents the default error for a given status type.
template <typename Status>
constexpr Status DefaultError();

// Check if a result types is valid.
template <typename Result>
constexpr bool IsValid(const Result& result) {
  return static_cast<bool>(result);
}

// Helper struct to get the status type of a result. This is needed for the StatusValue helper below
// to automatically infer the return type.
//
// Per default we use the same type as the
// result, which holds for all status types. Thus this only has to be specialized for unwrappable
// types.
template <typename Result>
struct StatusType {
  using Type = Result;
};

// Helper function to get the status of a result. We have to use a static member function instead of
// a free function because we want the possibility for a user to only provide a partial template
// specialization of this.
template <typename Result>
struct StatusValue {
  static constexpr typename StatusType<Result>::Type Get(const Result& result) { return result; }
};

// Get the name corresponding to a status, s.t. it can be printed.
template <typename Status, typename = void>
struct StatusName {
  static std::string Get(Status status);
};

// Get the invalid unwrappable corresponding to an unwrappable. Eg. std::nullopt for std::optional.
template <typename Unwrappable, typename Status, typename = void>
struct InvalidUnwrappable {
  static Unwrappable Get(Status status);
};

constexpr Severity kDefaultSeverity = Severity::ERROR;

template <typename Result>
constexpr typename StatusType<Result>::Type GetStatus(const Result& result) {
  return StatusValue<Result>::Get(result);
}

template <typename Status>
std::string GetStatusName(Status status) {
  return StatusName<Status>::Get(status);
}

template <typename Unwrappable, typename Status>
Unwrappable GetInvalidUnwrappable(Status status) {
  return InvalidUnwrappable<Unwrappable, Status>::Get(status);
}

// ProxyFactory to create an ResultProxy. We create a separate class for these functions
// because ResultProxy is templated and thus could not be used without explicitly
// specifying the template.

template <typename Status>
class ResultProxy;

// ProxyFactory to create an ResultProxy. We create a separate class for these functions
// because ResultProxy is templated and thus could not be used without explicitly
// specifying the template.
class ProxyFactory {
 public:
  // Constructs the proxy from an unwrappable type. The unwrappable has to be in an error state. We
  // do not check this because the macro should have already done this check. We use static
  // methods instead of constructors to explicitly disallow construction from certain types in
  // different situations.
  template <
      typename Unwrappable, typename = std::enable_if_t<IsUnwrappable<Unwrappable>::value>>
  static ResultProxy<typename StatusType<Unwrappable>::Type> FromUnwrappable(
      const Unwrappable& unwrappable) {
    return ResultProxy(GetStatus(unwrappable));
  }

  // Constructs the proxy from a status type. The status has to be in an error state. We do not
  // check this because the macro should have already done this check. We use static methods instead
  // of constructors to explicitly disallow construction from certain types in different situations.
  template <typename Result>
  static ResultProxy<typename StatusType<Result>::Type> FromStatusOrUnwrappable(
      const Result& result) {
    return ResultProxy(GetStatus(result));
  }
};

// A proxy class to allow implicit casting of one result type to another result type. Thus in a
// function that returns a result type one can simply return this proxy and then it will implicitly
// cast to the appropriate return type.
template <typename Status>
class ResultProxy {
 public:
  // Casts the proxy to a status or unwrappable type. Note that this cast is not allowed to be
  // explicit.
  template <
      typename T,
      typename = std::enable_if_t<IsStatus<T>::value || IsUnwrappable<T>::value>>
  constexpr operator T() const {
    if constexpr (IsStatus<T>::value) {
      return castToStatus<T>();
    } else {
      return castToInvalidUnwrappable<T>();
    }
  }

 private:
  // Constructs the proxy from a status type. The status has to be in an error state.
  // We do not check for this because we rely on the macro already having done that check.
  constexpr explicit ResultProxy(Status error) : error_(error) {}

  // Casts the proxy to a status type. If the error type is not equal to the proxy's error type,
  // a default error is used.
  template <typename OtherStatus>
  constexpr OtherStatus castToStatus() const {
    static_assert(IsStatus<OtherStatus>::value, "OtherStatus has to be a status type.");
    if constexpr (std::is_same_v<OtherStatus, Status>) {
      return error_;
    } else {
      return DefaultError<OtherStatus>();
    }
  }

  // Casts the proxy to an invalid unwrappable type (eg. std::nullopt or nvidia::Unexpected).
  template <typename Unwrappable>
  constexpr Unwrappable castToInvalidUnwrappable() const {
    static_assert(
        IsUnwrappable<Unwrappable>::value, "Unwrappable has to be a status type.");
    using OtherStatus = typename StatusType<Unwrappable>::Type;
    return GetInvalidUnwrappable<Unwrappable>(castToStatus<OtherStatus>());
  }

  Status error_;
  // TODO(tazhang): enable the following.
  // static_assert(IsStatus<Status>::value, "Status has to be a status type.");

  friend ProxyFactory;
};

// Helper function for the logging in the above macros. This version should be used when the user
// also specifies the logging severity. The variadic arguments can be used to do string
// interpolation in the custom_text variable.
template <typename ExpressionResult, typename... Args>
void LogHelper(const char* file, int line, const ExpressionResult& expression_result,
               const std::string& expression_string, Severity severity,
               const std::string& custom_txt = "", Args... args) {
  const auto error = GetStatus(expression_result);
  const std::string text = "Expression '" + expression_string + "' failed with error '" +
                           GetStatusName(error) + "'. " + custom_txt;

  // GCC is not able to do format security validation when the string is coming from a variadic
  // template, even if the string is originally a char* ignore this warning until a more recent GCC
  // version fixes this behavior.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
  ::nvidia::Log(file, line, severity, text.c_str(), args...);
#pragma GCC diagnostic pop
}

// Overload of the LogHelper above. This version does not take the severity as an argument and used
// the default severity instead.
template <typename ExpressionResult, typename... Args>
void LogHelper(const char* file, int line, const ExpressionResult& expression_result,
               const std::string& expression_string, const std::string& custom_text = "",
               Args... args) {
  LogHelper(file, line, expression_result, expression_string, kDefaultSeverity, custom_text,
            args...);
}

}  // namespace nvidia::expected_macro
