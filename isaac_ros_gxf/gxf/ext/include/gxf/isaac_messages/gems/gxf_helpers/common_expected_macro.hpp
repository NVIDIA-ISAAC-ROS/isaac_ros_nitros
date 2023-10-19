// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
  ::nvidia::internal::LogHelper(                                                               \
      __FILE__, __LINE__, expression_result, expression_string, ##__VA_ARGS__);

#define EXPECTED_MACRO_INTERNAL_CHECK_EXPRESSION_IS_EXPECTED_VOID_OR_RESULT(expression_result) \
  static_assert(                                                                                 \
        ::nvidia::internal::IsExpectedVoid<decltype(maybe_result)>::value ||                     \
        std::is_enum_v<decltype(maybe_result)>,                                                  \
        EXPECTED_MACRO_INTERNAL_FILE_LINE() ": GXF_RETURN_IF_ERROR can only be used with "       \
        "expressions that return Expected<void> or enum. For expressions returning Expected<T> " \
        "use GXF_UNWRAP_OR_RETURN instead.");

#define EXPECTED_MACRO_INTERNAL_CHECK_EXPRESSION_IS_EXPECTED_T(expression_result) \
  static_assert(::nvidia::internal::IsExpectedT<decltype(maybe_result)>::value,                  \
        EXPECTED_MACRO_INTERNAL_FILE_LINE() ": GXF_UNWRAP_OR_RETURN can only be used with "      \
        "expressions that return Expected<T>. For expressions returning Expected<void> or enum " \
        "use RETURN_IF_ERROR instead.");

// Evaluates an expression that returns an Expected<void> or result
// enum. If the returned type contains an error it returns the
// error. This macro can be used in functions returning both
// Expected<T> and result enum.
//
// Per default the macro already creates an error message that
// includes the evaluated expression. If needed an optional string can
// be passed that will be appended to the default error message. It is
// also possible to use format specifiers to customize the string.
//
// It is also possible to pass the Severity used for logging as an
// additional argument. This is required if using a custom error
// message.
//
// Example:
// Expected<void> DoSomething();
// Expected<void> DoAnotherThing();
//
// Expected<void> foo(){
//   GXF_RETURN_IF_ERROR(DoSomething());
//   GXF_RETURN_IF_ERROR(DoAnotherThing(), Severity::WARNING);
//   GXF_RETURN_IF_ERROR(DoAnotherThing(), Severity::WARNING, "Custom error message.");
// }
#define RETURN_IF_ERROR(expression, ...)                                                    \
  do {                                                                                      \
    auto maybe_result = (expression);                                                       \
    EXPECTED_MACRO_INTERNAL_CHECK_EXPRESSION_IS_EXPECTED_VOID_OR_RESULT(maybe_result)       \
    if (!::nvidia::internal::IsValid(maybe_result)) {                                       \
      EXPECTED_MACRO_INTERNAL_LOG_IN_EXPECT_MACRO(maybe_result, #expression, ##__VA_ARGS__) \
      return ::nvidia::internal::ProxyFactory::FromExpectedVoidOrResult(maybe_result);      \
    }                                                                                       \
  } while (0)

// Evaluates an expression that returns an Expected<T>. If the returned type
// contains an error it returns the error, else it unwraps the value contained in the Expected<T>.
// This macro can be used in functions returning both Expected<T> and result enum.
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
//   const std::string str1 = GXF_UNWRAP_OR_RETURN(GetString());
//   std::string str2;
//   str2 = GXF_UNWRAP_OR_RETURN(GetAnotherString(), "This should not fail. Str1 has value %s.",
//       str1.c_str());
//   const std::string str3 = GXF_UNWRAP_OR_RETURN(GetAnotherString(), Severity::WARNING);
//   const std::string str4 = GXF_UNWRAP_OR_RETURN(GetAnotherString(), Severity::WARNING,
//       "Custom error message");
//   return str1.size() + str2.size() + str3.size() + str4.size();
// }
#define UNWRAP_OR_RETURN(expression, ...)                                                   \
  ({                                                                                        \
    auto maybe_result = (expression);                                                       \
    EXPECTED_MACRO_INTERNAL_CHECK_EXPRESSION_IS_EXPECTED_T(maybe_result)                    \
    if (!::nvidia::internal::IsValid(maybe_result)) {                                       \
      EXPECTED_MACRO_INTERNAL_LOG_IN_EXPECT_MACRO(maybe_result, #expression, ##__VA_ARGS__) \
      return ::nvidia::internal::ProxyFactory::FromExpectedValue(maybe_result);             \
    }                                                                                       \
    std::move(maybe_result.value());                                                        \
  })

namespace nvidia {
// This struct has to be specialized for a given result enum.
template <typename Error>
struct ExpectedMacroConfig {
  constexpr static Error DefaultSuccess();
  constexpr static Error DefaultError();
  static std::string Name(Error error) { return std::to_string(static_cast<int>(error)); }
};
}  // namespace nvidia

namespace nvidia::internal {

constexpr Severity kDefaultSeverity = Severity::ERROR;

template <typename>
struct IsExpectedVoid : public std::false_type {};

template <typename Error>
struct IsExpectedVoid<Expected<void, Error>> : public std::true_type {};

template <typename>
struct IsExpectedT : public std::false_type {};

template <typename Error>
struct IsExpectedT<Expected<void, Error>> : public std::false_type {};

template <typename Value, typename Error>
struct IsExpectedT<Expected<Value, Error>> : public std::true_type {};

// Returns true if the passed result is valid.
template <typename Error>
constexpr bool IsValid(Error result) {
  return result == ExpectedMacroConfig<Error>::DefaultSuccess();
}

// Returns true if the passed expected is valid.
template <typename Value, typename Error>
constexpr bool IsValid(const Expected<Value, Error>& expected) {
  return static_cast<bool>(expected);
}

template <typename Value, typename Error>
constexpr Error GetError(const Expected<Value, Error>& expected) {
  return expected.error();
}

template <typename Error>
constexpr Error GetError(const Error error) {
  return error;
}

template <typename Error>
class UnexpectedOrErrorProxy;

class ProxyFactory {
 public:
  // Constructs the proxy from a Expected<T>. The Expected<T> has to be in an error state. We
  // do not check this because the macro should have already done this check. We use static
  // methods instead of constructors to explicitly disallow construction from certain types in
  // different situations.
  template <typename Value, typename Error>
  static UnexpectedOrErrorProxy<Error> FromExpectedValue(const Expected<Value, Error>& expected) {
    static_assert(
        !std::is_same<Value, void>::value,
        "This function should not be used with Expected<void, Error>, use "
        "FromExpectedVoidOrResult() instead.");
    return UnexpectedOrErrorProxy(expected);
  }

  // Constructs the proxy from a Expected<void>. The Expected<void> has to be in an error state.
  // We do not check this because the macro should have already done this check. This function
  // needs to be overloaded with a function taking in a result enum. We use static methods instead
  // of constructors to explicitly disallow construction from certain types in different
  // situations.
  template <typename Error>
  static UnexpectedOrErrorProxy<Error> FromExpectedVoidOrResult(
      const Expected<void, Error> expected) {
    return UnexpectedOrErrorProxy(expected);
  }

  // Constructs the proxy from a result enum. The result enum has to be in an error state. We do
  // not check this because the macro should have already done this check. This function needs to
  // be overloaded with a function taking in an Expected<void>. We use static methods instead of
  // constructors to explicitly disallow construction from certain types in different situations.
  template <typename Error>
  constexpr static UnexpectedOrErrorProxy<Error> FromExpectedVoidOrResult(Error error) {
    return UnexpectedOrErrorProxy(error);
  }
};

// A proxy class to abstract away the difference between a result enum and an Expected<T>.
// This class defines casting operators s.t. it can implicitly cast to both result enum and
// Expected<T>. Thus in a function that returns one of the afore mentioned types one can simply
// return this proxy and then it will implicitly cast to the appropriate return type.
template <typename Error>
class UnexpectedOrErrorProxy {
 public:
  // Casts the proxy to an error. Note that this cast is not allowed to be explicit. The
  // OtherError has to be an enum (or enum class).
  template <typename OtherError, typename = std::enable_if_t<std::is_enum_v<OtherError>>>
  constexpr operator OtherError() const {
    return castToError<OtherError>();
  }

  // Casts the proxy to an Expected<T>. Note that this cast is not allowed to be explicit.
  template <typename Value, typename OtherError>
  constexpr operator Expected<Value, OtherError>() const {
    return castToUnexpected<OtherError>();
  }

 private:
  // Constructs the proxy from a Expected<T>. Note that the Expected<T> needs to contain an error.
  // We do not check for this because we rely on the macro already having done that check.
  template <typename Value>
  constexpr explicit UnexpectedOrErrorProxy(const Expected<Value, Error>& expected)
      : error_(expected.error()) {}

  // Constructs the proxy from a result enum. The result enum needs to be in an error state.
  // We do not check for this because we rely on the macro already having done that check.
  constexpr explicit UnexpectedOrErrorProxy(Error error) : error_(error) {}

  // Casts the proxy to any error type. If the error type is not equal to the proxy's error type,
  // a default error is used.
  template <typename OtherError>
  constexpr OtherError castToError() const {
    static_assert(std::is_enum_v<OtherError>, "Can only cast to errors of type enum.");
    if constexpr (std::is_same_v<OtherError, Error>) {
      return error_;
    } else {
      return ExpectedMacroConfig<OtherError>::DefaultError();
    }
  }

  // Casts the proxy to an unexpected using any error type. If the error type is not equal to the
  // proxy's error type, a default error is used.
  template <typename OtherError>
  constexpr Unexpected<OtherError> castToUnexpected() const {
    return Unexpected<OtherError>(castToError<OtherError>());
  }

  Error error_;
  static_assert(std::is_enum_v<Error>, "Error has to be an enum.");

  friend ProxyFactory;
};

// Helper function for the logging in the above macros. This version should be used when the user
// also specifies the logging severity. The variadic arguments can be used to do string
// interpolation in the custom_text variable.
template <typename ExpressionResult, typename... Args>
void LogHelper(
    const char* file, int line, const ExpressionResult& expression_result,
    const std::string& expression_string, Severity severity, const std::string& custom_txt = "",
    Args... args) {
  const auto error = GetError(expression_result);
  using Error = std::remove_const_t<decltype(error)>;
  const std::string text = "Expression '" + expression_string + "' failed with error '" +
                           ExpectedMacroConfig<Error>::Name(error) + "'. " + custom_txt;
  // GCC is not able to do format security validation when the string
  // is coming from a variadic template, even if the string is
  // originally a char* ignore this warning until a more recent GCC
  // version fixes this behavior
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
  ::nvidia::Log(file, line, severity, text.c_str(), &args...);
#pragma GCC diagnostic pop
}

// Overload of the LogHelper above. This version does not take the severity as an argument and used
// the default severity instead.
template <typename ExpressionResult, typename... Args>
void LogHelper(
    const char* file, int line, const ExpressionResult& expression_result,
    const std::string& expression_string, const std::string& custom_text = "", Args... args) {
  LogHelper(
      file, line, expression_result, expression_string, kDefaultSeverity, custom_text, &args...);
}

}  // namespace nvidia::internal
