/*
 * SPDX-FileCopyrightText: Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_GXF_CORE_PARAMETER_HPP_
#define NVIDIA_GXF_CORE_PARAMETER_HPP_

#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include "common/assert.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"
#include "gxf/core/handle.hpp"
#include "gxf/std/parameter_parser.hpp"
#include "gxf/std/parameter_wrapper.hpp"


namespace nvidia {
namespace gxf {

// Base class for parameters stored in ParameterStorage
class ParameterBackendBase {
 public:
  virtual ~ParameterBackendBase() = default;

  // The context to which this parameter backend belongs
  gxf_context_t context() const { return context_; }

  // The object to which this parameter is attached.
  gxf_uid_t uid() const { return uid_; }

  // The name of the parameter
  const char* key() const { return key_; }

  // Returns true if the parameter is guaranteed to always have a value set. Only mandatory
  // parameters can be accessed direclty with 'get' instead of using 'try_get'.
  bool isMandatory() const { return (flags_ & GXF_PARAMETER_FLAGS_OPTIONAL) == 0; }

  // Returns true if the parameter can not be changed after the component has been activated.
  bool isConstant() const { return (flags_ & GXF_PARAMETER_FLAGS_DYNAMIC) == 0; }

  // Sets the latest value from the backend to the frontend.
  virtual void writeToFrontend() = 0;

  // Parses the parameter from the given YAML object.
  virtual Expected<void> parse(const YAML::Node& node, const std::string& prefix) = 0;

  // Wrap the parameter as a YAML node.
  virtual Expected<YAML::Node> wrap() = 0;

  // Returns true if the parameter is set
  virtual bool isAvailable() const = 0;

  // Returns true if it is possible to change this parameter
  bool isImmutable() const {
    if (isConstant()) {
      const bool is_active = false;
      if (is_active) {
        return true;
      }
    }
    return false;
  }

  gxf_context_t context_;
  gxf_uid_t uid_;
  gxf_parameter_flags_t flags_;
  bool is_dynamic_;
  const char* key_;
  const char* headline_;
  const char* description_;
};

template <typename T>
class Parameter;

// This class stores a parameter of a specific type in the backend.
template <typename T>
class ParameterBackend : public ParameterBackendBase {
 public:
  void writeToFrontend() override;
  Expected<void> parse(const YAML::Node& node, const std::string& prefix) override;
  bool isAvailable() const override { return value_.has_value(); }

  // Sets the parameter to the given value.
  Expected<void> set(T value) {
    // Make sure that the new value passes the validator
    if (validator_&& !validator_(value)) { return Unexpected{GXF_PARAMETER_OUT_OF_RANGE}; }
    // Don't allow modification of a parameter which is currently immutable
    if (isImmutable()) { return Unexpected{GXF_PARAMETER_CAN_NOT_MODIFY_CONSTANT}; }
    // Update the parameter value
    value_ = std::move(value);
    return Success;
  }

  Expected<YAML::Node> wrap() {
    if (!value_) {
      return Unexpected{GXF_UNINITIALIZED_VALUE};
    }
    return ParameterWrapper<T>::Wrap(context(), value_.value());
  }

  // Gets the current value of the parameter.
  const Expected<T>& try_get() const { return value_; }

  nvidia::gxf::Parameter<T>* frontend_ = nullptr;
  std::function<bool(const T&)> validator_;
  Expected<T> value_ = Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
};

// An intermediate base class for parameters which store a handle.
class HandleParameterBackend : public ParameterBackendBase {
 public:
  virtual ~HandleParameterBackend() = default;

  // Gets the component ID of the handle.
  virtual Expected<gxf_uid_t> get() const = 0;

  // Sets the handle using just a component ID
  virtual Expected<void> set(gxf_uid_t cid) = 0;
};

// A specialization of ParameterBackend<T> for handle types. It derives from the intermediate base
// class HandleParameterBackend so that parameter backends of handle types all have a common base
// class.
template <typename T>
class ParameterBackend<Handle<T>> : public HandleParameterBackend {
 public:
  void writeToFrontend() override;
  Expected<void> parse(const YAML::Node& node, const std::string& prefix) override;
  bool isAvailable() const override {
    return (value_.has_value()) && (value_ != Handle<T>::Unspecified());
  }

  Expected<YAML::Node> wrap() {
    if (!value_ || value_ == Handle<T>::Unspecified()) {
      return Unexpected{GXF_UNINITIALIZED_VALUE};
    }

    return ParameterWrapper<Handle<T>>::Wrap(context(), value_.value());
  }

  // Sets the handle using just a component ID
  Expected<void> set(gxf_uid_t cid) override {
    auto expected = Handle<T>::Create(context(), cid);
    if (expected) {
      value_ = expected.value();
      return Success;
    } else {
      return ForwardError(expected);
    }
  }

  // Gets the component ID of the handle.
  Expected<gxf_uid_t> get() const override {
    if (!value_) {
      GXF_LOG_VERBOSE("Handle parameter with name '%s' is not initialized", key());
      return Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
    } else if (value_ == Handle<T>::Unspecified()) {
      GXF_LOG_VERBOSE("Handle parameter with name '%s' is unspecified", key());
      return Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
    }
    return value_->cid();
  }

  // Sets the parameter to the given value.
  Expected<void> set(Handle<T> value) {
    if (isImmutable()) { return Unexpected{GXF_PARAMETER_CAN_NOT_MODIFY_CONSTANT}; }
    value_ = std::move(value);
    return Success;
  }

  // Gets the current value of the parameter, return Unexpected if not
  // set or if it's set to Handle<T>::Unspecified()
  const Expected<Handle<T>>& try_get() const {
    if (value_ == Handle<T>::Unspecified()) { return unspecified_handle_; }
    return value_;
  }

  const Expected<Handle<T>> unspecified_handle_ = Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
  nvidia::gxf::Parameter<Handle<T>>* frontend_ = nullptr;
  Expected<Handle<T>> value_ = Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
};


// Common base class for specializations of Parameter<T>.
class ParameterBase {
 public:
  virtual ~ParameterBase() = default;
};

// This type represents a parameter of a component and can be used in custom components. It
// communicates with the backend to set and get parameters as configured.
template <typename T>
class Parameter : public ParameterBase {
 public:
  Parameter() = default;

  // enable copying
  Parameter(const Parameter<T>& other)
  : mutex_() {
    std::unique_lock<std::mutex> lock(other.mutex_);
    value_ = other.value_;
    backend_ = other.backend_;
  }
  // Gets the current parameter value. Only valid if the parameter is marked as 'mandatory' in the
  // paramater interface. Otherwise an assert will be raised.
  const T& get() const {
    std::unique_lock<std::mutex> lock(mutex_);
    GXF_ASSERT(backend_ != nullptr, "A parameter with type '%s' was not registered.",
               TypenameAsString<T>());
    GXF_ASSERT(backend_->isMandatory(), "Only mandatory parameters can be accessed with get(). "
               "'%s' is not marked as mandatory", backend_->key());
    GXF_ASSERT(value_, "Mandatory parameter '%s' was not set.", backend_->key());
    return value_.value();
  }

  // Convenience function for accessing a mandatory parameter.
  operator const T&() const {
    return get();
  }

  // Tries to get the parameter value. If the parameter is not set Unexpected is returned.
  const Expected<T>& try_get() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return value_;
  }

  // Sets the parameter to the given value.
  Expected<void> set(T value) {
    GXF_ASSERT(backend_ != nullptr, "Parameter '%s' was not registered.", backend_->key());
    const auto result = backend_->set(value);
    if (!result) {
      return result;
    }
    value_ = std::move(value);
    return Success;
  }

  // Sets the parameter to the given value, but does not notify the backend about the change.
  // This function shall only be used by the ParameterBackend class.
  void setWithoutPropagate(const T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    value_ = value;
  }

  // Connects this parameter frontend to the corresponding backend.
  void connect(ParameterBackend<T>* backend) {
    backend_ = backend;
  }

  const char* key() {
    return backend_ == nullptr ? nullptr : backend_->key();
  }

 private:
  Expected<T> value_ = Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
  ParameterBackend<T>* backend_ = nullptr;
  mutable std::mutex mutex_;
};

// A specialization of Parameter<T> for handle types.
template <typename S>
class Parameter<Handle<S>> : public ParameterBase {
 public:
  // Gets the current parameter value. Only valid if the parameter is marked as 'mandatory' in the
  // paramater interface. Otherwise an assert will be raised.
  const Handle<S>& get() const {
    GXF_ASSERT(backend_ != nullptr, "A handle parameter with type '%s' was not registered.",
               TypenameAsString<S>());
    GXF_ASSERT(backend_->isMandatory(), "Only mandatory parameters can be accessed with get(). "
               "'%s' is not marked as mandatory", backend_->key());
    GXF_ASSERT(value_, "Mandatory parameter '%s' was not set.", backend_->key());
    GXF_ASSERT(value_ != Handle<S>::Unspecified(), "Handle was created but not assigned."
               "Unspecified handles cannot be accessed.");
    return value_.value();
  }

  // Convenience function for accessing a mandatory parameter.
  operator const Handle<S>&() const {
    return get();
  }

  // Tries to get the parameter value. If the parameter is not set or set to
  // Handle<S>::Unspecified()  is returned.
  const Expected<Handle<S>>& try_get() const {
    if (value_ == Handle<S>::Unspecified()) { return unspecified_handle_; }
    return value_;
  }

  // Only if T = Handle<S>
  S* operator->() const {
    return get().get();
  }

  // Sets the parameter to the given value.
  Expected<void> set(Handle<S> value) {
    GXF_ASSERT(backend_ != nullptr, "Parameter '%s' was not registered.", backend_->key());
    const auto result = backend_->set(value);
    if (!result) {
      return result;
    }
    value_ = std::move(value);
    return Success;
  }

  // Sets the parameter to the given value, but does not notify the backend about the change.
  // This function shall only be used by the ParameterBackend class.
  void setWithoutPropagate(const Handle<S>& value) {
    value_ = value;
  }

  // Connects this parameter frontend to the corresponding backend.
  void connect(ParameterBackend<Handle<S>>* backend) {
    backend_ = backend;
  }

  const char* key() {
    return backend_ == nullptr ? nullptr : backend_->key();
  }

 private:
  // Used to return an Unexpected when the handle is not specified
  const Expected<Handle<S>> unspecified_handle_ = Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
  Expected<Handle<S>> value_ = Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
  ParameterBackend<Handle<S>>* backend_ = nullptr;
};

// -------------------------------------------------------------------------------------------------

template <typename T>
void ParameterBackend<T>::writeToFrontend() {
  if (frontend_ && value_) {
    frontend_->setWithoutPropagate(*value_);
  }
}

template <typename T>
Expected<void> ParameterBackend<T>::parse(const YAML::Node& node, const std::string& prefix) {
  return ParameterParser<T>::Parse(context(), uid(), key(), node, prefix)
        .map([this] (const T& value) { return set(value); })
        .and_then([this] { writeToFrontend(); });
}

template <typename T>
void ParameterBackend<Handle<T>>::writeToFrontend() {
  if (frontend_ && value_) {
    frontend_->setWithoutPropagate(*value_);
  }
}

template <typename T>
Expected<void> ParameterBackend<Handle<T>>::parse(const YAML::Node& node,
                                                  const std::string& prefix) {
  return ParameterParser<Handle<T>>::Parse(context(), uid(), key(), node, prefix)
        .map([this] (const Handle<T>& value) { return set(value); })
        .and_then([this] { writeToFrontend(); });
}

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CORE_PARAMETER_HPP_
