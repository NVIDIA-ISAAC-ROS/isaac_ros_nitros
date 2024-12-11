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

#ifndef NVIDIA_GXF_GRAPH_ARG_HPP_
#define NVIDIA_GXF_GRAPH_ARG_HPP_

#include <any>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "common/type_name.hpp"
#include "common/type_utils.hpp"
#include "gxf/core/parameter_parser.hpp"
#include "gxf/core/parameter_parser_std.hpp"
#include "gxf/core/parameter_registrar.hpp"
#include "gxf/core/parameter_wrapper.hpp"

namespace nvidia {

namespace gxf {

class ProxyComponent;

/**
 * @brief Struct to hold type info of an Arg
 *
 */
typedef struct ArgInfo {
  gxf_parameter_type_t type;
  std::string type_name;
  int32_t rank = 0;
  std::array<int32_t, ParameterInfo<int32_t>::kMaxRank> shape = {0};
} ArgInfo;

/**
 * @brief ArgOverride for scalar parameter type
 *
 * @tparam T A valid argument type
 */
template <typename T>
struct ArgOverride {
  static Expected<void> apply(ArgInfo& info) {
    info.type = ParameterTypeTrait<T>::type;
    info.type_name.assign(ParameterTypeTrait<T>::type_name);
    info.rank = 0;
    if (info.type == GXF_PARAMETER_TYPE_CUSTOM) { return Unexpected{GXF_ARGUMENT_INVALID}; }
    return Success;
  }

  static Expected<YAML::Node> wrap(const T& value) { return YAML::Node(value); }
};

/**
 * @brief Specialized ArgOverride for args of type handle<T>
 *
 * @tparam T A valid argument type
 */
template <typename T>
struct ArgOverride<Handle<T>> {
  static Expected<void> apply(ArgInfo& info) {
    info.type = GXF_PARAMETER_TYPE_HANDLE;
    info.type_name.assign(TypenameAsString<T>());
    info.rank = 0;
    return Success;
  }

  static Expected<YAML::Node> wrap(const Handle<T>& value) {
    std::string c_name = value.name();
    const char* entity_name;
    gxf_uid_t eid = kNullUid;
    auto result = GxfComponentEntity(value.context(), value.cid(), &eid);
    if (result != GXF_SUCCESS) {
      GXF_LOG_ERROR("Unable to find the entity for %s", c_name.c_str());
      return Unexpected{result};
    }
    result = GxfEntityGetName(value.context(), eid, &entity_name);
    if (result != GXF_SUCCESS) {
      GXF_LOG_ERROR("Unable to get the entity name");
      return Unexpected{result};
    }

    std::string full_name = std::string(entity_name) + "/" + c_name;
    return YAML::Node(full_name);
  }
};

/**
 * @brief Specialized ArgOverride for args of type std::vector<T>
 *
 * @tparam T A valid argument type
 */
template <typename T>
struct ArgOverride<std::vector<T>> {
  static Expected<void> apply(ArgInfo& info) {
    // Get the element info
    ArgInfo element_info;
    const auto result = ArgOverride<T>::apply(element_info);
    if (!result) { return ForwardError(result); }

    info.type = element_info.type;
    info.type_name = element_info.type_name;

    // Fetch the shape of <T> and update it to the current ArgInfo
    for (int32_t i = 0; i < element_info.rank; ++i) { info.shape[i] = element_info.shape[i]; }

    // A vector increases the rank by 1 and adds shape [-1] to <T>.
    info.shape[element_info.rank] = -1;
    info.rank = element_info.rank + 1;

    return Success;
  }

  static Expected<YAML::Node> wrap(const std::vector<T>& value) {
    YAML::Node node(YAML::NodeType::Sequence);
    for (const auto& val : value) {
      auto maybe = ArgOverride<T>::wrap(val);
      if (!maybe) { return Unexpected{maybe.error()}; }
      node.push_back(maybe.value());
    }
    return node;
  }
};

/**
 * @brief Specialized ArgOverride for parameters of type std::array<T,N>
 *
 * @tparam T A valid argument type
 * @tparam N length of the array
 */
template <typename T, std::size_t N>
struct ArgOverride<std::array<T, N>> {
  static Expected<void> apply(ArgInfo& info) {
    // Get the element info
    ArgInfo element_info;
    const auto result = ArgOverride<T>::apply(element_info);
    if (!result) { return ForwardError(result); }

    info.type = element_info.type;
    info.type_name = element_info.type_name;

    // Fetch the shape of <T> and update it to the current ArgInfo
    for (int32_t i = 0; i < element_info.rank; ++i) { info.shape[i] = element_info.shape[i]; }

    // An array increases the rank by 1 and adds shape [N] to <T>.
    info.shape[element_info.rank] = N;
    info.rank = element_info.rank + 1;

    return Success;
  }

  static Expected<YAML::Node> wrap(const std::array<T, N>& value) {
    YAML::Node node(YAML::NodeType::Sequence);
    for (const auto& val : value) {
      auto maybe = ArgOverride<T>::wrap(val);
      if (!maybe) { return Unexpected{maybe.error()}; }
      node.push_back(maybe.value());
    }
    return node;
  }
};

/**
 * @brief Specialized ArgOverride for parameters of type ProxyComponent
 *
 */
template <>
struct ArgOverride<ProxyComponent> {
  static Expected<void> apply(ArgInfo& info) {
    info.type = GXF_PARAMETER_TYPE_HANDLE;
    info.rank = 0;
    return Success;
  }

  static Expected<YAML::Node> wrap(const ProxyComponent& value) { return YAML::Node(); }
};

/**
 * @brief Argument interface to enable configuring parameters in GXF Components from
 * the application layer. All parameter types from gxf_parameter_type_t enum is supported.
 *
 */
class Arg {
 public:
  ~Arg() = default;

  // Constructors
  explicit Arg(const std::string& key) : key_(key) {}

  template <typename T>
  Arg(const std::string& key, const T& value) {
    key_ = key;
    auto result = ArgOverride<T>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg [%s] of type [%s] with error [%s]", key.c_str(),
                    TypenameAsString<T>(), GxfResultStr(result.error()));
      return;
    }
    set_value<T>(value);
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key.c_str(), info_.type_name.c_str());
  }

  template <typename T>
  Arg(const std::string& key, const Handle<T>& value) {
    key_ = key;
    auto result = ArgOverride<Handle<T>>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg type %s ", key.c_str());
      return;
    }
    set_value<Handle<T>>(value);
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key.c_str(), info_.type_name.c_str());
    if (!value.is_null()) {
      handle_cid_ = value->cid();
      // handle_tid_ = value->tid();
      GxfComponentTypeId(value.context(), info_.type_name.c_str(), &handle_tid_);
    }
  }

  // Special case for string literals typed arguments
  Arg(const std::string& key, const char* value) {
    key_ = key;
    auto result = ArgOverride<std::string>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg [%s] of type [%s] with error [%s]", key.c_str(),
                    TypenameAsString<std::string>(), GxfResultStr(result.error()));
      return;
    }
    set_value<std::string>(value);
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key.c_str(), info_.type_name.c_str());
  }

  Arg(const std::string& key, const ProxyComponent& value) {
    key_ = key;
    info_.type = GXF_PARAMETER_TYPE_HANDLE;
    // info_.type_name = value.type_name();
    handle_cid_ = kUnspecifiedUid;
    info_.rank = 0;
    set_value<ProxyComponent>(value);
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key.c_str(), info_.type_name.c_str());
  }

  template <typename T, typename = std::enable_if_t<!std::is_lvalue_reference<T>::value>>
  Arg(const std::string& key, T&& value) {
    using DeducedT = std::remove_reference_t<std::remove_cv_t<T>>;
    key_ = key;
    auto result = ArgOverride<DeducedT>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg type %s ", key.c_str());
      return;
    }
    set_value<T>(std::forward<T>(value));
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key.c_str(), info_.type_name.c_str());
  }

  template <typename T, typename = std::enable_if_t<!std::is_lvalue_reference<T>::value>>
  Arg(const std::string& key, Handle<T>&& value) {
    using DeducedT = std::remove_reference_t<std::remove_cv_t<Handle<T>>>;
    key_ = key;
    auto result = ArgOverride<DeducedT>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg type %s ", key.c_str());
      return;
    }
    if (!value.is_null()) {
      handle_cid_ = value->cid();
      // handle_tid_ = value->tid();
      GxfComponentTypeId(value.context(), info_.type_name.c_str(), &handle_tid_);
    }
    set_value<Handle<T>>(std::forward<Handle<T>>(value));
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key.c_str(), info_.type_name.c_str());
  }

  Arg(const std::string& key, ProxyComponent&& value) {
    key_ = key;
    info_.type = GXF_PARAMETER_TYPE_HANDLE;
    // info_.type_name = value.type_name();
    handle_cid_ = kUnspecifiedUid;
    info_.rank = 0;
    set_value<ProxyComponent>(std::forward<ProxyComponent>(value));
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key.c_str(), info_.type_name.c_str());
  }

  template <typename T, typename = std::enable_if_t<!std::is_same_v<Arg, std::decay_t<T>>>>
  Arg& operator=(const T& value) {
    auto result = ArgOverride<T>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg type %s ", key_.c_str());
      return *this;
    }
    set_value<T>(value);
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key_.c_str(), info_.type_name.c_str());
    return *this;
  }

  template <typename T, typename = std::enable_if_t<!std::is_same_v<Arg, std::decay_t<T>>>>
  Arg& operator=(const Handle<T>& value) {
    auto result = ArgOverride<Handle<T>>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg type %s ", key_.c_str());
      return *this;
    }
    if (!value.is_null()) {
      handle_cid_ = value->cid();
      // handle_tid_ = value->tid();
      GxfComponentTypeId(value.context(), info_.type_name.c_str(), &handle_tid_);
    }
    set_value<Handle<T>>(value);
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key_.c_str(), info_.type_name.c_str());
    return *this;
  }

  Arg& operator=(ProxyComponent& value) {
    info_.type = GXF_PARAMETER_TYPE_HANDLE;
    // info_.type_name = value.type_name();
    handle_cid_ = kUnspecifiedUid;
    info_.rank = 0;
    set_value<ProxyComponent>(value);
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key_.c_str(), info_.type_name.c_str());
    return *this;
  }

  template <typename T, typename = std::enable_if_t<!std::is_same_v<Arg, std::decay_t<T>> &&
                                                    !std::is_lvalue_reference<T>::value>>
  Arg&& operator=(T&& value) {
    using DeducedT = std::remove_reference_t<std::remove_cv_t<T>>;
    auto result = ArgOverride<DeducedT>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg type %s ", key_.c_str());
      return std::move(*this);
    }
    set_value<T>(std::forward<T>(value));
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key_.c_str(), info_.type_name.c_str());
    return std::move(*this);
  }

  template <typename T, typename = std::enable_if_t<!std::is_same_v<Arg, std::decay_t<Handle<T>>> &&
                                                    !std::is_lvalue_reference<Handle<T>>::value>>
  Arg&& operator=(Handle<T>&& value) {
    using DeducedT = std::remove_reference_t<std::remove_cv_t<Handle<T>>>;
    auto result = ArgOverride<DeducedT>::apply(info_);
    if (!result) {
      GXF_LOG_ERROR("Invalid Arg type %s ", key_.c_str());
      return std::move(*this);
    }
    if (!value.is_null()) {
      handle_cid_ = value->cid();
      // handle_tid_ = value->tid();
      GxfComponentTypeId(value.context(), info_.type_name.c_str(), &handle_tid_);
    }
    set_value<Handle<T>>(std::forward<Handle<T>>(value));
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key_.c_str(), info_.type_name.c_str());
    return std::move(*this);
  }

  Arg&& operator=(ProxyComponent&& value) {
    info_.type = GXF_PARAMETER_TYPE_HANDLE;
    // info_.type_name = value.type_name();
    handle_cid_ = kUnspecifiedUid;
    info_.rank = 0;
    set_value<ProxyComponent>(value);
    GXF_LOG_VERBOSE("Arg: [%s], Type : [%s] created", key_.c_str(), info_.type_name.c_str());
    return std::move(*this);
  }

  template <typename T, typename = std::enable_if_t<IsDefaultConstructible_v<T>>>
  T as() const {
    const T* value = std::any_cast<T>(&value_);
    if (value == nullptr) {
      GXF_LOG_ERROR("Arg [%s] cannot be read as [%s]", key_.c_str(), TypenameAsString<T>());
      return T();
    }
    return *value;
  }

  const gxf_uid_t handle_uid() const { return handle_cid_; }

  const gxf_tid_t handle_tid() const { return handle_tid_; }

  const char* key() const { return key_.c_str(); }

  const std::string arg_type_name() const { return info_.type_name; }

  const ArgInfo arg_info() const { return info_; }

  const YAML::Node yaml_node() const { return yaml_node_; }

  const int32_t rank() const { return info_.rank; }

  const std::array<int32_t, ParameterInfo<int32_t>::kMaxRank> shape() const { return info_.shape; }

  const gxf_parameter_type_t parameter_type() const { return info_.type; }

  bool has_value() const { return value_.has_value(); }

  std::any value() const { return value_; }

 private:
  std::any value_;
  std::string key_;
  ArgInfo info_;
  gxf_uid_t handle_cid_ = kNullUid;
  gxf_tid_t handle_tid_ = GxfTidNull();
  YAML::Node yaml_node_{};

  template <typename T>
  void set_value(const T& value) {
    value_ = value;
    auto maybe_yaml = ArgOverride<T>::wrap(value);
    if (!maybe_yaml) {
      GXF_LOG_ERROR("Arg [%s] failed to parse as a YAML node with error [%s]", key_.c_str(),
                    GxfResultStr(maybe_yaml.error()));
    } else {
      yaml_node_ = maybe_yaml.value();
    }
  }

  template <typename T>
  void set_value(T&& value) {
    auto maybe_yaml = ArgOverride<T>::wrap(value);
    if (!maybe_yaml) {
      GXF_LOG_ERROR("Arg [%s] failed to parse as a YAML node with error [%s]", key_.c_str(),
                    GxfResultStr(maybe_yaml.error()));
    } else {
      yaml_node_ = maybe_yaml.value();
    }
    value_ = std::move(value);
  }
};

}  // namespace gxf

}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_ARG_HPP_
