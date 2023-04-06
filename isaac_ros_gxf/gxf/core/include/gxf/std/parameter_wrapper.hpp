/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_PARAMETER_WRAPPER_HPP_
#define NVIDIA_GXF_STD_PARAMETER_WRAPPER_HPP_

#include <string>
#include <vector>

#include "gxf/core/gxf.h"
#include "gxf/core/handle.hpp"
#include "yaml-cpp/yaml.h"

namespace nvidia {
namespace gxf {

template <typename T, typename V = void>
struct ParameterWrapper;

template <typename T>
struct ParameterWrapper<T> {
  // Wrap the value to a YAML::Node instance
  static Expected<YAML::Node> Wrap(gxf_context_t context, const T& value) {
    return YAML::Node(value);
  }
};

template <typename T>
struct ParameterWrapper<Handle<T>> {
  static Expected<YAML::Node> Wrap(gxf_context_t context, const Handle<T>& value) {
    std::string c_name = value.name();
    const char *entity_name;
    gxf_uid_t eid = kNullUid;
    auto result = GxfComponentEntity(context, value.cid(), &eid);
    if (result != GXF_SUCCESS) {
      GXF_LOG_ERROR("Unable to find the entity for %s", c_name.c_str());
      return Unexpected{result};
    }
    result = GxfParameterGetStr(
      context, eid, kInternalNameParameterKey, &entity_name);
    if (result != GXF_SUCCESS) {
      GXF_LOG_ERROR("Unable to get the entity name");
      return Unexpected{result};
    }

    std::string full_name = std::string(entity_name) + "/" + c_name;
    return YAML::Node(full_name);
  }
};

template<typename T>
struct ParameterWrapper<std::vector<nvidia::gxf::Handle<T>>> {
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const std::vector<nvidia::gxf::Handle<T>>& value) {
      YAML::Node node(YAML::NodeType::Sequence);
      for (auto &h : value) {
        auto maybe = ParameterWrapper<Handle<T>>::Wrap(context, h);
        if (!maybe) {
          return Unexpected{maybe.error()};
        }
        node.push_back(maybe.value());
      }
    return node;
  }
};

template<typename T, size_t N>
struct ParameterWrapper<std::array<nvidia::gxf::Handle<T>, N>> {
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const std::array<nvidia::gxf::Handle<T>, N>& value) {
      YAML::Node node(YAML::NodeType::Sequence);
      for (auto &h : value) {
        auto maybe = ParameterWrapper<Handle<T>>::Wrap(context, h);
        if (!maybe) {
          return Unexpected{maybe.error()};
        }
        node.push_back(maybe.value());
      }
    return node;
  }
};

template<typename T, size_t N>
struct ParameterWrapper<nvidia::FixedVector<T, N>> {
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const nvidia::FixedVector<T, N>& value) {
      YAML::Node node(YAML::NodeType::Sequence);
      for (size_t i = 0; i < value.size(); i++) {
        auto v = value.at(i).value();
        auto maybe = ParameterWrapper<T>::Wrap(context, v);
        if (!maybe) {
          return Unexpected{maybe.error()};
        }
        node.push_back(maybe.value());
      }
    return node;
  }
};

template<size_t N>
struct ParameterWrapper<nvidia::FixedString<N>> {
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const nvidia::FixedString<N>& value) {
      return ParameterWrapper<std::string>::Wrap(context, std::string(value.c_str()));
  }
};

template<typename T, size_t N>
struct ParameterWrapper<nvidia::FixedVector<nvidia::gxf::Handle<T>, N>> {
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const nvidia::FixedVector<nvidia::gxf::Handle<T>, N>& value) {
      YAML::Node node(YAML::NodeType::Sequence);
      for (size_t i = 0; i < value.size(); i++) {
        auto h = value.at(i).value();
        auto maybe = ParameterWrapper<Handle<T>>::Wrap(context, h);
        if (!maybe) {
          return Unexpected{maybe.error()};
        }
        node.push_back(maybe.value());
      }
    return node;
  }
};

template<>
struct ParameterWrapper<nvidia::gxf::FilePath> {
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const nvidia::gxf::FilePath& value) {
    return ParameterWrapper<std::string>::Wrap(context, value);
  }
};

}  // namespace gxf
}  // namespace nvidia

#endif
