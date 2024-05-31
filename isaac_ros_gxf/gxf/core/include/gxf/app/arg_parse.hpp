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

#ifndef NVIDIA_GXF_GRAPH_ARG_PARSE_HPP_
#define NVIDIA_GXF_GRAPH_ARG_PARSE_HPP_

#include <string>
#include <vector>

#include "gxf/app/arg.hpp"
#include "gxf/app/proxy_component.hpp"
#include "gxf/core/component.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief Function returns a list of elements filtered from an input parameter pack of
 * arguments
 *
 * @tparam T Type of parameter to be filtered from the parameter pack
 * @return std::vector<T>
 */
template <typename T, typename First, typename... Rest>
std::vector<T> parseArgsOfType(const First& first, const Rest&... rest) {
  std::vector<T> arg_list;
  if constexpr (std::is_same_v<T, First>) { arg_list.push_back(first); }
  if constexpr (sizeof...(rest) > 0) {
    auto rest_list = parseArgsOfType<T>(rest...);
    arg_list.insert(arg_list.end(), rest_list.begin(), rest_list.end());
  }
  return arg_list;
}


// Returns a list of proxyComponent args from a list of args
// The proxy component args are removed from the input arg list
std::vector<Arg> filterProxyComponents(std::vector<Arg>& args);

// Sets a parameter value for a component using the info stored in Arg type
Expected<void> applyArg(Handle<Component> component, const Arg& arg);

// Returns an arg from a list of args
Expected<Arg> findArg(const std::vector<Arg>& args, const std::string& key,
  const gxf_parameter_type_t type);

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_ARG_PARSE_HPP_
