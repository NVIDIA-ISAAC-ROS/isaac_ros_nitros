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

#ifndef NVIDIA_GXF_PROXY_COMPONENT_HPP_
#define NVIDIA_GXF_PROXY_COMPONENT_HPP_

#include <map>
#include <string>
#include <vector>

#include "common/assert.hpp"
#include "gxf/core/component.hpp"
#include "gxf/core/expected_macro.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

/**
 * @brief A type to aid in lazy component creation by storing component type
 * and arg info
 *
 */
class ProxyComponent {
 public:
  ProxyComponent() {}
  ProxyComponent(const std::string& type_name, const std::string& component_name,
                 const std::vector<Arg>& args)
      : typename_(type_name), component_name_(component_name), args_(args) {}

  std::string type_name() const { return typename_; }

  std::string name() const { return component_name_; }

  std::vector<Arg> args() const { return args_; }

 private:
  std::string typename_;
  std::string component_name_;
  std::vector<Arg> args_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_PROXY_COMPONENT_HPP_
