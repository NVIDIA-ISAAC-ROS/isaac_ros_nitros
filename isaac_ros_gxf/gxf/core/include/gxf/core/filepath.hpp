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
#ifndef NVIDIA_GXF_CORE_FILEPATH_HPP_
#define NVIDIA_GXF_CORE_FILEPATH_HPP_

#include <string>
#include "yaml-cpp/yaml.h"

namespace nvidia {
namespace gxf {


// Specialization of ParameterParser for a file parameter type to identify parameters that are file
// system paths
struct FilePath : public std::string {
  FilePath() : std::string() {}

  FilePath(const char* s) : std::string(s) {}

  FilePath(const std::string& str) : std::string(str) {}
};


}  // namespace gxf
}  // namespace nvidia

template<>
struct YAML::convert<nvidia::gxf::FilePath> {
    static Node encode(const nvidia::gxf::FilePath& data) {
      Node node = YAML::Load(data);
      return node;
    }

    static bool decode(const Node& node, nvidia::gxf::FilePath& data) {
      if (!node.IsScalar()) {
        return false;
      }
      data = node.as<std::string>();
      return true;
    }
};



#endif  // NVIDIA_GXF_CORE_FILEPATH_HPP_
