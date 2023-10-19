// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace nvidia {
namespace gxf {
namespace optimizer {

using ComponentKey = std::string;

struct ComponentInfo {
  std::string component_type_name;
  std::string component_name;
  std::string entity_name;

  bool operator==(const ComponentInfo& other) const {
    return component_type_name == other.component_type_name &&
           component_name == other.component_name &&
           entity_name == other.entity_name;
  }
};

struct GraphIOGroupSupportedDataTypesInfo {
  std::vector<ComponentInfo> ingress_infos;
  std::vector<ComponentInfo> egress_infos;
  std::vector<std::map<ComponentKey, std::string>> supported_data_types;
};
using GraphIOGroupSupportedDataTypesInfoList = std::vector<GraphIOGroupSupportedDataTypesInfo>;

struct GraphIOGroupDataTypeConfigurations {
  std::vector<ComponentInfo> ingress_infos;
  std::vector<ComponentInfo> egress_infos;
  std::map<ComponentKey, std::string> data_type_configurations;
};
using GraphIOGroupDataTypeConfigurationsList = std::vector<GraphIOGroupDataTypeConfigurations>;

inline ComponentKey GenerateComponentKey(const ComponentInfo& comp_info) {
  return comp_info.entity_name + "/" + comp_info.component_name;
}

inline std::vector<std::string> GetSupportedDataTypes(
    const GraphIOGroupSupportedDataTypesInfo& supported_data_types_info,
    const ComponentInfo& comp_info) {
  std::vector<std::string> supported_data_types;
  for (auto& type_map : supported_data_types_info.supported_data_types) {
    // Add the data type but avoid the duplicates
    const std::string& data_type = type_map.at(GenerateComponentKey(comp_info));
    if (std::find(supported_data_types.begin(), supported_data_types.end(), data_type) ==
        supported_data_types.end()) {
      supported_data_types.push_back(data_type);
    }
  }
  return supported_data_types;
}

}  // namespace optimizer
}  // namespace gxf
}  // namespace nvidia
