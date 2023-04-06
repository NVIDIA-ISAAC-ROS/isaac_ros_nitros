/*
Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
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
