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
#ifndef NVIDIA_GXF_GXF_STD_RESOURCE_REGISTRAR_HPP_
#define NVIDIA_GXF_GXF_STD_RESOURCE_REGISTRAR_HPP_

#include <map>
#include <memory>
#include <string>

#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

/// @brief Maintains a map of {component_tid : ComponentInfo(Resources)}
class ResourceRegistrar {
 public:
  ResourceRegistrar(gxf_context_t context) : context_(context) {}
  // Struct to hold information about a single Resource
  struct ComponentResourceInfo {
    gxf_tid_t handle_tid = GxfTidNull();
    std::string description;
  };

  // Struct to hold information about all the Resources in a component
  struct ComponentInfo {
    std::string type_name;
    std::map<gxf_tid_t, ComponentResourceInfo> resources;
  };

  template <typename T>
  Expected<void> registerComponentResource(gxf_tid_t comp_tid,
                                           const char* comp_type_name,
                                           const char* resource_description) {
    auto it = component_resources_.find(comp_tid);
    if (it == component_resources_.end()) {
      component_resources_[comp_tid] = std::make_unique<ComponentInfo>();
      it = component_resources_.find(comp_tid);
    }
    gxf_tid_t resource_tid;
    gxf_result_t result = GxfComponentTypeId(context_, TypenameAsString<T>(), &resource_tid);
    if (result != GXF_SUCCESS) {
      GXF_LOG_ERROR("ResourceRegistrar: Runtime cannot "
                    "find tid of resource [type name: %s]", TypenameAsString<T>());
      return Unexpected { result };
    }
    const auto jt = it->second->resources.find(resource_tid);
    if (jt != it->second->resources.end()) {
      return Unexpected{GXF_PARAMETER_ALREADY_REGISTERED};
    }
    ComponentResourceInfo info;
    info.handle_tid = resource_tid;
    info.description = std::string(resource_description);
    it->second->type_name = comp_type_name;
    it->second->resources[info.handle_tid] = info;

    return Success;
  }

 private:
  gxf_context_t context_;
  std::map<gxf_tid_t, std::unique_ptr<ComponentInfo>> component_resources_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GXF_STD_RESOURCE_REGISTRAR_HPP_

