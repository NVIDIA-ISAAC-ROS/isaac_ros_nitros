/*
 * SPDX-FileCopyrightText: Copyright (c) 2020 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_YAML_FILE_LOADER_HPP_
#define NVIDIA_GXF_STD_YAML_FILE_LOADER_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "common/fixed_vector.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"
#include "yaml-cpp/yaml.h"

namespace YAML { class Node; }

namespace nvidia {
namespace gxf {

class ParameterStorage;

// Loads a GXF file and creates entities
// Components will be initialized in the order they are specified in the YAML file and
// deinitialized in reverse-initialization order
class YamlFileLoader {
 public:
  // Sets the mandatory parameter storage where parameters loaded from YAML are stored.
  void setParameterStorage(ParameterStorage* parameter_storage) {
    parameter_storage_ = parameter_storage;
  }

  void setFileRoot(const std::string& root) { root_ = root; }

  // Load GXF Graph from file.
  Expected<void> loadFromFile(gxf_context_t context, const std::string& filename,
                              const std::string& entity_prefix,
                              const char* parameters_override_string[],
                              const uint32_t num_overrides);

  // Load GXF Graph from string.
  Expected<void> loadFromString(gxf_context_t context, const std::string& text,
                                const std::string& entity_prefix,
                                const char* parameters_override_string[],
                                const uint32_t num_overrides);

  // Load GXF Graph from parsed YAML graph.
  Expected<void> load(gxf_context_t context, const FixedVectorBase<YAML::Node>& nodes,
                      std::string entity_prefix, gxf_uid_t parent_eid,
                      const char* parameters_override_string[],
                      const uint32_t num_overrides,
                      const YAML::Node& prerequisites);

  // Save GXF Graph to YAML file.
  Expected<void> saveToFile(gxf_context_t context, const std::string& filename);

 private:
  // Finds existing entity or creates a new one
  Expected<gxf_uid_t> findOrCreateEntity(
      gxf_context_t context, const Expected<std::string>& entity_name);

  // Creates a new component in an entity by typename
  Expected<gxf_uid_t> addComponent(gxf_context_t context, gxf_uid_t eid, const char* type);

  // Finds a component in an entity by name
  Expected<gxf_uid_t> findComponent(gxf_context_t context, gxf_uid_t eid, const char* name);

  // Sets the parameters of a component
  Expected<void> setParameters(gxf_context_t context, gxf_uid_t handle,
                               const std::string& prefix, const YAML::Node& parameters);

  // Add a component to an entity's interface mapping
  Expected<void> addComponentToInterface(gxf_context_t context, gxf_uid_t eid,
                                         const std::string& entity_prefix,
                                         const std::string& interface_name,
                                         const std::string& tag);
  ParameterStorage* parameter_storage_ = nullptr;

  std::string root_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_YAML_FILE_LOADER_HPP_
