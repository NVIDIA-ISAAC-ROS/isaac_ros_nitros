/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
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
                      const uint32_t num_overrides);

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

  ParameterStorage* parameter_storage_ = nullptr;

  std::string root_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_YAML_FILE_LOADER_HPP_
