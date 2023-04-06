/*
Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "extensions/gxf_optimizer/common/type.hpp"
#include "extensions/gxf_optimizer/exporter/graph_types.hpp"
#include "gxf/core/expected.hpp"

namespace nvidia {
namespace gxf {
namespace optimizer {

class ConfigurationGraph;

class Optimizer {
  class OptimizerImpl;
  std::unique_ptr<OptimizerImpl> pimpl;

 public:
  // Constructor & destructor
  Optimizer();
  ~Optimizer();

  // Load a graph from the given YAML file.
  Expected<void> loadGraph(const std::string& graph_yaml_file_path);

  // Load optimization rules from the given YAML file.
  Expected<void> loadRules(const std::string& rules_yaml_file_path);

  // Load extension specs from the YAML file.
  Expected<void> loadExtensionSpecs(const std::string& specs_yaml_file_path);

  // Load preset extension specs.
  Expected<void> loadPresetExtensionSpecs(const std::string& preset_key);

  // Export the originally loaded graph as YAML string.
  // It returns a YAML string that corresponds to the application graph,
  // otherwise one of the GXF error codes.
  Expected<std::string> exportLoadedGraph() const;

  // Export the graph candidate at the given index as a YAML string.
  // It returns a YAML string that corresponds to the graph of the specified candidate,
  // otherwise one of the GXF error codes.
  Expected<std::string> exportGraph(const size_t index) const;

  // Get the graph based on the given ingress-egres group data type assignments.
  // If a prefix string is given, then unnamed entities will be given random names and all
  // entity names will be prepended with the given prefix.
  // It returns a string that corresponds to the graph's YAML of the specified candidate,
  // otherwise one of the GXF error codes.
  Expected<std::string> exportGraph(
      const GraphIOGroupDataTypeConfigurationsList& configs,
      const std::string& prefix = std::string());

  // Get the graph based on the given ingress-egres group data type assignments.
  // If a prefix string is given, then unnamed entities will be given random names and all
  // entity names will be prepended with the given prefix.
  // The resulting graph(s) (including all subgraphs) are saved to the specified directory.
  // The top level graph YAML file is named <export_base_filename>.yaml. Its first-level
  // subgraphs are named <export_base_filename>_subgraph<index>.yaml and second-level
  // subgraphs being <export_base_filename>_subgraph<index>_subgraph<2nd_level_index>.yaml
  // and so on and so forth if any.
  Expected<void> exportGraphToFiles(
      const GraphIOGroupDataTypeConfigurationsList& configs,
      const std::string& export_directory,
      const std::string& export_base_filename,
      const std::string& prefix = std::string());

  // Export ingress-egress groups and their supported data types
  Expected<GraphIOGroupSupportedDataTypesInfoList> exportGraphIOGroupSupportedDataTypesInfoList();

  // Run graph optimization.
  // It returns SUCCESS if the operation was finished without an error,
  // otherwise one of the GXF error codes.
  Expected<void> optimize(const gxf_cog_factor_t top_factor);

  // Get a copy of the loaded configuration graph.
  // It returns a copy of the loaded configruation graph,
  // otherwise one of the GXF error codes.
  Expected<ConfigurationGraph> getLoadedGraph() const;

  // Get a graph candidate at the given index.
  // It returns a copy of the graph candidate at the given index or
  // GXF_ARGUMENT_OUT_OF_RANGE if the index is out of range.
  Expected<ConfigurationGraph> getGraphCandidate(const size_t index) const;

  // Get the number of available graph candidates.
  size_t getGraphCandidateCount() const;
};

}  // namespace optimizer
}  // namespace gxf
}  // namespace nvidia
