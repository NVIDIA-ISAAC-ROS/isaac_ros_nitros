// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS__NITROS_NODE_HPP_
#define ISAAC_ROS_NITROS__NITROS_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "extensions/gxf_optimizer/core/optimizer.hpp"
#include "extensions/gxf_optimizer/exporter/graph_types.hpp"

#include "isaac_ros_nitros/nitros_context.hpp"
#include "isaac_ros_nitros/nitros_publisher_subscriber_group.hpp"

#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

using gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;
using gxf::optimizer::GraphIOGroupDataTypeConfigurations;
using gxf::optimizer::ComponentInfo;
using gxf::optimizer::ComponentKey;
using NitrosPubSubGroupPointerList = std::vector<std::shared_ptr<NitrosPublisherSubscriberGroup>>;

// A hardware-accelerated ROS node base class
class NitrosNode : public rclcpp::Node
{
public:
  ~NitrosNode();

  NitrosNode(const NitrosNode & node) = delete;

  NitrosNode & operator=(const NitrosNode & node) = delete;

protected:
  // Constructor with user-defined IO groups
  NitrosNode(
    const rclcpp::NodeOptions & options,
    const std::string & app_yaml_filename,
    const GraphIOGroupSupportedDataTypesInfoList & gxf_io_group_info_list,
    const NitrosPublisherSubscriberConfigMap & config_map,
    const std::vector<std::string> & extension_spec_filenames,
    const std::vector<std::string> & generator_rule_filenames,
    const std::vector<std::pair<std::string, std::string>> & extensions,
    const std::string & package_name);

  // Constructor with user-defined IO groups (with preset specs)
  NitrosNode(
    const rclcpp::NodeOptions & options,
    const std::string & app_yaml_filename,
    const GraphIOGroupSupportedDataTypesInfoList & gxf_io_group_info_list,
    const NitrosPublisherSubscriberConfigMap & config_map,
    const std::vector<std::string> & extension_spec_preset_names,
    const std::vector<std::string> & extension_spec_filenames,
    const std::vector<std::string> & generator_rule_filenames,
    const std::vector<std::pair<std::string, std::string>> & extensions,
    const std::string & package_name);

  // Constructor without user-defined IO groups
  NitrosNode(
    const rclcpp::NodeOptions & options,
    const std::string & app_yaml_filename,
    const NitrosPublisherSubscriberConfigMap & config_map,
    const std::vector<std::string> & extension_spec_filenames,
    const std::vector<std::string> & generator_rule_filenames,
    const std::vector<std::pair<std::string, std::string>> & extensions,
    const std::string & package_name);

  // Constructor without user-defined IO groups (with preset specs)
  NitrosNode(
    const rclcpp::NodeOptions & options,
    const std::string & app_yaml_filename,
    const NitrosPublisherSubscriberConfigMap & config_map,
    const std::vector<std::string> & extension_spec_preset_names,
    const std::vector<std::string> & extension_spec_filenames,
    const std::vector<std::string> & generator_rule_filenames,
    const std::vector<std::pair<std::string, std::string>> & extensions,
    const std::string & package_name);

  // Complete constructor
  NitrosNode(
    const rclcpp::NodeOptions & options,
    const std::string & app_yaml_filename,
    const GraphIOGroupSupportedDataTypesInfoList & gxf_io_group_info_list,
    const bool use_custom_io_group_info_list,
    const bool use_raw_graph_no_optimizer,
    const NitrosPublisherSubscriberConfigMap & config_map,
    const std::vector<std::string> & extension_spec_preset_names,
    const std::vector<std::string> & extension_spec_filenames,
    const std::vector<std::string> & generator_rule_filenames,
    const std::vector<std::pair<std::string, std::string>> & extensions,
    const std::string & package_name);

  // Getter for the underlying Nitros context object
  NitrosContext & getNitrosContext();

  // Initialize and start the NitrosNode
  void startNitrosNode();

  // The callback to be implemented by users for any required initialization before graph
  // is loaded (but after negotiation results are available)
  virtual void preLoadGraphCallback() {}

  // The callback to be implemented by users for any required initialization after graph is loaded
  virtual void postLoadGraphCallback() {}

  // Override a parameter value in the graph to be loaded
  void preLoadGraphSetParameter(
    const std::string & entity_name,
    const std::string & component_name,
    const std::string & parameter_name,
    const std::string & value
  );

  // Find the corresponding Nitros publisher of the given component
  std::shared_ptr<NitrosPublisher> findNitrosPublisher(
    const gxf::optimizer::ComponentInfo & comp_info);

  // Find the corresponding Nitros subscriber of the given component
  std::shared_ptr<NitrosSubscriber> findNitrosSubscriber(
    const gxf::optimizer::ComponentInfo & comp_info);

  // Get the negotiated data format for the given component
  std::string getNegotiatedDataFormat(const ComponentInfo comp_info) const;

  // Get the negotiated data format or the compatible format if negotiation failed
  // for the given component
  std::string getFinalDataFormat(const ComponentInfo comp_info) const;

  // Setter for use_custom_io_group_info_list_ that determines whether to use a user-defined
  // custom IO group list (in contrast to letting the optimizer create the IO group list)
  void setUseCustomIOGroupInfoList(const bool use_custom);

  // Add an extension spec file to be loaded into the graph optimizer
  void addExtensionSpecFilename(std::string package_relative_filepath);

  // Add a generator rule file to be loaded into the graph optimizer
  void addGeneratorRuleFilename(std::string package_relative_filepath);

  // Set the graph YAML file to be loaded
  void setAppYamlFilename(std::string package_relative_filepath);

  // Set the frame id for a custom frame id publisher
  void setFrameIdSource(std::string source_frame_id_map_key, std::string frame_id);

  // Register a supported type
  template<typename T>
  void registerSupportedType()
  {
    nitros_type_manager_->registerSupportedType<T>();
  }

  // Nitros type manager
  std::shared_ptr<NitrosTypeManager> nitros_type_manager_;

  // A list of I/O port groups
  GraphIOGroupSupportedDataTypesInfoList gxf_io_group_info_list_;

  // Determine if a user-defined IO group info list should be used
  // (in contrast to creating the IO group list by using the optimizer)
  bool use_custom_io_group_info_list_ = false;

  // A map for specifying the configurations for all I/O ports
  NitrosPublisherSubscriberConfigMap config_map_;

  // Path to the shared directory under this (isaac_ros_nitros) package
  std::string nitros_package_share_directory_;

  // Path to the shared directory under the sub-class package
  std::string package_share_directory_;

private:
  // The function to be called after negotiation timeout
  void postNegotiationCallback();

  // The function for checking the status of the underlying graph
  void gxfHeartbeatCallback();

  // A randomly generated namespace that's unique for the current node
  std::string graph_namespace_;

  // An application graph to be loaded in the node's context
  std::string app_yaml_filename_;

  // A list of preset component spec to be loaded in the graph optimizer
  std::vector<std::string> extension_spec_preset_names_;

  // A list of component specs to be loaded in the graph optimizer
  std::vector<std::string> extension_spec_filenames_;

  // A list of rules to be loaded in the graph optimizer
  std::vector<std::string> generator_rule_filenames_;

  // A list of component extension so files to be loaded in the node's context
  std::vector<std::pair<std::string, std::string>> extensions_;

  // A list of pub/sub groups in which the data formats of a group of publishers and subscribers
  // are dependent on each other
  NitrosPubSubGroupPointerList nitros_pub_sub_groups_;

  // The graph optimizer
  nvidia::gxf::optimizer::Optimizer optimizer_;

  // The node's package name
  std::string package_name_;

  // Negotiation wait timer
  rclcpp::TimerBase::SharedPtr negotiation_timer_;

  // The GXF graph heartbeat timer
  rclcpp::TimerBase::SharedPtr gxf_heartbeat_timer_;

  // The eid for checking the graph status
  gxf_uid_t heartbeat_eid_ = -1;

  // Map for frame_id passthrough
  std::shared_ptr<std::map<ComponentKey, std::string>> frame_id_map_ptr_;

  // Configurations for a Nitros diagnostics
  std::shared_ptr<NitrosDiagnosticsConfig> diagnostics_config_;

  // When enabled namespacing and the graph optimizer are ignored
  // In such a case, graph(s) will be loaded intact
  bool use_raw_graph_no_optimizer_ = false;

  // Nitros context that has a shared underlying context across all NitrosNodes in the same process
  std::shared_ptr<NitrosContext> nitros_context_ptr_;

  // Wait time in seconds before concluding negotiation results
  int64_t type_negotiation_duration_s_ = 1;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_NODE_HPP_
