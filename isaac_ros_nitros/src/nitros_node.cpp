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

#include <unistd.h>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "extensions/gxf_optimizer/core/optimizer.hpp"
#include "gxf/core/gxf.h"

#include "isaac_ros_nitros/nitros_node.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

constexpr char kGraphExportDirectory[] = "/tmp/isaac_ros_nitros/graphs";

constexpr char kGxfVaultComponentTypeName[] = "nvidia::gxf::Vault";
constexpr char kGxfMessageRelayComponentTypeName[] =
  "nvidia::isaac_ros::MessageRelay";

const std::vector<std::pair<std::string, std::string>> BUILTIN_EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/std/libgxf_std.so"},
  {"isaac_ros_gxf", "gxf/lib/multimedia/libgxf_multimedia.so"},
  {"gxf_isaac_message_compositor", "gxf/lib/libgxf_isaac_message_compositor.so"}
};

const std::vector<std::string> BUILTIN_EXTENSION_SPEC_FILENAMES = {};

const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_nitros",
};

namespace
{
// Generate a random string of the given length
unsigned int seed = time(NULL) ^ getpid();
std::string GenerateRandomString(const size_t length)
{
  std::string output_string;
  char rand_char;
  for (size_t i = 0; i < length; i++) {
    rand_char = 'A' + rand_r(&seed) % 26;
    output_string.push_back(rand_char);
  }
  return output_string;
}
}  // namespace

NitrosNode::NitrosNode(
  const rclcpp::NodeOptions & options,
  const std::string & app_yaml_filename,
  const GraphIOGroupSupportedDataTypesInfoList & gxf_io_group_info_list,
  const NitrosPublisherSubscriberConfigMap & config_map,
  const std::vector<std::string> & extension_spec_filenames,
  const std::vector<std::string> & generator_rule_filenames,
  const std::vector<std::pair<std::string, std::string>> & extensions,
  const std::string & package_name)
: NitrosNode(
    options,
    app_yaml_filename,
    gxf_io_group_info_list,
    true,  // Use the given gxf_io_group_info_list
    true,
    config_map,
    {},
    extension_spec_filenames,
    generator_rule_filenames,
    extensions,
    package_name)
{
}

NitrosNode::NitrosNode(
  const rclcpp::NodeOptions & options,
  const std::string & app_yaml_filename,
  const GraphIOGroupSupportedDataTypesInfoList & gxf_io_group_info_list,
  const NitrosPublisherSubscriberConfigMap & config_map,
  const std::vector<std::string> & extension_spec_preset_names,
  const std::vector<std::string> & extension_spec_filenames,
  const std::vector<std::string> & generator_rule_filenames,
  const std::vector<std::pair<std::string, std::string>> & extensions,
  const std::string & package_name)
: NitrosNode(
    options,
    app_yaml_filename,
    gxf_io_group_info_list,
    true,  // Use the given gxf_io_group_info_list
    false,
    config_map,
    extension_spec_preset_names,
    extension_spec_filenames,
    generator_rule_filenames,
    extensions,
    package_name)
{
}


NitrosNode::NitrosNode(
  const rclcpp::NodeOptions & options,
  const std::string & app_yaml_filename,
  const NitrosPublisherSubscriberConfigMap & config_map,
  const std::vector<std::string> & extension_spec_filenames,
  const std::vector<std::string> & generator_rule_filenames,
  const std::vector<std::pair<std::string, std::string>> & extensions,
  const std::string & package_name)
: NitrosNode(
    options,
    app_yaml_filename,
    {},  // Use an empty GXF IO information (to be initialized by optimizer)
    false,
    false,
    config_map,
    {},
    extension_spec_filenames,
    generator_rule_filenames,
    extensions,
    package_name)
{
}

NitrosNode::NitrosNode(
  const rclcpp::NodeOptions & options,
  const std::string & app_yaml_filename,
  const NitrosPublisherSubscriberConfigMap & config_map,
  const std::vector<std::string> & extension_spec_preset_names,
  const std::vector<std::string> & extension_spec_filenames,
  const std::vector<std::string> & generator_rule_filenames,
  const std::vector<std::pair<std::string, std::string>> & extensions,
  const std::string & package_name)
: NitrosNode(
    options,
    app_yaml_filename,
    {},  // Use an empty GXF IO information (to be initialized by optimizer)
    false,
    false,
    config_map,
    extension_spec_preset_names,
    extension_spec_filenames,
    generator_rule_filenames,
    extensions,
    package_name)
{
}

NitrosNode::NitrosNode(
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
  const std::string & package_name)
: Node("NitrosNode", options),
  gxf_io_group_info_list_(gxf_io_group_info_list),
  use_custom_io_group_info_list_(use_custom_io_group_info_list),
  config_map_(config_map),
  app_yaml_filename_(app_yaml_filename),
  extension_spec_preset_names_(extension_spec_preset_names),
  extension_spec_filenames_(extension_spec_filenames),
  generator_rule_filenames_(generator_rule_filenames),
  extensions_(extensions),
  package_name_(package_name),
  use_raw_graph_no_optimizer_(use_raw_graph_no_optimizer),
  nitros_context_ptr_(std::make_shared<NitrosContext>())
{
  RCLCPP_INFO(get_logger(), "[NitrosNode] Initializing NitrosNode");

  // Set extension log level from an user-specified parameter
  nitros_context_ptr_->setExtensionLogSeverity(
    static_cast<gxf_severity_t>(declare_parameter<uint16_t>(
      "extension_log_level",
      gxf_severity_t::GXF_SEVERITY_WARNING)));

  // This line triggers the creation of a type adapter context that
  // loads a GXF graph containing common components accessible by
  // all NitrosNode subclasses created in the same process.
  GetTypeAdapterNitrosContext();

  nitros_context_ptr_->setNode(this);

  // Create an NITROS type manager for the node
  diagnostics_config_ = std::make_shared<NitrosDiagnosticsConfig>();

  // Setup ROS parameters for NITROS diagnostics
  diagnostics_config_->enable_node_time_diagnostics = declare_parameter<bool>(
    "enable_node_time_diagnostics", false);
  diagnostics_config_->enable_msg_time_diagnostics = declare_parameter<bool>(
    "enable_msg_time_diagnostics", false);
  diagnostics_config_->enable_increasing_msg_time_diagnostics = declare_parameter<bool>(
    "enable_increasing_msg_time_diagnostics", false);
  diagnostics_config_->enable_all_diagnostics = declare_parameter<bool>(
    "enable_all_diagnostics", false);
  if (diagnostics_config_->enable_all_diagnostics) {
    diagnostics_config_->enable_node_time_diagnostics = true;
    diagnostics_config_->enable_msg_time_diagnostics = true;
    diagnostics_config_->enable_increasing_msg_time_diagnostics = true;
  }

  const char * enable_global_diagnostics = std::getenv("ENABLE_GLOBAL_NITROS_DIAGNOSTICS");
  std::string global_env_value;

  if (enable_global_diagnostics != nullptr) {
    global_env_value = std::string(enable_global_diagnostics);
  } else {
    global_env_value = "";
  }

  if (global_env_value == "True" ||
    global_env_value == "TRUE" ||
    global_env_value == "true" ||
    global_env_value == "1")
  {
    diagnostics_config_->enable_all_topic_diagnostics = true;
  }

  if (
    diagnostics_config_->enable_all_topic_diagnostics ||
    diagnostics_config_->enable_node_time_diagnostics ||
    diagnostics_config_->enable_msg_time_diagnostics ||
    diagnostics_config_->enable_increasing_msg_time_diagnostics)
  {
    diagnostics_config_->enable_diagnostics = true;
    diagnostics_config_->diagnostics_publish_rate = declare_parameter<float>(
      "diagnostics_publish_rate",
      1.0);
    diagnostics_config_->filter_window_size =
      declare_parameter<int>("filter_window_size", 100);
    diagnostics_config_->jitter_tolerance_us =
      declare_parameter<int>("jitter_tolerance_us", 5000.0);
    // List of expected topic names
    std::vector<std::string> topics_name_list =
      declare_parameter<std::vector<std::string>>("topics_list", std::vector<std::string>());
    // List of fps for each expected topic
    std::vector<double> expected_fps_list =
      declare_parameter<std::vector<double>>("expected_fps_list", std::vector<double>());
    if (topics_name_list.size() !=
      expected_fps_list.size())
    {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] topics_name_list and expected_fps_list do not have the same size";
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
    // Populate the topic_name -> expected time difference
    for (size_t i = 0; i < topics_name_list.size(); i++) {
      diagnostics_config_->topic_name_expected_dt_map[topics_name_list[i]] = 1000000 /
        expected_fps_list[i];
    }
  }

  // Data type negotiation duration
  type_negotiation_duration_s_ =
    declare_parameter<int>("type_negotiation_duration_s", 1);
  RCLCPP_DEBUG(
    get_logger(),
    "[NitrosNode] Type negotiation duration (seconds) = %ld",
    type_negotiation_duration_s_);

  if (use_raw_graph_no_optimizer_) {
    graph_namespace_ = "";
    RCLCPP_INFO(get_logger(), "[NitrosNode] Enabled to use raw graph(s)");
  } else {
    // Genearate a random unique namespace for the underlying GXF graph
    graph_namespace_ = GenerateRandomString(10);
  }
  nitros_context_ptr_->setGraphNamespace(graph_namespace_);
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Namespace = %s", graph_namespace_.c_str());

  package_share_directory_ =
    ament_index_cpp::get_package_share_directory(package_name_);
  RCLCPP_DEBUG(
    get_logger(),
    "[NitrosNode] Package's directory: %s", package_share_directory_.c_str());

  nitros_package_share_directory_ =
    ament_index_cpp::get_package_share_directory("isaac_ros_nitros");
  RCLCPP_DEBUG(
    get_logger(),
    "[NitrosNode] Nitros's directory: %s", nitros_package_share_directory_.c_str());

  // Create a frame_id map for the node
  frame_id_map_ptr_ = std::make_shared<std::map<ComponentKey, std::string>>();

  // Create an NITROS type manager for the node
  nitros_type_manager_ = std::make_shared<NitrosTypeManager>(this);
}

NitrosContext & NitrosNode::getNitrosContext()
{
  return *nitros_context_ptr_;
}

void NitrosNode::startNitrosNode()
{
  RCLCPP_INFO(get_logger(), "[NitrosNode] Starting NitrosNode");

  // Process user-custom per-topic qos and format parameters
  for (auto & config_pair : config_map_) {
    const std::string topic_name = config_pair.second.topic_name;

    // QoS setting
    const std::string topic_qos_parameter_name = topic_name + "_qos_depth";
    const int topic_qos_depth = declare_parameter<int>(
      topic_qos_parameter_name, -1);
    if (topic_qos_depth > 0) {
      RCLCPP_INFO(
        get_logger(),
        "[NitrosNode] Overriding QoS depth for topic \"%s\" to %d",
        topic_name.c_str(), topic_qos_depth);
      config_pair.second.qos = rclcpp::QoS(topic_qos_depth);
    }

    // Data format setting
    const std::string topic_format_parameter_name = topic_name + "_nitros_format";
    const std::string topic_nitros_format = declare_parameter<std::string>(
      topic_format_parameter_name, "");
    if (!topic_nitros_format.empty()) {
      // Validate if the given format string is valid and supported
      if (!nitros_type_manager_->hasFormat(topic_nitros_format)) {
        std::stringstream error_msg;
        error_msg << "[NitrosNode] The given pinning data format \"" <<
          topic_nitros_format << "\" for topic \"" << topic_name << "\" is invalid";
        RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
        throw std::runtime_error(error_msg.str().c_str());
      }

      RCLCPP_INFO(
        get_logger(),
        "[NitrosNode] Pinning format for topic \"%s\" to \"%s\"",
        topic_name.c_str(), topic_nitros_format.c_str());
      config_pair.second.compatible_data_format = topic_nitros_format;
      config_pair.second.use_compatible_format_only = true;
    }
  }

  // Initialize the GXF graph optimizer

  // Load built-in preset extension specs
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Loading built-in preset extension specs");
  for (const std::string & preset_name : PRESET_EXTENSION_SPEC_NAMES) {
    auto load_result =
      optimizer_.loadPresetExtensionSpecs(preset_name);
    if (!load_result) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] loadPresetExtensionSpecs Error: " <<
        GxfResultStr(load_result.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // Load built-in extension specs
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Loading built-in extension specs");
  for (const std::string & filename : BUILTIN_EXTENSION_SPEC_FILENAMES) {
    auto load_result =
      optimizer_.loadExtensionSpecs(nitros_package_share_directory_ + "/" + filename);
    if (!load_result) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] loadExtensionSpecs Error: " << GxfResultStr(load_result.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // Load preset extension specs
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Loading preset extension specs");
  for (const std::string & preset_name : extension_spec_preset_names_) {
    auto load_result =
      optimizer_.loadPresetExtensionSpecs(preset_name);
    if (!load_result) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] loadPresetExtensionSpecs Error: " <<
        GxfResultStr(load_result.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // Load extension specs
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Loading extension specs");
  for (const std::string & filename : extension_spec_filenames_) {
    auto load_result = optimizer_.loadExtensionSpecs(package_share_directory_ + "/" + filename);
    if (!load_result) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] loadExtensionSpecs Error: " << GxfResultStr(load_result.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // Load generator rules
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Loading generator rules");
  for (const std::string & filename : generator_rule_filenames_) {
    auto load_result = optimizer_.loadRules(package_share_directory_ + "/" + filename);
    if (!load_result) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] loadRules Error: " << GxfResultStr(load_result.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // Load required extension files
  RCLCPP_INFO(get_logger(), "[NitrosNode] Loading extensions");
  std::vector<std::pair<std::string, std::string>> combined_extensions = BUILTIN_EXTENSIONS;
  combined_extensions.insert(
    combined_extensions.end(),
    extensions_.begin(),
    extensions_.end());
  gxf_result_t code;
  for (const auto & extension_pair : combined_extensions) {
    const std::string package_directory =
      ament_index_cpp::get_package_share_directory(extension_pair.first);
    code = nitros_context_ptr_->loadExtension(package_directory, extension_pair.second);
    if (code != GXF_SUCCESS) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] loadExtensions Error: " << GxfResultStr(code);
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }
  // Load extensions of the registered NITROS types
  nitros_type_manager_->loadExtensions();

  if (!use_raw_graph_no_optimizer_) {
    // Load graph
    RCLCPP_INFO(get_logger(), "[NitrosNode] Loading graph to the optimizer");
    auto load_graph_result =
      optimizer_.loadGraph(package_share_directory_ + "/" + app_yaml_filename_);
    if (!load_graph_result) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] optimizer_.loadGraph Error: " <<
        GxfResultStr(load_graph_result.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    // Run optimization
    RCLCPP_INFO(get_logger(), "[NitrosNode] Running optimization");
    auto optimize_result =
      optimizer_.optimize(nvidia::gxf::optimizer::gxf_cog_factor_t::THROUGHPUT);
    if (!optimize_result) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] optimize Error: " << GxfResultStr(optimize_result.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  if (!use_custom_io_group_info_list_) {
    // Use the optimizer to get IO groups for the underlying graph
    RCLCPP_INFO(
      get_logger(),
      "[NitrosNode] Obtaining graph IO group info from the optimizer");
    auto maybe_gxf_io_group_info_list = optimizer_.exportGraphIOGroupSupportedDataTypesInfoList();
    if (!maybe_gxf_io_group_info_list) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] exportGraphIOGroupSupportedDataTypesInfoList Error: " <<
        GxfResultStr(maybe_gxf_io_group_info_list.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
    gxf_io_group_info_list_ = maybe_gxf_io_group_info_list.value();
  } else {
    RCLCPP_INFO(get_logger(), "[NitrosNode] Use the given graph IO group info");
  }

  // Print all the supported data formats
  RCLCPP_DEBUG(get_logger().get_child("NitrosNode"), "Supported data format combinations:");
  for (size_t group_index = 0; group_index < gxf_io_group_info_list_.size(); group_index++) {
    RCLCPP_DEBUG(get_logger().get_child("NitrosNode"), "#%ld I/O group:", group_index + 1);
    const auto & gxf_io_group = gxf_io_group_info_list_[group_index];
    for (size_t combo_index = 0;
      combo_index < gxf_io_group.supported_data_types.size();
      combo_index++)
    {
      const auto & supported_data_type_map = gxf_io_group.supported_data_types[combo_index];
      RCLCPP_DEBUG(
        get_logger().get_child("NitrosNode"), "\t#%ld format combination:",
        combo_index + 1);
      // ingress ports
      for (const auto & ingress_comp_info : gxf_io_group.ingress_infos) {
        const std::string component_key =
          gxf::optimizer::GenerateComponentKey(ingress_comp_info);
        const std::string supported_format = supported_data_type_map.at(component_key);
        RCLCPP_DEBUG(
          get_logger().get_child("NitrosNode"), "\t\t[in]\t%s: %s",
          component_key.c_str(), supported_format.c_str());
      }
      // egress ports
      for (const auto & egress_comp_info : gxf_io_group.egress_infos) {
        const std::string component_key =
          gxf::optimizer::GenerateComponentKey(egress_comp_info);
        const std::string supported_format = supported_data_type_map.at(component_key);
        RCLCPP_DEBUG(
          get_logger().get_child("NitrosNode"), "\t\t[out]\t%s: %s",
          component_key.c_str(), supported_format.c_str());
      }
    }
  }

  // Create publishers and subscribers from each GXF ingress-egress group
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Creating negotiated publishers/subscribers");
  for (const auto & gxf_io_group : gxf_io_group_info_list_) {
    nitros_pub_sub_groups_.emplace_back(
      std::make_shared<NitrosPublisherSubscriberGroup>(
        *this, nitros_context_ptr_->getContext(),
        nitros_type_manager_, gxf_io_group, config_map_, frame_id_map_ptr_, *diagnostics_config_));
  }

  // Start negotiation
  RCLCPP_INFO(get_logger(), "[NitrosNode] Starting negotiation...");
  for (auto & nitros_pub_sub_group : nitros_pub_sub_groups_) {
    nitros_pub_sub_group->start();
  }

  // Set the negotiation timer
  negotiation_timer_ = create_wall_timer(
    std::chrono::seconds(type_negotiation_duration_s_),
    [this]() -> void {
      negotiation_timer_->cancel();
      postNegotiationCallback();
    });
}

void NitrosNode::postNegotiationCallback()
{
  RCLCPP_INFO(get_logger(), "[NitrosNode] Starting post negotiation setup");

  // Get negotiated data types
  RCLCPP_INFO(get_logger(), "[NitrosNode] Getting data format negotiation results");
  std::vector<GraphIOGroupDataTypeConfigurations> configs;
  for (const auto & nitros_pub_sub_group : nitros_pub_sub_groups_) {
    nitros_pub_sub_group->postNegotiationCallback();
    auto io_group_config = nitros_pub_sub_group->getDataFormatConfigurations();
    configs.push_back(io_group_config);
  }

  std::string temp_yaml_filename;
  if (!use_raw_graph_no_optimizer_) {
    std::string node_graph_export_directory =
      std::string(kGraphExportDirectory) + "/" + graph_namespace_;
    std::filesystem::create_directories(node_graph_export_directory);
    // Get the graph with the data formats assigned
    RCLCPP_INFO(
      get_logger(),
      "[NitrosNode] Exporting the final graph based on the negotiation results");
    auto export_result = optimizer_.exportGraphToFiles(
      configs, node_graph_export_directory, graph_namespace_, graph_namespace_);
    if (!export_result) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] exportGraphToFiles Error: " <<
        GxfResultStr(export_result.error());
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
    temp_yaml_filename = node_graph_export_directory + "/" + graph_namespace_ + ".yaml";
  } else {
    // Use the original graph intact
    temp_yaml_filename = package_share_directory_ + "/" + app_yaml_filename_;
  }

  RCLCPP_INFO(
    get_logger(),
    "[NitrosNode] Wrote the final top level YAML graph to \"%s\"", temp_yaml_filename.c_str());

  // Load the application graph
  gxf_result_t code;

  // Call the user's pre-load-graph initialization callback
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Calling user's pre-load-graph callback");
  preLoadGraphCallback();

  // Load the new YAML graph that's configured with the selected data types
  RCLCPP_INFO(get_logger(), "[NitrosNode] Loading application");
  code = nitros_context_ptr_->loadApplication(temp_yaml_filename);
  if (code != GXF_SUCCESS) {
    std::stringstream error_msg;
    error_msg << "[NitrosNode] LoadApplication Error: " << GxfResultStr(code);
    RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Get the pointers to the ingress/egress components in the loaded GXF graph and set
  // them to the coresponding Nitros publishers/subscribers
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Linking Nitros pub/sub to the loaded application");
  for (auto & pub_sub_group_ptr : nitros_pub_sub_groups_) {
    // Subscribers
    for (auto & sub_ptr : pub_sub_group_ptr->getNitrosSubscribers()) {
      auto comp_info = sub_ptr->getComponentInfo();
      void * pointer;
      code = nitros_context_ptr_->getComponentPointer(
        comp_info.entity_name,
        comp_info.component_name,
        comp_info.component_type_name,
        &pointer);
      if (code != GXF_SUCCESS) {
        std::stringstream error_msg;
        error_msg <<
          "[NitrosNode] Failed to get the pointer of " <<
          comp_info.component_type_name.c_str() << " (" <<
          comp_info.entity_name.c_str() << "/" <<
          comp_info.component_name.c_str() << ") for linking a NitrosSubscriber: " <<
          GxfResultStr(code);
        RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
        throw std::runtime_error(error_msg.str().c_str());
      }
      RCLCPP_DEBUG(
        get_logger(),
        "[NitrosNode] Setting up a subscriber's underyling receiver (pointer=%p)", pointer);
      sub_ptr->setReceiverPointer(pointer);
      sub_ptr->setReceiverPolicy(0);
      const std::string topic_qos_parameter_name = sub_ptr->getConfig().topic_name + "_qos_depth";
      const int custom_sub_qos_depth = get_parameter(topic_qos_parameter_name).as_int();
      if (custom_sub_qos_depth > 0) {
        sub_ptr->setReceiverCapacity(custom_sub_qos_depth);
      }
      if (heartbeat_eid_ == -1) {
        nitros_context_ptr_->getEid(comp_info.entity_name, heartbeat_eid_);
      }
    }

    // Publishers
    for (auto & pub_ptr : pub_sub_group_ptr->getNitrosPublishers()) {
      auto comp_info = pub_ptr->getComponentInfo();
      if (comp_info.component_type_name == kGxfVaultComponentTypeName) {
        void * pointer = nullptr;
        code = nitros_context_ptr_->getComponentPointer(
          comp_info.entity_name,
          comp_info.component_name,
          comp_info.component_type_name,
          &pointer);
        pub_ptr->setVaultPointer(pointer);
        RCLCPP_DEBUG(
          get_logger(), "[NitrosNode] Set a publisher's pointer: %p", pointer);
        pub_ptr->startGxfVaultPeriodicPollingTimer();
      } else if (comp_info.component_type_name == kGxfMessageRelayComponentTypeName) {
        // Set egress component's pointer
        void * pointer = nullptr;
        code = nitros_context_ptr_->getComponentPointer(
          comp_info.entity_name,
          comp_info.component_name,
          comp_info.component_type_name,
          &pointer);
        pub_ptr->setMessageRelayPointer(pointer);
        pub_ptr->enableNitrosPublisherWaitable();

        // Set egress component's callback function
        void * cb_func_pointer = &pub_ptr->getGxfMessageRelayCallbackFunc();
        nitros_context_ptr_->setParameterInt64(
          comp_info.entity_name, comp_info.component_type_name,
          "callback_address",
          reinterpret_cast<uintptr_t>(cb_func_pointer));
        RCLCPP_DEBUG(
          get_logger(), "[NitrosNode] Set a publisher's callback pointer: %p", cb_func_pointer);
      } else {
        std::stringstream error_msg;
        error_msg <<
          "[NitrosNode] Unsupported egress component type: " <<
          comp_info.component_type_name.c_str() << " (" <<
          comp_info.entity_name.c_str() << "/" <<
          comp_info.component_name.c_str() << ")";
        RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
        throw std::runtime_error(error_msg.str().c_str());
      }
      if (heartbeat_eid_ == -1) {
        nitros_context_ptr_->getEid(comp_info.entity_name, heartbeat_eid_);
      }
    }
  }

  // Call the user's post-load-graph initialization callback
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Calling user's post-load-graph callback");
  postLoadGraphCallback();

  RCLCPP_INFO(get_logger(), "[NitrosNode] Initializing and running GXF graph");
  code = nitros_context_ptr_->runGraphAsync();
  if (code != GXF_SUCCESS) {
    std::stringstream error_msg;
    error_msg << "[NitrosNode] runGraphAsync Error: " << GxfResultStr(code);
    RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Set the timer for checking the status of the underlying graph
  if (heartbeat_eid_ != -1) {
    RCLCPP_DEBUG(
      get_logger(),
      "[NitrosNode] Starting a heartbeat timer (eid=%ld)",
      heartbeat_eid_);
    gxf_heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() -> void {
        gxfHeartbeatCallback();
      });
  }

  // Enable NitrosSbuscriber's callbacks
  RCLCPP_DEBUG(get_logger(), "[NitrosNode] Enabling NitrosSubscriber's callbacks");
  for (auto & pub_sub_group_ptr : nitros_pub_sub_groups_) {
    for (auto & sub_ptr : pub_sub_group_ptr->getNitrosSubscribers()) {
      sub_ptr->setIsGxfRunning(true);
    }
  }

  RCLCPP_INFO(get_logger(), "[NitrosNode] Node was started");
}

void NitrosNode::gxfHeartbeatCallback()
{
  std::stringstream nvtx_tag_name;
  nvtx_tag_name <<
    "[" << get_name() << "] NitrosNode::gxfHeartbeatCallback()";
  nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_GRAY);

  if (heartbeat_eid_ == -1) {
    RCLCPP_ERROR(get_logger(), "[NitrosNode] The heartbeat entity was not set");
    nvtxRangePopWrapper();
    rclcpp::shutdown();
  }

  // Check the status of the target heartbeat entity.
  // gxf_entity_status_t is expected to be GXF_ENTITY_STATUS_STARTED (2)
  // when the graph is running
  gxf_entity_status_t entity_status;
  gxf_result_t code =
    GxfEntityGetStatus(nitros_context_ptr_->getContext(), heartbeat_eid_, &entity_status);
  if (code == GXF_ENTITY_NOT_FOUND) {
    RCLCPP_WARN(
      get_logger(),
      "[NitrosNode] The heartbeat entity (eid=%ld) was stopped. "
      "The graph may have been terminated.",
      heartbeat_eid_);
    nvtxRangePopWrapper();
    rclcpp::shutdown();
  } else if (code != GXF_SUCCESS) {
    RCLCPP_WARN(
      get_logger(),
      "[NitrosNode] Failed to get the heartbeat entity (eid=%ld) status: %s",
      heartbeat_eid_,
      GxfResultStr(code));
    nvtxRangePopWrapper();
    rclcpp::shutdown();
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger(
      std::string("GXFMonitor.") + std::string(get_logger().get_name())),
    "Heartbeat entity (eid=%ld) status: %d",
    heartbeat_eid_,
    static_cast<int>(entity_status));
  nvtxRangePopWrapper();
}

void NitrosNode::preLoadGraphSetParameter(
  const std::string & entity_name,
  const std::string & component_name,
  const std::string & parameter_name,
  const std::string & value
)
{
  nitros_context_ptr_->preLoadGraphSetParameter(entity_name, component_name, parameter_name, value);
}

std::shared_ptr<NitrosPublisher> NitrosNode::findNitrosPublisher(
  const gxf::optimizer::ComponentInfo & comp_info)
{
  for (auto & nitros_pub_sub_group : nitros_pub_sub_groups_) {
    auto nitros_pub = nitros_pub_sub_group->findNitrosPublisher(comp_info);
    if (nitros_pub != nullptr) {
      return nitros_pub;
    }
  }
  return nullptr;
}

std::shared_ptr<NitrosSubscriber> NitrosNode::findNitrosSubscriber(
  const gxf::optimizer::ComponentInfo & comp_info)
{
  {
    for (auto & nitros_pub_sub_group : nitros_pub_sub_groups_) {
      auto nitros_sub = nitros_pub_sub_group->findNitrosSubscriber(comp_info);
      if (nitros_sub != nullptr) {
        return nitros_sub;
      }
    }
    return nullptr;
  }
}

std::string NitrosNode::getNegotiatedDataFormat(const ComponentInfo comp_info) const
{
  for (auto & nitros_pub_sub_group : nitros_pub_sub_groups_) {
    auto nitros_sub = nitros_pub_sub_group->findNitrosSubscriber(comp_info);
    if (nitros_sub != nullptr) {
      return nitros_sub->getNegotiatedDataFormat();
    }
    auto nitros_pub = nitros_pub_sub_group->findNitrosPublisher(comp_info);
    if (nitros_pub != nullptr) {
      return nitros_pub->getNegotiatedDataFormat();
    }
  }
  RCLCPP_WARN(
    get_logger(),
    "[NitrosNode] Could not recognize the component \"%s/%s\" (type=\"%s\") for "
    "getting its negotiated data format",
    comp_info.entity_name.c_str(),
    comp_info.component_name.c_str(),
    comp_info.component_type_name.c_str());
  return "";
}

std::string NitrosNode::getFinalDataFormat(const ComponentInfo comp_info) const
{
  for (auto & nitros_pub_sub_group : nitros_pub_sub_groups_) {
    auto nitros_sub = nitros_pub_sub_group->findNitrosSubscriber(comp_info);
    if (nitros_sub != nullptr) {
      return nitros_sub->getFinalDataFormat();
    }
    auto nitros_pub = nitros_pub_sub_group->findNitrosPublisher(comp_info);
    if (nitros_pub != nullptr) {
      return nitros_pub->getFinalDataFormat();
    }
  }
  RCLCPP_WARN(
    get_logger(),
    "[NitrosNode] Could not recognize the component \"%s/%s\" (type=\"%s\") for "
    "getting its final data format",
    comp_info.entity_name.c_str(),
    comp_info.component_name.c_str(),
    comp_info.component_type_name.c_str());
  return "";
}

void NitrosNode::setUseCustomIOGroupInfoList(const bool use_custom)
{
  use_custom_io_group_info_list_ = use_custom;
}

void NitrosNode::addExtensionSpecFilename(std::string package_relative_filepath)
{
  extension_spec_filenames_.push_back(package_relative_filepath);
}

void NitrosNode::addGeneratorRuleFilename(std::string package_relative_filepath)
{
  generator_rule_filenames_.push_back(package_relative_filepath);
}

void NitrosNode::setAppYamlFilename(std::string package_relative_filepath)
{
  app_yaml_filename_ = package_relative_filepath;
}

void NitrosNode::setFrameIdSource(std::string source_frame_id_map_key, std::string frame_id)
{
  if (frame_id_map_ptr_ == nullptr) {
    std::stringstream error_msg;
    error_msg <<
      "[NitrosNode] setFrameIdSource was called before frame_id_map_ptr_ was initialized";
    RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  (*frame_id_map_ptr_.get())[source_frame_id_map_key] = frame_id;
  RCLCPP_DEBUG(
    get_logger(),
    "[NitrosNode] Set frame id for key %s to %s", source_frame_id_map_key.c_str(),
    (*frame_id_map_ptr_.get())[source_frame_id_map_key].c_str());
}

NitrosNode::~NitrosNode()
{
  RCLCPP_INFO(get_logger(), "[NitrosNode] Terminating the running application");
  nitros_context_ptr_->destroy();
  RCLCPP_INFO(get_logger(), "[NitrosNode] Application termination done");
}


}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
