/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "extensions/gxf_optimizer/core/optimizer.hpp"
#include "gxf/core/gxf.h"

#include "isaac_ros_nitros/nitros_node.hpp"
#include "isaac_ros_nitros/types/nitros_empty.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

const std::vector<std::string> BUILTIN_EXTENSION_SPEC_FILENAMES = {};

const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_nitros",
};

namespace
{
// Generate a random string of the given length
unsigned int seed = time(NULL);
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// The constructor for starting a test node
NitrosNode::NitrosNode(const rclcpp::NodeOptions & options)
: NitrosNode(
    options,
    // Application graph filename
    "config/test_forward_node.yaml",
    // I/O configuration map
      {
        {"forward/input",
          {
            .type = NitrosPublisherSubscriberType::NEGOTIATED,
            .qos = rclcpp::QoS(1),
            .compatible_data_format = "nitros_empty",
            .topic_name = "topic_forward_input",
          }
        },
        {"vault/vault",
          {
            .type = NitrosPublisherSubscriberType::NEGOTIATED,
            .qos = rclcpp::QoS(1),
            .compatible_data_format = "nitros_empty",
            .topic_name = "topic_forward_output",
          }
        }
      },
    // Extension specs
    {},
    // Optimizer's rule filenames
    {},
    // Extension so file list
      {
        {"isaac_ros_gxf", "gxf/lib/std/libgxf_std.so"},
        {"isaac_ros_gxf", "gxf/lib/cuda/libgxf_cuda.so"},
        {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
        {"isaac_ros_gxf", "gxf/lib/libgxf_message_compositor.so"}
      },
    // Test node package name
    "isaac_ros_nitros")
{
  std::string compatible_format = declare_parameter<std::string>("compatible_format", "");
  if (!compatible_format.empty()) {
    config_map_["forward/input"].compatible_data_format = compatible_format;
    config_map_["forward/input"].use_compatible_format_only = true;

    config_map_["vault/vault"].compatible_data_format = compatible_format;
    config_map_["vault/vault"].use_compatible_format_only = true;
  }

  // Register NitrosEmpty for POL test
  registerSupportedType<NitrosEmpty>();

  startNitrosNode();
}
#pragma GCC diagnostic pop

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
  package_name_(package_name)
{
  RCLCPP_INFO(get_logger(), "[NitrosNode] Initializing NitrosNode");

  // This line triggers the creation of a type adapter context that
  // loads a GXF graph containing common components accessible by
  // all NitrosNode subclasses created in the same process.
  GetTypeAdapterNitrosContext();

  nitros_context_.setNode(this);

  // Create an NITROS type manager for the node
  statistics_config_ = std::make_shared<NitrosStatisticsConfig>();

  statistics_config_->enable_statistics = declare_parameter<bool>("enable_statistics", false);
  statistics_config_->statistics_publish_rate = declare_parameter<float>(
    "statistics_publish_rate",
    1.0);
  statistics_config_->filter_window_size = declare_parameter<int>("filter_window_size", 100);

  // Set extension log level from an user-specified parameter
  nitros_context_.setExtensionLogSeverity(
    static_cast<gxf_severity_t>(declare_parameter<uint16_t>(
      "extension_log_level",
      gxf_severity_t::GXF_SEVERITY_WARNING)));

  // Genearate a random unique namespace for the underlying GXF graph
  graph_namespace_ = GenerateRandomString(10);
  nitros_context_.setGraphNamespace(graph_namespace_);
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
  return nitros_context_;
}

void NitrosNode::startNitrosNode()
{
  RCLCPP_INFO(get_logger(), "[NitrosNode] Starting NitrosNode");

  // Initialize the GXF graph optimizer

  // Load built-in preset extension specs
  RCLCPP_INFO(get_logger(), "[NitrosNode] Loading built-in preset extension specs");
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
  RCLCPP_INFO(get_logger(), "[NitrosNode] Loading built-in extension specs");
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
  RCLCPP_INFO(get_logger(), "[NitrosNode] Loading preset extension specs");
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
  RCLCPP_INFO(get_logger(), "[NitrosNode] Loading extension specs");
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
  RCLCPP_INFO(get_logger(), "[NitrosNode] Loading generator rules");
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
  gxf_result_t code;
  for (const auto & extension_pair : extensions_) {
    const std::string package_directory =
      ament_index_cpp::get_package_share_directory(extension_pair.first);
    code = nitros_context_.loadExtension(package_directory, extension_pair.second);
    if (code != GXF_SUCCESS) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode] loadExtensions Error: " << GxfResultStr(code);
      RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }
  // Load extensions of the registered NITROS types
  nitros_type_manager_->loadExtensions();

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
  auto optimize_result = optimizer_.optimize(nvidia::gxf::optimizer::gxf_cog_factor_t::THROUGHPUT);
  if (!optimize_result) {
    std::stringstream error_msg;
    error_msg << "[NitrosNode] optimize Error: " << GxfResultStr(optimize_result.error());
    RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
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

  // Create publishers and subscribers from each GXF ingress-egress group
  RCLCPP_INFO(get_logger(), "[NitrosNode] Creating negotiated publishers/subscribers");
  for (const auto & gxf_io_group : gxf_io_group_info_list_) {
    nitros_pub_sub_groups_.emplace_back(
      std::make_shared<NitrosPublisherSubscriberGroup>(
        *this, nitros_context_.getContext(),
        nitros_type_manager_, gxf_io_group, config_map_, frame_id_map_ptr_, *statistics_config_));
  }

  // Start negotiation
  RCLCPP_INFO(get_logger(), "[NitrosNode] Starting negotiation...");
  for (auto & nitros_pub_sub_group : nitros_pub_sub_groups_) {
    nitros_pub_sub_group->start();
  }

  // Set the negotiation timer
  negotiation_timer_ = create_wall_timer(
    std::chrono::seconds(1),
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

  // Get the graph with the data formats assigned
  RCLCPP_INFO(
    get_logger(),
    "[NitrosNode] Exporting the final graph based on the negotiation results");
  auto export_result = optimizer_.exportGraphToFiles(
    configs, package_share_directory_, graph_namespace_, graph_namespace_);
  if (!export_result) {
    std::stringstream error_msg;
    error_msg << "[NitrosNode] exportGraphToFiles Error: " <<
      GxfResultStr(export_result.error());
    RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  std::string temp_yaml_filename = package_share_directory_ + "/" + graph_namespace_ + ".yaml";
  RCLCPP_INFO(
    get_logger(),
    "[NitrosNode] Wrote the final top level YAML graph to \"%s\"", temp_yaml_filename.c_str());

  // Load the application graph
  gxf_result_t code;

  // Call the user's pre-load-graph initialization callback
  RCLCPP_INFO(get_logger(), "[NitrosNode] Calling user's pre-load-graph callback");
  preLoadGraphCallback();

  // Load the new YAML graph that's configured with the selected data types
  RCLCPP_INFO(get_logger(), "[NitrosNode] Loading application");
  code = nitros_context_.loadApplication(temp_yaml_filename);
  if (code != GXF_SUCCESS) {
    std::stringstream error_msg;
    error_msg << "[NitrosNode] LoadApplication Error: " << GxfResultStr(code);
    RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Get the pointers to the ingress/egress components in the loaded GXF graph and set
  // them to the coresponding Nitros publishers/subscribers
  RCLCPP_INFO(get_logger(), "[NitrosNode] Linking Nitros pub/sub to the loaded application");
  for (auto & pub_sub_group_ptr : nitros_pub_sub_groups_) {
    // Subscribers
    for (auto & sub_ptr : pub_sub_group_ptr->getNitrosSubscribers()) {
      auto comp_info = sub_ptr->getComponentInfo();
      void * pointer;
      code = nitros_context_.getComponentPointer(
        comp_info.entity_name,
        comp_info.component_name,
        comp_info.component_type_name,
        &pointer);
      sub_ptr->setReceiverPointer(pointer);
      RCLCPP_DEBUG(get_logger(), "[NitrosNode] Setting a subscriber's pointer: %p", pointer);
    }

    // Publishers
    for (auto & pub_ptr : pub_sub_group_ptr->getNitrosPublishers()) {
      auto comp_info = pub_ptr->getComponentInfo();
      void * pointer;
      code = nitros_context_.getComponentPointer(
        comp_info.entity_name,
        comp_info.component_name,
        comp_info.component_type_name,
        &pointer);
      pub_ptr->setVaultPointer(pointer);
      RCLCPP_DEBUG(get_logger(), "[NitrosNode] Setting a publisher's pointer: %p", pointer);
      pub_ptr->startGxfVaultPeriodicPollingTimer();
    }
  }

  // Call the user's post-load-graph initialization callback
  RCLCPP_INFO(get_logger(), "[NitrosNode] Calling user's post-load-graph callback");
  postLoadGraphCallback();

  code = nitros_context_.runGraphAsync();
  if (code != GXF_SUCCESS) {
    std::stringstream error_msg;
    error_msg << "[NitrosNode] runGraphAsync Error: " << GxfResultStr(code);
    RCLCPP_ERROR(get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
}

void NitrosNode::preLoadGraphSetParameter(
  const std::string & entity_name,
  const std::string & component_name,
  const std::string & parameter_name,
  const std::string & value
)
{
  nitros_context_.preLoadGraphSetParameter(entity_name, component_name, parameter_name, value);
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
  nitros_context_.destroy();
  RCLCPP_INFO(get_logger(), "[NitrosNode] Application termination done");
}


}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosNode)
