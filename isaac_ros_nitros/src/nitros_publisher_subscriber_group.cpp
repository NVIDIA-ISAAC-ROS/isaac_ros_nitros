/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "isaac_ros_nitros/nitros_publisher_subscriber_group.hpp"

#include "extensions/gxf_optimizer/common/type.hpp"
#include "negotiated/combinations.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosPublisherSubscriberGroup::NitrosPublisherSubscriberGroup(
  rclcpp::Node & node,
  const gxf_context_t context,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const gxf::optimizer::GraphIOGroupSupportedDataTypesInfo & gxf_io_supported_data_formats_info,
  const NitrosPublisherSubscriberConfigMap & nitros_pub_sub_configs,
  const std::shared_ptr<std::map<ComponentKey, std::string>> frame_id_map_ptr,
  const NitrosStatisticsConfig & statistics_config)
: node_(node),
  context_(context),
  nitros_type_manager_(nitros_type_manager),
  gxf_io_supported_data_formats_info_(gxf_io_supported_data_formats_info),
  nitros_pub_sub_configs_(nitros_pub_sub_configs),
  frame_id_map_ptr_(frame_id_map_ptr),
  statistics_config_(statistics_config)
{
  // Expand data formats if all ports in this IO group support "any" data formats
  expandAnyDataFormats();

  // Remove undesired supported data formats if needed
  if (!applyUseCompatibleFormatOnly()) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[NitrosPublisherSubscriberGroup] applyUseCompatibleFormatOnly Error");
    throw std::runtime_error(
            "[NitrosPublisherSubscriberGroup] applyUseCompatibleFormatOnly Error");
  }

  // Create Nitros subscribers
  createNitrosSubscribers();

  // Create Nitros publishers
  createNitrosPublishers();
}

std::vector<std::shared_ptr<NitrosPublisher>>
NitrosPublisherSubscriberGroup::getNitrosPublishers()
{
  return nitros_pubs_;
}

std::vector<std::shared_ptr<NitrosSubscriber>>
NitrosPublisherSubscriberGroup::getNitrosSubscribers()
{
  return nitros_subs_;
}

void NitrosPublisherSubscriberGroup::postNegotiationCallback()
{
  for (auto & nitros_pub : nitros_pubs_) {
    nitros_pub->postNegotiationCallback();
  }
  for (auto & nitros_sub : nitros_subs_) {
    nitros_sub->postNegotiationCallback();
  }

  // In the case that some pubs/subs have sucessful negotiation and some don't (and hence
  // compatible formats are chosen for these pubs/subs), the resulting group format
  // combaination may not be valid. We validate and adjust the compatible formats as needed.
  postNegotiationAdjustCompatibleFormats();
}

std::vector<gxf::optimizer::ComponentInfo>
NitrosPublisherSubscriberGroup::getAllComponentInfos() const
{
  std::vector<gxf::optimizer::ComponentInfo> component_info_list =
    gxf_io_supported_data_formats_info_.ingress_infos;
  component_info_list.insert(
    component_info_list.end(),
    gxf_io_supported_data_formats_info_.egress_infos.begin(),
    gxf_io_supported_data_formats_info_.egress_infos.end());
  return component_info_list;
}

void NitrosPublisherSubscriberGroup::expandAnyDataFormats()
{
  // In the case that all ingress/egress ports in this group support "any" format,
  // the supported data formats are expanded using the registered data formats.
  // In such a case, the given IO group data format support information would
  // only contain one data format combination with all port's supported data format
  // as "any".
  if (gxf_io_supported_data_formats_info_.supported_data_types.size() == 1) {
    bool to_be_resolved = true;
    // Check if every port's supported data format is "any"
    for (const auto & supported_data_type :
      gxf_io_supported_data_formats_info_.supported_data_types[0])
    {
      if (supported_data_type.second != gxf::optimizer::kAnyDataType) {
        to_be_resolved = false;
        break;
      }
    }
    if (to_be_resolved == false) {
      return;
    }

    // This IO group has all ingress and egress ports supporting "any".
    // We make them actually support any format by expanding the formats with all
    // the registered data formats.
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosPublisherSubscriberGroup] Expanding \"any\" data formats to all "
      "registered data formats");
    gxf_io_supported_data_formats_info_.supported_data_types.clear();
    std::map<ComponentKey, std::string> supported_data_format_map;
    for (const std::string registered_data_format :
      nitros_type_manager_->getAllRegisteredDataFormats())
    {
      std::vector<gxf::optimizer::ComponentInfo> ingress_egress_infos =
        gxf_io_supported_data_formats_info_.ingress_infos;
      ingress_egress_infos.insert(
        ingress_egress_infos.end(),
        gxf_io_supported_data_formats_info_.egress_infos.begin(),
        gxf_io_supported_data_formats_info_.egress_infos.end()
      );
      for (const auto & ingress_comp_info : ingress_egress_infos) {
        const std::string component_key =
          gxf::optimizer::GenerateComponentKey(ingress_comp_info);
        supported_data_format_map[component_key] = registered_data_format;
      }
      gxf_io_supported_data_formats_info_.supported_data_types.push_back(supported_data_format_map);
    }
  }
}

bool NitrosPublisherSubscriberGroup::applyUseCompatibleFormatOnly()
{
  // Iterate through each component in the set and remove the supported format sets that
  // do not contain the component's compatible format
  for (const auto & comp_info : getAllComponentInfos()) {
    ComponentKey comp_key = GenerateComponentKey(comp_info);
    if (nitros_pub_sub_configs_.count(comp_key) == 0) {
      RCLCPP_WARN(
        node_.get_logger(),
        "[NitrosPublisherSubscriberGroup] Could not find a config for component \"%s/%s\" "
        "(type=\"%s\")",
        comp_info.entity_name.c_str(),
        comp_info.component_name.c_str(),
        comp_info.component_type_name.c_str());
      continue;
    }
    if (nitros_pub_sub_configs_[comp_key].use_compatible_format_only == false) {
      continue;
    }
    std::string compatible_format = nitros_pub_sub_configs_[comp_key].compatible_data_format;
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosPublisherSubscriberGroup] Pinning the component \"%s/%s\" "
      "(type=\"%s\") to use its compatible format only: \"%s\"",
      comp_info.entity_name.c_str(),
      comp_info.component_name.c_str(),
      comp_info.component_type_name.c_str(),
      compatible_format.c_str());

    std::vector<std::map<ComponentKey, std::string>> filtered_supported_data_types;
    for (const auto & supported_data_type_map :
      gxf_io_supported_data_formats_info_.supported_data_types)
    {
      if (supported_data_type_map.at(comp_key) == compatible_format) {
        // Only keep the supported format map that has the comp_info's compatible format
        filtered_supported_data_types.push_back(supported_data_type_map);
        continue;
      }
    }
    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosPublisherSubscriberGroup] Filtering supported formats based on the compatible "
      "format for the component \"%s/%s\" (type=\"%s\"): # of supported formats %ld -> %ld",
      comp_info.entity_name.c_str(),
      comp_info.component_name.c_str(),
      comp_info.component_type_name.c_str(),
      gxf_io_supported_data_formats_info_.supported_data_types.size(),
      filtered_supported_data_types.size());
    // Update the supported data format list with the filtered one
    gxf_io_supported_data_formats_info_.supported_data_types = filtered_supported_data_types;
    if (gxf_io_supported_data_formats_info_.supported_data_types.size() == 0) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "[NitrosPublisherSubscriberGroup] Supported formats for the component \"%s/%s\" "
        "(type=\"%s\") became emtpy",
        comp_info.entity_name.c_str(),
        comp_info.component_name.c_str(),
        comp_info.component_type_name.c_str());
      return false;
    }
  }
  return true;
}

void NitrosPublisherSubscriberGroup::postNegotiationAdjustCompatibleFormats()
{
  std::vector<std::shared_ptr<NitrosPublisherSubscriberBase>> all_nitros_pubsubs_;
  all_nitros_pubsubs_.insert(all_nitros_pubsubs_.end(), nitros_pubs_.begin(), nitros_pubs_.end());
  all_nitros_pubsubs_.insert(all_nitros_pubsubs_.end(), nitros_subs_.begin(), nitros_subs_.end());

  // Find the valid format combinations based on the successfully negotiated formats
  std::vector<std::map<ComponentKey, std::string>> valid_negotiated_format_combination_maps;
  for (const auto & supported_data_type_map :
    gxf_io_supported_data_formats_info_.supported_data_types)
  {
    bool is_valid_format_combination = true;
    for (auto & nitros_pubsub : all_nitros_pubsubs_) {
      std::string negotiated_format = nitros_pubsub->getNegotiatedDataFormat();
      if (negotiated_format.empty()) {
        continue;
      }
      ComponentKey comp_key = GenerateComponentKey(nitros_pubsub->getComponentInfo());
      if (supported_data_type_map.at(comp_key) != negotiated_format) {
        is_valid_format_combination = false;
        break;
      }
    }
    if (is_valid_format_combination) {
      valid_negotiated_format_combination_maps.push_back(supported_data_type_map);
    }
  }

  if (valid_negotiated_format_combination_maps.size() == 0) {
    // No valid data format combination based on the successfully negotiated formats
    std::string error_msg = "[NitrosPublisherSubscriberGroup] Detected inconsistent negotiated "
      "formats! The selected data format combination was invalid.";
    RCLCPP_ERROR(node_.get_logger(), error_msg.c_str());
    throw std::runtime_error(error_msg.c_str());
  }

  // At this point, there is one or more valid format combinations based on the successfully
  // negotiated formats. Based on these valid combinations, find the follwoing two groups of
  // valid combinations:
  // 1. the format combinations that match all the compatible formats for those pubs/subs that
  //    have unsuccessful format negotiation
  // 2. the format combinations that match all the compatible formats for those pubs/subs that
  //    have unsuccessful format negotiation and have `use_flexible_compatible_format` set as
  //    `false` (i.e., those with `use_flexible_compatible_format = true` don't have to match).
  std::vector<std::map<ComponentKey, std::string>> valid_final_format_combination_maps;
  std::vector<std::map<ComponentKey, std::string>> valid_flexible_format_combination_maps;
  for (const auto & supported_data_type_map : valid_negotiated_format_combination_maps) {
    bool is_valid_format_combination = true;
    bool is_valid_flexible_format_combination = true;
    for (auto & nitros_pubsub : all_nitros_pubsubs_) {
      if (!nitros_pubsub->getNegotiatedDataFormat().empty()) {
        continue;
      }
      std::string compatible_format = nitros_pubsub->getCompatibleDataFormat();
      ComponentKey comp_key = GenerateComponentKey(nitros_pubsub->getComponentInfo());
      if (supported_data_type_map.at(comp_key) != compatible_format) {
        is_valid_format_combination = false;
        if (nitros_pub_sub_configs_[comp_key].use_flexible_compatible_format == false) {
          is_valid_flexible_format_combination = false;
          break;
        }
      }
    }
    if (is_valid_format_combination) {
      valid_final_format_combination_maps.push_back(supported_data_type_map);
    }
    if (is_valid_flexible_format_combination) {
      valid_flexible_format_combination_maps.push_back(supported_data_type_map);
    }
  }
  if (valid_final_format_combination_maps.size() > 0) {
    // The current negotiated formats + compatible formats combination is valid, so
    // continue using the existing selections.
    return;
  }

  if (valid_flexible_format_combination_maps.size() == 0) {
    // No valid format combination exists even with considering flexible compatible formats
    // for those pubs/subs that have `use_flexible_compatible_format = true`
    std::string error_msg = "[NitrosPublisherSubscriberGroup] Detected invalid format "
      "combination due to inflexible compatible formats.";
    RCLCPP_ERROR(node_.get_logger(), error_msg.c_str());
    throw std::runtime_error(error_msg.c_str());
  }

  // A valid format combination has been found with some pubs/subs being configured with
  // flexible compatible format (i.e., `use_flexible_compatible_format = true`).
  // Adjust the compatible formats for those pubs/subs based on the first (top priority)
  // valid format combination.
  for (auto & nitros_pubsub : all_nitros_pubsubs_) {
    ComponentKey comp_key = GenerateComponentKey(nitros_pubsub->getComponentInfo());

    if (!nitros_pubsub->getNegotiatedDataFormat().empty() ||
      (nitros_pub_sub_configs_[comp_key].use_flexible_compatible_format == false))
    {
      // This pub/sub is either negotiated successfully or configured to have a fixed
      // compatible format, so skip it.
      continue;
    }

    std::string old_compatible_format = nitros_pubsub->getCompatibleDataFormat();
    std::string valid_compatible_format = valid_flexible_format_combination_maps[0].at(comp_key);

    if (old_compatible_format == valid_compatible_format) {
      // The valid format remains the same
      continue;
    }

    // Adjust the compatible format to be the one from the top valid format combination
    nitros_pubsub->setCompatibleDataFormat(valid_compatible_format);
    nitros_pub_sub_configs_[comp_key].compatible_data_format = valid_compatible_format;

    auto comp_info = nitros_pubsub->getComponentInfo();
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosPublisherSubscriberGroup] Adjusted the compatible format of the component "
      "\"%s/%s\" (type=\"%s\") from \"%s\" to \"%s\"",
      comp_info.entity_name.c_str(),
      comp_info.component_name.c_str(),
      comp_info.component_type_name.c_str(),
      old_compatible_format.c_str(),
      valid_compatible_format.c_str());
  }
}

void NitrosPublisherSubscriberGroup::createNitrosSubscribers()
{
  // Create subscribers
  for (const auto & ingress_comp_info : gxf_io_supported_data_formats_info_.ingress_infos) {
    const std::string component_key = gxf::optimizer::GenerateComponentKey(ingress_comp_info);
    if (nitros_pub_sub_configs_.count(component_key) == 0) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "[NitrosPublisherSubscriberGroup] Topic name was not specified for a "
        "GXF graph IO");
    }

    std::vector<std::string> supported_data_formats =
      gxf::optimizer::GetSupportedDataTypes(gxf_io_supported_data_formats_info_, ingress_comp_info);

    NitrosPublisherSubscriberConfig component_config = nitros_pub_sub_configs_[component_key];

    auto nitros_sub = std::make_shared<NitrosSubscriber>(
      node_,
      context_,
      nitros_type_manager_,
      ingress_comp_info,
      supported_data_formats,
      component_config,
      statistics_config_);

    nitros_sub->setFrameIdMap(frame_id_map_ptr_);

    nitros_subs_.push_back(nitros_sub);
  }
}

void NitrosPublisherSubscriberGroup::createNitrosPublishers()
{
  // Create publishers
  for (const auto & egress_comp_info : gxf_io_supported_data_formats_info_.egress_infos) {
    const std::string component_key = gxf::optimizer::GenerateComponentKey(egress_comp_info);

    if (nitros_pub_sub_configs_.count(component_key) == 0) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "[NitrosPublisherSubscriberGroup] Topic name was not specified for a "
        "GXF graph IO");
    }

    std::vector<std::string> supported_data_formats =
      gxf::optimizer::GetSupportedDataTypes(gxf_io_supported_data_formats_info_, egress_comp_info);

    // Set up callback functions for the negotiated publisher
    negotiated::NegotiatedPublisherOptions negotiated_pub_options;
    negotiated_pub_options.update_downstream_cb =
      std::bind(
      &NitrosPublisherSubscriberGroup::updateUpstreamSubscriberDownstreamSupportedFormatsCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4,
      std::placeholders::_5,
      std::placeholders::_6,
      egress_comp_info);

    negotiated_pub_options.negotiation_cb =
      std::bind(
      &NitrosPublisherSubscriberGroup::publisherNegotiationCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4,
      egress_comp_info);

    NitrosPublisherSubscriberConfig component_config = nitros_pub_sub_configs_[component_key];

    auto nitros_pub = std::make_shared<NitrosPublisher>(
      node_,
      context_,
      nitros_type_manager_,
      egress_comp_info,
      supported_data_formats,
      component_config,
      negotiated_pub_options,
      statistics_config_);

    nitros_pub->setFrameIdMap(frame_id_map_ptr_);

    // Add all subscribers in the same group as upstream dependency
    for (auto & nitros_sub : nitros_subs_) {
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[NitrosPublisherSubscriberGroup] An upstream subscriber added.");
      nitros_pub->getNegotiatedPublisher()->add_upstream_negotiated_subscription(
        nitros_sub->getNegotiatedSubscriber());
    }

    nitros_pubs_.push_back(nitros_pub);
  }
}

std::shared_ptr<NitrosPublisher> NitrosPublisherSubscriberGroup::findNitrosPublisher(
  const gxf::optimizer::ComponentInfo & comp_info)
{
  for (auto & nitros_pub : nitros_pubs_) {
    if (nitros_pub->getComponentInfo() == comp_info) {
      return nitros_pub;
    }
  }
  return nullptr;
}

std::shared_ptr<NitrosSubscriber> NitrosPublisherSubscriberGroup::findNitrosSubscriber(
  const gxf::optimizer::ComponentInfo & comp_info)
{
  for (auto & nitros_sub : nitros_subs_) {
    if (nitros_sub->getComponentInfo() == comp_info) {
      return nitros_sub;
    }
  }
  return nullptr;
}

std::shared_ptr<NitrosSubscriber> NitrosPublisherSubscriberGroup::findNitrosSubscriber(
  const std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub)
{
  for (auto & nitros_sub : nitros_subs_) {
    if (nitros_sub->getNegotiatedSubscriber() == negotiated_sub) {
      return nitros_sub;
    }
  }
  return nullptr;
}

void NitrosPublisherSubscriberGroup::start()
{
  for (auto & nitros_pub : nitros_pubs_) {
    nitros_pub->start();
  }
  for (auto & nitros_sub : nitros_subs_) {
    nitros_sub->start();
  }
}

gxf::optimizer::GraphIOGroupDataTypeConfigurations
NitrosPublisherSubscriberGroup::getDataFormatConfigurations()
{
  nvidia::gxf::optimizer::GraphIOGroupDataTypeConfigurations io_group_config;
  io_group_config.ingress_infos = gxf_io_supported_data_formats_info_.ingress_infos;
  io_group_config.egress_infos = gxf_io_supported_data_formats_info_.egress_infos;

  // Publishers
  for (auto & nitros_pub : nitros_pubs_) {
    auto comp_key = gxf::optimizer::GenerateComponentKey(nitros_pub->getComponentInfo());
    if (nitros_pub->getNegotiatedDataFormat().empty()) {
      io_group_config.data_type_configurations[comp_key] = nitros_pub->getCompatibleDataFormat();
    } else {
      io_group_config.data_type_configurations[comp_key] = nitros_pub->getNegotiatedDataFormat();
    }
  }
  // Subscribers
  for (auto & nitros_sub : nitros_subs_) {
    auto comp_key = gxf::optimizer::GenerateComponentKey(nitros_sub->getComponentInfo());
    if (nitros_sub->getNegotiatedDataFormat().empty()) {
      io_group_config.data_type_configurations[comp_key] = nitros_sub->getCompatibleDataFormat();
    } else {
      io_group_config.data_type_configurations[comp_key] = nitros_sub->getNegotiatedDataFormat();
    }
  }
  return io_group_config;
}

void NitrosPublisherSubscriberGroup::updateUpstreamSubscriberDownstreamSupportedFormatsCallback(
  const std::map<std::string, negotiated::detail::SupportedTypeInfo> & key_to_supported_types,
  const std::shared_ptr<std::map<
    negotiated::detail::PublisherGid,
    std::vector<std::string>>> & negotiated_subscription_type_gids,
  const std::unordered_set<std::shared_ptr<
    negotiated::detail::UpstreamNegotiatedSubscriptionHandle>> & upstream_negotiated_subscriptions,
  const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_add,
  const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_remove,
  negotiated::detail::PublisherGid gid_key,
  gxf::optimizer::ComponentInfo comp_info)
{
  (void)key_to_supported_types;
  (void)negotiated_subscription_type_gids;

  negotiated_interfaces::msg::SupportedTypes upstream_types_to_add;
  negotiated_interfaces::msg::SupportedTypes upstream_types_to_remove;

  auto nitros_pub = findNitrosPublisher(comp_info);

  for (const std::shared_ptr<negotiated::detail::UpstreamNegotiatedSubscriptionHandle> & handle :
    upstream_negotiated_subscriptions)
  {
    // Map downstream types to the upstream subscriber's types
    auto nitros_sub = findNitrosSubscriber(handle->subscription);

    std::unordered_set<std::string> upstream_format_names_to_add;
    for (auto & downstream_type : downstream_types_to_add.supported_types) {
      auto upstream_formats = mapDownstreamToUpstreamDataFormats(
        comp_info,
        nitros_sub->getComponentInfo(),
        downstream_type.supported_type_name);
      for (auto upstream_format : upstream_formats) {
        if (upstream_format_names_to_add.count(upstream_format) == 0) {
          upstream_format_names_to_add.insert(upstream_format);

          negotiated_interfaces::msg::SupportedType upstream_type_to_add(downstream_type);
          // Supported format name
          upstream_type_to_add.supported_type_name = upstream_format;
          // The corresponding ROS type name
          upstream_type_to_add.ros_type_name =
            nitros_type_manager_->getFormatCallbacks(
            upstream_type_to_add.supported_type_name).getROSTypeName();

          upstream_types_to_add.supported_types.push_back(upstream_type_to_add);
        }
      }
    }

    std::unordered_set<std::string> upstream_format_names_to_remove;
    for (auto & downstream_type : downstream_types_to_remove.supported_types) {
      auto upstream_formats = mapDownstreamToUpstreamDataFormats(
        comp_info,
        nitros_sub->getComponentInfo(),
        downstream_type.supported_type_name);
      for (auto upstream_format : upstream_formats) {
        if (upstream_format_names_to_add.count(upstream_format) == 0) {
          upstream_format_names_to_add.insert(upstream_format);

          negotiated_interfaces::msg::SupportedType upstream_type_to_remove(downstream_type);
          // Supported format name
          upstream_type_to_remove.supported_type_name = upstream_format;
          // The corresponding ROS type name
          upstream_type_to_remove.ros_type_name =
            nitros_type_manager_->getFormatCallbacks(
            upstream_type_to_remove.supported_type_name).getROSTypeName();

          upstream_types_to_remove.supported_types.push_back(upstream_type_to_remove);
        }
      }
    }

    handle->subscription->update_downstream_supported_types(
      upstream_types_to_add,
      upstream_types_to_remove,
      gid_key);
  }
}

std::unordered_set<std::string> NitrosPublisherSubscriberGroup::mapDownstreamToUpstreamDataFormats(
  const gxf::optimizer::ComponentInfo & downstream_comp_info,
  const gxf::optimizer::ComponentInfo & upstream_comp_info,
  const std::string & downstream_data_format) const
{
  std::unordered_set<std::string> upstream_data_formats;
  for (auto & data_format_map : gxf_io_supported_data_formats_info_.supported_data_types) {
    if (data_format_map.at(gxf::optimizer::GenerateComponentKey(downstream_comp_info)) ==
      downstream_data_format)
    {
      upstream_data_formats.insert(
        data_format_map.at(
          gxf::optimizer::GenerateComponentKey(
            upstream_comp_info)));
    }
  }
  return upstream_data_formats;
}

std::unordered_set<std::string> NitrosPublisherSubscriberGroup::mapUpstreamToDownstreamDataFormats(
  const gxf::optimizer::ComponentInfo & upstream_comp_info,
  const gxf::optimizer::ComponentInfo & downstream_comp_info,
  const std::string & upstream_data_format) const
{
  std::unordered_set<std::string> downstream_data_formats;
  for (auto & data_format_map : gxf_io_supported_data_formats_info_.supported_data_types) {
    if (data_format_map.at(gxf::optimizer::GenerateComponentKey(upstream_comp_info)) ==
      upstream_data_format)
    {
      downstream_data_formats.insert(
        data_format_map.at(
          gxf::optimizer::GenerateComponentKey(
            downstream_comp_info)));
    }
  }
  return downstream_data_formats;
}

std::vector<negotiated_interfaces::msg::SupportedType>
NitrosPublisherSubscriberGroup::publisherNegotiationCallback(
  const std::map<negotiated::detail::PublisherGid,
  std::vector<std::string>> & negotiated_sub_gid_to_keys,
  const std::map<std::string,
  negotiated::detail::SupportedTypeInfo> & key_to_supported_types,
  const std::unordered_set<
    std::shared_ptr<negotiated::detail::UpstreamNegotiatedSubscriptionHandle>> &
  upstream_negotiated_subscriptions,
  size_t maximum_solutions,
  gxf::optimizer::ComponentInfo pub_info)
{
  // What the negotiation algorithm does is to try to find the minimum number of publishers with
  // the maximum amount of weight to satisfy all of the subscriptions.  This is approximately
  // equivalent to the Cutting-stock problem (https://en.wikipedia.org/wiki/Cutting_stock_problem).
  // To do this, we examine all combinations at every level (a level being the number of publishers
  // to choose), finding the highest weight one.  If there is at least one at that level, we stop
  // processing.  If there are no solutions at that level, we increment the number of publishers to
  // choose by one and try again at the next level.  If we exhaust all levels, then we have failed
  // to find a match and negotiation fails.
  //
  // Some examples will help illustrate the process.
  //
  // Scenario 1:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that all 3 subscribers support one ros_type/supported_type_name combination,
  //    and that combination (F1) is the same across all 3.
  //   Finally assume that the publisher also supports the same ros_type/supported_type_name
  //    combination (F1)
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    Since all of the subscriptions and the publisher support F1, then a "level 1" solution
  //    exists, and the algorithm chooses that.
  //
  // Scenario 2:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that S1 and S2 support one ros_type/supported_type_name combination (F1), and
  //    S3 supports a different ros_type/supported_type_name combination (F2).
  //   Finally assume that the publisher supports both F1 and F2 ros_type/supported_type_name
  //    combinations.
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    The publisher and S1 and S2 support F1, but S3 does not, so there is no one publisher
  //    solution.  Next the algorithm tries all combinations of 2 publisher solutions.  In this
  //    case we can make 2 publishers, one to satisify F1 and F2, so that algorithm chooses that.
  //
  // Scenario 3:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that S1 and S2 support one ros_type/supported_type_name combination (F1), and
  //    S3 supports a different ros_type/supported_type_name combination (F2).
  //   Finally assume that the publisher supports only the F1 ros_type/supported_type_name
  //     combinations.
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    The publisher and S1 and S2 support F1, but S3 does not, so there is no one publisher
  //    solution.  Next the algorithm tries all combinations of 2 publisher solutions.  Since the
  //    publisher doesn't support F2, there are no 2 publisher solutions.  Finally the algorithm
  //    tries the 3 publisher solution, but since the publisher doesn't support F2 this can't
  //    work either.  So the negotiation fails in this case.

  std::vector<negotiated_interfaces::msg::SupportedType> matched_subs;

  // If there are upstream subscriptions that we should wait on before negotiating with our
  // downstream subscriptions, we'll discover it here.
  std::map<std::string, negotiated::detail::SupportedTypeInfo> upstream_filtered_supported_types;
  if (upstream_negotiated_subscriptions.size() > 0) {
    std::map<ComponentKey, std::string> nitros_sub_negotiated_format_map;  // NVIDIA
    for (const std::shared_ptr<negotiated::detail::UpstreamNegotiatedSubscriptionHandle> & handle :
      upstream_negotiated_subscriptions)
    {
      negotiated_interfaces::msg::NegotiatedTopicsInfo topics_info =
        handle->subscription->get_negotiated_topics_info();
      if (!topics_info.success || topics_info.negotiated_topics.size() == 0) {
        // NVIDIA
        continue;
        // NVIDIA
      }

      // NVIDIA
      auto nitros_sub = findNitrosSubscriber(handle->subscription);
      nitros_sub_negotiated_format_map[GenerateComponentKey(nitros_sub->getComponentInfo())] =
        topics_info.negotiated_topics[0].supported_type_name;
      // NVIDIA
    }

    // NVIDIA
    if (nitros_sub_negotiated_format_map.size() == 0) {
      // No upstream subscriber has done its negotiation.
      // The downstream negotiation still needs to continue with all supported data formats.
      upstream_filtered_supported_types = key_to_supported_types;
    } else {
      // Determine the publisher's current supported formats based on the valid format
      // combinations filtered by the upstream subscriber's negotiated formats
      ComponentKey pub_comp_key = GenerateComponentKey(pub_info);
      for (const auto & supported_data_type_map :
        gxf_io_supported_data_formats_info_.supported_data_types)
      {
        bool is_valid_format_combination = true;
        for (auto & nitros_sub : nitros_subs_) {
          ComponentKey comp_key = GenerateComponentKey(nitros_sub->getComponentInfo());
          std::string negotiated_format = nitros_sub_negotiated_format_map[comp_key];
          if (supported_data_type_map.at(comp_key) != negotiated_format) {
            is_valid_format_combination = false;
            break;
          }
        }
        if (is_valid_format_combination) {
          std::string pub_valid_format = supported_data_type_map.at(pub_comp_key);
          std::string key = negotiated::detail::generate_key(
            nitros_type_manager_->getFormatCallbacks(pub_valid_format).getROSTypeName(),
            pub_valid_format);
          if (key_to_supported_types.count(key) > 0) {
            upstream_filtered_supported_types[key] = key_to_supported_types.at(key);
          }
        }
      }
    }
    // NVIDIA

  } else {
    upstream_filtered_supported_types = key_to_supported_types;
  }

  std::set<negotiated::detail::PublisherGid> gid_set;
  for (const std::pair<negotiated::detail::PublisherGid,
    std::vector<std::string>> & gid : negotiated_sub_gid_to_keys)
  {
    gid_set.insert(gid.first);
  }

  std::vector<std::string> keys;
  std::vector<negotiated::detail::SupportedTypeInfo> compatible_supported_types;
  for (const std::pair<const std::string,
    negotiated::detail::SupportedTypeInfo> & supported_info : upstream_filtered_supported_types)
  {
    keys.push_back(supported_info.first);
    if (supported_info.second.is_compat) {
      compatible_supported_types.push_back(supported_info.second);
    }
  }

  for (size_t i = 1; i <= upstream_filtered_supported_types.size(); ++i) {
    double max_weight = 0.0;

    auto check_combination = [
      & upstream_filtered_supported_types = std::as_const(upstream_filtered_supported_types),
      & gid_set = std::as_const(gid_set),
      & compatible_supported_types = std::as_const(compatible_supported_types),
      & negotiated_sub_gid_to_keys = std::as_const(negotiated_sub_gid_to_keys),
      &max_weight,
      &matched_subs](
      std::vector<std::string>::iterator first,
      std::vector<std::string>::iterator last) -> bool
      {
        std::set<negotiated::detail::PublisherGid> gids_needed = gid_set;

        double sum_of_weights = 0.0;

        for (std::vector<std::string>::iterator it = first; it != last; ++it) {
          // The iterator should *always* be available in the upstream_filtered_supported_types
          // map, since we are iterating over that same map.  But we use .at just
          // to be safe.
          negotiated::detail::SupportedTypeInfo
            supported_type_info = upstream_filtered_supported_types.at(*it);

          for (const std::pair<negotiated::detail::PublisherGid,
            double> gid_to_weight : supported_type_info.gid_to_weight)
          {
            sum_of_weights += gid_to_weight.second;

            gids_needed.erase(gid_to_weight.first);
          }
        }

        std::vector<negotiated_interfaces::msg::SupportedType> compatible_subs;
        if (!compatible_supported_types.empty()) {
          // We've removed all of the ones we could above in this iteration.  Now we go through
          // the remaining list of GIDs, seeing if any of the "compatible" supported types satisfy
          // the requirements.
          for (std::set<negotiated::detail::PublisherGid>::const_iterator it = gids_needed.begin();
            it != gids_needed.end(); )
          {
            const std::vector<std::string> key_list = negotiated_sub_gid_to_keys.at(*it);

            bool found_key = false;
            for (const negotiated::detail::SupportedTypeInfo & compat_info :
              compatible_supported_types)
            {
              std::string key = negotiated::detail::generate_key(
                compat_info.ros_type_name,
                compat_info.supported_type_name);
              if (std::find(key_list.begin(), key_list.end(), key) != key_list.end()) {
                negotiated_interfaces::msg::SupportedType match;
                match.ros_type_name = compat_info.ros_type_name;
                match.supported_type_name = compat_info.supported_type_name;
                compatible_subs.push_back(match);

                found_key = true;
                break;
              }
            }

            if (found_key) {
              it = gids_needed.erase(it);
            } else {
              ++it;
            }
          }
        }

        if (gids_needed.empty()) {
          // Hooray!  We found a solution at this level.  We don't interrupt processing at this
          // level because there may be another combination that is more favorable, but we know
          // we don't need to descend to further levels.

          if (sum_of_weights > max_weight) {
            max_weight = sum_of_weights;

            matched_subs.clear();
            matched_subs = compatible_subs;
            for (std::vector<std::string>::iterator it = first; it != last; ++it) {
              negotiated::detail::SupportedTypeInfo supported_type_info =
                upstream_filtered_supported_types.at(*it);
              negotiated_interfaces::msg::SupportedType match;
              match.ros_type_name = supported_type_info.ros_type_name;
              match.supported_type_name = supported_type_info.supported_type_name;
              matched_subs.push_back(match);
            }
          }
        }

        return false;
      };

    for_each_combination(keys.begin(), keys.begin() + i, keys.end(), check_combination);

    if (!matched_subs.empty()) {
      break;
    }

    if (i == maximum_solutions) {
      break;
    }
  }

  RCLCPP_DEBUG(
    node_.get_logger(),
    "[NitrosPublisherSubscriberGroup] Returned matched_subs: %ld",
    matched_subs.size());

  return matched_subs;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
