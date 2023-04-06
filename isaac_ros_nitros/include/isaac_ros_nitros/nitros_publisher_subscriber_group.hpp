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

#ifndef ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_GROUP_HPP_
#define ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_GROUP_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "extensions/gxf_optimizer/exporter/graph_types.hpp"

#include "isaac_ros_nitros/nitros_publisher.hpp"
#include "isaac_ros_nitros/nitros_subscriber.hpp"

#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

using gxf::optimizer::ComponentKey;

// A pub/sub group that consists of associated Nitros publishers and subscribers
class NitrosPublisherSubscriberGroup
{
public:
  // Constructor
  NitrosPublisherSubscriberGroup(
    rclcpp::Node & node,
    const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::GraphIOGroupSupportedDataTypesInfo & gxf_io_supported_data_formats_info,
    const NitrosPublisherSubscriberConfigMap & nitros_pub_sub_configs,
    const std::shared_ptr<std::map<ComponentKey, std::string>> frame_id_map_ptr,
    const NitrosStatisticsConfig & statistics_config);

  // Find the corresponding Nitros publisher of the given component
  std::shared_ptr<NitrosPublisher> findNitrosPublisher(
    const gxf::optimizer::ComponentInfo & comp_info);

  // Find the corresponding Nitros subscriber of the given component
  std::shared_ptr<NitrosSubscriber> findNitrosSubscriber(
    const gxf::optimizer::ComponentInfo & comp_info);

  // Find the Nitros subscriber that holds the given negotiated subscriber
  std::shared_ptr<NitrosSubscriber> findNitrosSubscriber(
    const std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub);

  // Start negotiation
  void start();

  // Get negotiated configurations
  gxf::optimizer::GraphIOGroupDataTypeConfigurations getDataFormatConfigurations();

  // Negotiated publisher's callback for informing its upstream subscriber changes in its
  // supported data formats when receiving an update from its downstream negotiated subscribers
  void updateUpstreamSubscriberDownstreamSupportedFormatsCallback(
    const std::map<std::string, negotiated::detail::SupportedTypeInfo> & key_to_supported_types,
    const std::shared_ptr<std::map<negotiated::detail::PublisherGid,
    std::vector<std::string>>> & negotiated_subscription_type_gids,
    const std::unordered_set<
      std::shared_ptr<
        negotiated::detail::UpstreamNegotiatedSubscriptionHandle>> &
    upstream_negotiated_subscriptions,
    const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_add,
    const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_remove,
    negotiated::detail::PublisherGid gid_key,
    gxf::optimizer::ComponentInfo comp_info);

  // Get the upstream component's supported data formats mapped from the given data format of
  // the given downstream component
  std::unordered_set<std::string> mapDownstreamToUpstreamDataFormats(
    const gxf::optimizer::ComponentInfo & downstream_comp_info,
    const gxf::optimizer::ComponentInfo & upstream_comp_info,
    const std::string & downstream_data_format) const;

  // Get the downstream component's supported data format mapped from the given data format of
  // the given upstream component
  std::unordered_set<std::string> mapUpstreamToDownstreamDataFormats(
    const gxf::optimizer::ComponentInfo & upstream_comp_info,
    const gxf::optimizer::ComponentInfo & downstream_comp_info,
    const std::string & upstream_data_format) const;

  // Negotiated publisher's callback invoked for running a negotiation algorithm
  std::vector<negotiated_interfaces::msg::SupportedType> publisherNegotiationCallback(
    const std::map<negotiated::detail::PublisherGid,
    std::vector<std::string>> & negotiated_sub_gid_to_keys,
    const std::map<std::string, negotiated::detail::SupportedTypeInfo> & key_to_supported_types,
    const std::unordered_set<
      std::shared_ptr<
        negotiated::detail::UpstreamNegotiatedSubscriptionHandle>> &
    upstream_negotiated_subscriptions,
    size_t maximum_solutions,
    gxf::optimizer::ComponentInfo pub_info);

  // Getter for all the Nitros publishers
  std::vector<std::shared_ptr<NitrosPublisher>> getNitrosPublishers();

  // Getter for all the Nitros subscribers
  std::vector<std::shared_ptr<NitrosSubscriber>> getNitrosSubscribers();

  // Get all publisher/subscriber component infos
  std::vector<gxf::optimizer::ComponentInfo> getAllComponentInfos() const;

  // Expand "any" format in gxf_io_supported_data_formats_info_ with registered formats
  void expandAnyDataFormats();

  // For each component that has .use_compatible_format_only as true, remove all supported
  // formats except for the compatible format
  bool applyUseCompatibleFormatOnly();

  // Validate if the final data format combination is valid or not.
  // In the case that some pubs/subs have sucessful negotiation and some don't (and hence
  // compatible formats are chosen for these pubs/subs), the resulting group format combaination
  // may not be valid. In such a case, those pubs/subs that have use_flexible_compatible_format
  // set as true will have their compatible formats adjusted to form a valid format combination
  // for the group.
  void postNegotiationAdjustCompatibleFormats();

  // To be called after negotiation timer is up
  void postNegotiationCallback();

private:
  // Create all Nitros subscribers in this group based on information provided in group_info_
  void createNitrosSubscribers();

  // Create all Nitros publishers in this group based on information provided in group_info_
  // This must be called after Nitros subscribers are created (by calling createNitrosSubscribers)
  // to correctly set the upstream subscribers for each publisher.
  void createNitrosPublishers();

  // The ROS node that holds this group
  rclcpp::Node & node_;

  // The parent GXF context
  gxf_context_t context_;

  // Nitros type manager
  std::shared_ptr<NitrosTypeManager> nitros_type_manager_;

  // GXF graph information
  gxf::optimizer::GraphIOGroupSupportedDataTypesInfo gxf_io_supported_data_formats_info_;

  // Configurations for all Nitros publishers/subscribers
  NitrosPublisherSubscriberConfigMap nitros_pub_sub_configs_;

  // Nitros publishers and subscribers created in this group
  std::vector<std::shared_ptr<NitrosPublisher>> nitros_pubs_;
  std::vector<std::shared_ptr<NitrosSubscriber>> nitros_subs_;

  // Frame ID map
  std::shared_ptr<std::map<ComponentKey, std::string>> frame_id_map_ptr_;

  // Configurations for a Nitros statistics
  NitrosStatisticsConfig statistics_config_;
};


}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_GROUP_HPP_
