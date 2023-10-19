// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS__NITROS_SUBSCRIBER_HPP_
#define ISAAC_ROS_NITROS__NITROS_SUBSCRIBER_HPP_

#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "gxf/std/receiver.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/nitros_publisher_subscriber_base.hpp"

#include "negotiated/negotiated_subscription.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Nitros subscriber that supports type adaptation/negotiation and enables hardware acceleration
class NitrosSubscriber : public NitrosPublisherSubscriberBase
{
public:
  // Constructor
  NitrosSubscriber(
    rclcpp::Node & node,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config);

  NitrosSubscriber(
    rclcpp::Node & node,
    const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config);

  NitrosSubscriber(
    rclcpp::Node & node,
    const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config,
    const NitrosStatisticsConfig & statistics_config);

  // Constructor for creating a subscriber without an associated gxf ingress port
  NitrosSubscriber(
    rclcpp::Node & node,
    const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config,
    const NitrosStatisticsConfig & statistics_config);

  // Getter for the negotiated_sub_
  std::shared_ptr<negotiated::NegotiatedSubscription> getNegotiatedSubscriber();

  // Add a supported data format to the underlying negotiated subscriber
  void addSupportedDataFormat(
    const std::string & data_format,
    const double weight);

  // Start negotiation
  void start();

  // Create a compatible subscriber
  void createCompatibleSubscriber();

  // To be called after negotiation timer is up
  void postNegotiationCallback();

  // Set the gxf_receiver_ptr_ pointing to a receiver component in the running graph
  void setReceiverPointer(void * gxf_receiver_ptr);

  // Push an entity into the corresponding receiver in the underlying graph
  bool pushEntity(const int64_t eid);

  // The subscriber callback
  void subscriberCallback(
    NitrosTypeBase & msg_base,
    const std::string data_format_name);

private:
  // Only either of the following two subscribers will be active after negotiation at runtime

  // A negotiated subscriber for receiving data from a negotiated topic channel
  std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub_;

  // A subscriber for receiving data from the base topic
  std::shared_ptr<rclcpp::SubscriptionBase> compatible_sub_{nullptr};

  // A flag to specifiy if this subscriber is associated with a gxf receiver
  bool use_gxf_receiver_{true};

  // A pinter to the associated receiver component for sending data into the running graph
  nvidia::gxf::Receiver * gxf_receiver_ptr_{nullptr};
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_SUBSCRIBER_HPP_
