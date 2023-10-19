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

#ifndef ISAAC_ROS_NITROS__NITROS_PUBLISHER_HPP_
#define ISAAC_ROS_NITROS__NITROS_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "message_compositor/message_relay.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "gxf/std/vault.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/nitros_publisher_subscriber_base.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "negotiated/negotiated_publisher.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "std_msgs/msg/header.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Forward declaration
class NitrosPublisher;


// A waitable for publishing messages triggered from GXF graph
class NitrosPublisherWaitable : public rclcpp::Waitable
{
public:
  NitrosPublisherWaitable(
    rclcpp::Node & node,
    NitrosPublisher & nitros_publisher);

  size_t get_number_of_ready_guard_conditions()
  {
    return 1;
  }

  // Trigger guard_condition_
  void trigger();

  std::shared_ptr<void> take_data() override
  {
    return nullptr;
  }

  bool is_ready(rcl_wait_set_t * wait_set) override;
  void execute(std::shared_ptr<void> & data) override;
  void add_to_wait_set(rcl_wait_set_t * wait_set) override;

private:
  rclcpp::Node & node_;
  NitrosPublisher & nitros_publisher_;
  std::mutex guard_condition_mutex_;
  rclcpp::GuardCondition guard_condition_;
};


// Nitros publisher that supports type adaptation/negotiation and enables hardware acceleration
class NitrosPublisher : public NitrosPublisherSubscriberBase
{
public:
  // Constructor
  NitrosPublisher(
    rclcpp::Node & node,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config,
    const negotiated::NegotiatedPublisherOptions & negotiated_pub_options);

  NitrosPublisher(
    rclcpp::Node & node,
    const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config,
    const negotiated::NegotiatedPublisherOptions & negotiated_pub_options);

  NitrosPublisher(
    rclcpp::Node & node,
    const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config,
    const negotiated::NegotiatedPublisherOptions & negotiated_pub_options,
    const NitrosStatisticsConfig & statistics_config);

  // Constructor for creating a publisher without an associated gxf egress port
  NitrosPublisher(
    rclcpp::Node & node,
    const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config,
    const NitrosStatisticsConfig & statistics_config);

  // Getter for the negotiated_pub_
  std::shared_ptr<negotiated::NegotiatedPublisher> getNegotiatedPublisher();

  // Add a supported data format to the underlying negotiated publisher
  void addSupportedDataFormat(
    const std::string & data_format,
    const double weight);

  // Start negotiation
  void start();

  // Create a compatible publisher
  void createCompatiblePublisher();

  // To be called after negotiation timer is up
  void postNegotiationCallback();

  // Getter of gxf_message_relay_callback_func_
  std::function<void(void)> & getGxfMessageRelayCallbackFunc();

  // Setter for gxf_vault_ptr_ pointing to a vault component in the running graph
  void setVaultPointer(void * gxf_vault_ptr);

  // Setter for gxf_vault_ptr_ pointing to a message relay component in the running graph
  void setMessageRelayPointer(void * gxf_message_relay_ptr);

  // Start a timer for polling data queued in the vault component in the running graph
  void startGxfVaultPeriodicPollingTimer();

  void enableNitrosPublisherWaitable();

  // Publish the given handle for the negotiated format and compatible format
  void publish(const int64_t handle);

  // Publish the given handle for the negotiated format and compatible format while
  // revising the enclosed timestamp components, if any, with the time set in the
  // provided ROS header.
  // This is mainly for benchmarking/testing purposes.
  void publish(const int64_t handle, const std_msgs::msg::Header & ros_header);

  // Publish the given Nitros-typed message for the negotiated format and compatible format
  // while revising the enclosed timestamp components, if any, with the time set in the
  // provided ROS header.
  // This is mainly for benchmarking/testing purposes.
  void publish(NitrosTypeBase & base_msg, const std_msgs::msg::Header & ros_header);

  // Publish the given Nitros-typed message for the negotiated format and compatible format
  void publish(NitrosTypeBase & base_msg);

  // Extract message entities form Vault or MessageRelay in the running graph
  void extractMessagesFromGXF();

private:
  // The callback function that is invoked when a MessageRelay component receives
  // a message in the GXF graph.
  void gxfMessageRelayCallback();

  // The vault data polling timer
  rclcpp::TimerBase::SharedPtr gxf_vault_periodic_polling_timer_;

  // Negotiated publisher
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;

  // A publisher for publishing data to the base topic
  std::shared_ptr<rclcpp::PublisherBase> compatible_pub_{nullptr};

  // A pointer to the associated vault component for retrieving data from the running graph
  nvidia::gxf::Vault * gxf_vault_ptr_{nullptr};

  // A pointer to the associated MessageRelay component for retrieving data from the running graph
  nvidia::isaac_ros::MessageRelay * gxf_message_relay_ptr_{nullptr};

  // The function pointer that is used by a MessageRelay component to call
  // the actual callback function (gxfMessageRelayCallback)
  std::function<void(void)> gxf_message_relay_callback_func_;

  // NitrosPublisherWaitable waitable_;
  // rcl_guard_condition_t guard_condition_;
  std::shared_ptr<NitrosPublisherWaitable> waitable_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_PUBLISHER_HPP_
