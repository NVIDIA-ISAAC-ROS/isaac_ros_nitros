/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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

  // A pinter to the associated receiver component for sending data into the running graph
  nvidia::gxf::Receiver * gxf_receiver_ptr_ = nullptr;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_SUBSCRIBER_HPP_
