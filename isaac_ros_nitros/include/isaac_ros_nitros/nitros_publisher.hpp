/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__NITROS_PUBLISHER_HPP_
#define ISAAC_ROS_NITROS__NITROS_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "gxf/std/vault.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/nitros_publisher_subscriber_base.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "negotiated/negotiated_publisher.hpp"
#include "std_msgs/msg/header.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

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

  // Setter for gxf_vault_ptr_ pointing to a vault component in the running graph
  void setVaultPointer(void * gxf_vault_ptr);

  // Start a timer for polling data queued in the vault component in the running graph
  void startGxfVaultPeriodicPollingTimer();

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

private:
  // The callback function for polling data form the vault component in the running graph
  void gxfVaultPeriodicPollingCallback();

  // The vault data polling timer
  rclcpp::TimerBase::SharedPtr gxf_vault_periodic_polling_timer_;

  // Negotiated publisher
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;

  // A publisher for publishing data to the base topic
  std::shared_ptr<rclcpp::PublisherBase> compatible_pub_{nullptr};

  // A pointer to the associated vault component for retrieving data from the running graph
  nvidia::gxf::Vault * gxf_vault_ptr_ = nullptr;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_PUBLISHER_HPP_
