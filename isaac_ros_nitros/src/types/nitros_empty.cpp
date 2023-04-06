/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/types/nitros_empty.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosEmpty,
  std_msgs::msg::Empty>::convert_to_ros_message(
  const custom_type & source,
  ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosEmpty::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosEmpty"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  (void)source;
  (void)destination;
  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto gxf_message = nvidia::gxf::Entity::Shared(context, source.handle);
  if (!gxf_message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Error getting message entity: " <<
      GxfResultStr(gxf_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosEmpty"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Nothing to populate for empty message
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosEmpty,
  std_msgs::msg::Empty>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosEmpty::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosEmpty"),
    "[convert_to_custom] Conversion started");

  (void)source;

  // Create an entity for storing the actual data in the context
  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto gxf_message = nvidia::gxf::Entity::New(context);
  if (!gxf_message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error initializing new message entity: " <<
      GxfResultStr(gxf_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosEmpty"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // No payload to transfer
  // Set the entity's pointer (EID) in a Nitros type data struct
  destination.handle = gxf_message->eid();
  destination.data_format_name = "nitros_empty";

  // Increase the reference count for the created entity so it doesn't get destroyed after
  // exiting this function
  GxfEntityRefCountInc(context, gxf_message->eid());

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
