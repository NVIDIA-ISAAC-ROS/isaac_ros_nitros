// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "extensions/messages/encoder_tick_message.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_encoder_ticks_type/nitros_encoder_ticks.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosEncoderTicks,
  isaac_ros_nova_interfaces::msg::EncoderTicks>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosEncoderTicks::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosEncoderTicks"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto maybe_encoder_ticks_parts = nvidia::isaac::GetEncoderTickMessage(
    msg_entity.value());
  if (!maybe_encoder_ticks_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get battery state gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_encoder_ticks_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosEncoderTicks"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto encoder_ticks_parts = maybe_encoder_ticks_parts.value();

  destination.left_ticks = *encoder_ticks_parts.left_ticks.get();
  destination.right_ticks = *encoder_ticks_parts.right_ticks.get();
  destination.encoder_timestamp = *encoder_ticks_parts.encoder_timestamp.get();

  // Populate timestamp information back into ROS header
  auto input_timestamp = encoder_ticks_parts.timestamp;
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp->acqtime % static_cast<uint64_t>(1e9));
  }

  // Set frame ID
  destination.header.frame_id = source.frame_id;
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosEncoderTicks"),
    "[convert_to_ros_message] frame_id = %s", destination.header.frame_id.c_str());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosEncoderTicks"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosEncoderTicks,
  isaac_ros_nova_interfaces::msg::EncoderTicks>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosEncoderTicks::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosEncoderTicks"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // create encoder_ticks gxf msg
  auto maybe_encoder_ticks_parts = nvidia::isaac::CreateEncoderTickMessage(context);
  if (!maybe_encoder_ticks_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to create encoder ticks gxf msg " << GxfResultStr(
      maybe_encoder_ticks_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosEncoderTicks"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto encoder_ticks_parts = maybe_encoder_ticks_parts.value();

  *encoder_ticks_parts.left_ticks.get() = source.left_ticks;
  *encoder_ticks_parts.right_ticks.get() = source.right_ticks;
  *encoder_ticks_parts.encoder_timestamp.get() = source.encoder_timestamp;

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  encoder_ticks_parts.timestamp->acqtime = input_timestamp;

  // Set frame ID
  destination.frame_id = source.header.frame_id;
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosEncoderTicks"),
    "[convert_to_custom] frame_id = %s", destination.frame_id.c_str());

  // Set Entity Id
  destination.handle = encoder_ticks_parts.message.eid();
  GxfEntityRefCountInc(context, encoder_ticks_parts.message.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosEncoderTicks"),
    "[convert_to_custom] Conversion completed (handle=%ld)", destination.handle);

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
