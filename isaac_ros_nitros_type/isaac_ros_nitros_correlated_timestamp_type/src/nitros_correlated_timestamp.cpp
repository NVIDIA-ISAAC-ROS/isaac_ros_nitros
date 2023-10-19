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
#include "correlated_timestamp_message.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_correlated_timestamp_type/nitros_correlated_timestamp.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosCorrelatedTimestamp,
  isaac_ros_nova_interfaces::msg::CorrelatedTimestamp>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosCorrelatedTimestamp::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosCorrelatedTimestamp"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto maybe_correlated_timestamp_parts = nvidia::isaac::GetCorrelatedTimestampMessage(
    msg_entity.value());
  if (!maybe_correlated_timestamp_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get correlated_timestamp gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_correlated_timestamp_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosCorrelatedTimestamp"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto correlated_timestamp_parts = maybe_correlated_timestamp_parts.value();
  destination.phc_val = correlated_timestamp_parts.correlated_timestamps->phc_val;
  destination.tsc_val = correlated_timestamp_parts.correlated_timestamps->tsc_val;
  destination.phc2_val = correlated_timestamp_parts.correlated_timestamps->phc2_val;
  destination.sys_val = correlated_timestamp_parts.correlated_timestamps->sys_val;
  destination.phc_latency = correlated_timestamp_parts.correlated_timestamps->phc_latency;

  // Populate timestamp information back into ROS header
  auto input_timestamp = correlated_timestamp_parts.timestamp;
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp->acqtime % static_cast<uint64_t>(1e9));
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosCorrelatedTimestamp"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosCorrelatedTimestamp,
  isaac_ros_nova_interfaces::msg::CorrelatedTimestamp>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosCorrelatedTimestamp::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosCorrelatedTimestamp"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // // Get pointer to allocator component
  // gxf_uid_t cid;
  // nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
  //   kEntityName, kComponentName, kComponentTypeName, cid);
  // auto maybe_allocator_handle =
  //   nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, cid);
  // if (!maybe_allocator_handle) {
  //   std::stringstream error_msg;
  //   error_msg <<
  //     "[convert_to_custom] Failed to get allocator's handle: " <<
  //     GxfResultStr(maybe_allocator_handle.error());
  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("NitrosCorrelatedTimestamp"), error_msg.str().c_str());
  //   throw std::runtime_error(error_msg.str().c_str());
  // }
  // auto allocator_handle = maybe_allocator_handle.value();

  // create correlated_timestamp gxf msg
  auto maybe_correlated_timestamp_parts = nvidia::isaac::CreateCorrelatedTimestampMessage(context);
  if (!maybe_correlated_timestamp_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_gxf_message] Failed to create correlated_timestamp gxf msg " << GxfResultStr(
      maybe_correlated_timestamp_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosCorrelatedTimestamp"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto correlated_timestamp_parts = maybe_correlated_timestamp_parts.value();

  // Add correlated_timestamp data
  correlated_timestamp_parts.correlated_timestamps->phc_val = source.phc_val;
  correlated_timestamp_parts.correlated_timestamps->tsc_val = source.tsc_val;
  correlated_timestamp_parts.correlated_timestamps->phc2_val = source.phc2_val;
  correlated_timestamp_parts.correlated_timestamps->sys_val = source.sys_val;
  correlated_timestamp_parts.correlated_timestamps->phc_latency = source.phc_latency;
  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  correlated_timestamp_parts.timestamp->acqtime = input_timestamp;

  // Set Entity Id
  destination.handle = correlated_timestamp_parts.entity.eid();
  GxfEntityRefCountInc(context, correlated_timestamp_parts.entity.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosCorrelatedTimestamp"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
