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
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "extensions/messages/helpers.hpp"
#include "extensions/messages/composite_message.hpp"
#include "gems/control_types/differential_drive.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_twist_type/nitros_twist.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr int kLinearSpeedIndx = 0;
constexpr int kAngularSpeedIndx = 1;
constexpr int DifferentialBaseCommandIndices = 2;
}  // namespace

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosTwist,
  geometry_msgs::msg::Twist>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosTwist::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTwist"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  // Create composite msg and populate values
  auto maybe_composite_message = nvidia::isaac::GetCompositeMessage(msg_entity.value());
  if (!maybe_composite_message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get maybe_composite_message gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_composite_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTwist"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto composite_message = maybe_composite_message.value();

  // Populate linear and angular speeds of the ROS msg from gxf msg
  destination.linear.x = composite_message.view(0, kLinearSpeedIndx);
  destination.angular.z = composite_message.view(0, kAngularSpeedIndx);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTwist"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosTwist,
  geometry_msgs::msg::Twist>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosTwist::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTwist"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // Get pointer to allocator component
  const std::string kAllocatorEntityName = "memory_pool";
  const std::string kAllocatorComponentName = "unbounded_allocator";
  const std::string kAllocatorComponentTypeName = "nvidia::gxf::UnboundedAllocator";
  gxf_uid_t allocator_cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kAllocatorEntityName, kAllocatorComponentName, kAllocatorComponentTypeName, allocator_cid);
  auto maybe_allocator_handle =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, allocator_cid);
  if (!maybe_allocator_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get allocator's handle: " <<
      GxfResultStr(maybe_allocator_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosAprilTagDetectionArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  // Create composite msg and populate values
  auto maybe_composite_message = nvidia::isaac::CreateCompositeMessage(
    context, allocator_handle, 1, DifferentialBaseCommandIndices);
  if (!maybe_composite_message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get maybe_composite_message gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_composite_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTwist"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto composite_message = maybe_composite_message.value();

  // Get pointer to composite_schema_server component
  const std::string kSchemaServerEntityName = "global_pose_tree";
  const std::string kSchemaServerComponentName = "composite_schema_server";
  const std::string kSchemaServerComponentTypeName = "nvidia::isaac::CompositeSchemaServer";
  gxf_uid_t schema_server_cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kSchemaServerEntityName, kSchemaServerComponentName,
    kSchemaServerComponentTypeName, schema_server_cid);
  auto maybe_schema_server_handle =
    nvidia::gxf::Handle<nvidia::isaac::CompositeSchemaServer>::Create(context, schema_server_cid);
  if (!maybe_schema_server_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get maybe_schema_server_handle gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_schema_server_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTwist"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto schema_server_handle = maybe_schema_server_handle.value();

  // Add schema to server if it doesn't already exist
  auto maybe_schema_uid = schema_server_handle->add(
    nvidia::isaac::DifferentialBaseCommandCompositeSchema());
  if (!maybe_schema_uid) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Cannot add schema to server" <<
      GxfResultStr(maybe_composite_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTwist"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  // Set schema uid for the gxf msg
  composite_message.composite_schema_uid->uid = maybe_schema_uid.value();

  // Populate linear and angular speeds of the gxf msg from ROS msg
  composite_message.view(0, kLinearSpeedIndx) = source.linear.x;
  composite_message.view(0, kAngularSpeedIndx) = source.angular.z;

  // Set Entity Id
  destination.handle = composite_message.message.eid();
  GxfEntityRefCountInc(context, composite_message.message.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTwist"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
