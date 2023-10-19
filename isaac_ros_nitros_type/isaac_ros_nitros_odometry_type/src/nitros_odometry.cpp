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
#include "gems/pose_tree/pose_tree.hpp"
#include "extensions/atlas/pose_tree_frame.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_odometry_type/nitros_odometry.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace
{
constexpr int kPositionXIndx = 0;
constexpr int kPositionYIndx = 1;
constexpr int kHeadingIndx = 2;
constexpr int kLinearSpeedIndx = 3;
constexpr int kAngularSpeedIndx = 4;
constexpr int DifferentialBaseEgoMotionIndices = 7;
constexpr char kPoseTreeEntityName[] = "global_pose_tree";
constexpr char kPoseTreeComponentName[] = "pose_tree";
constexpr char kPoseTreeComponentTypeName[] = "nvidia::isaac::PoseTree";
}  // namespace

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosOdometry,
  nav_msgs::msg::Odometry>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosOdometry::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosOdometry"),
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
      rclcpp::get_logger("NitrosOdometry"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto composite_message = maybe_composite_message.value();

  // Populate odometry data of the ROS msg from gxf msg
  destination.pose.pose.position.x = composite_message.view(0, kPositionXIndx);
  destination.pose.pose.position.y = composite_message.view(0, kPositionYIndx);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, composite_message.view(0, kHeadingIndx));
  destination.pose.pose.orientation.x = quaternion.getX();
  destination.pose.pose.orientation.y = quaternion.getY();
  destination.pose.pose.orientation.z = quaternion.getZ();
  destination.pose.pose.orientation.w = quaternion.getW();
  destination.twist.twist.linear.x = composite_message.view(0, kLinearSpeedIndx);
  destination.twist.twist.angular.z = composite_message.view(0, kAngularSpeedIndx);

  // Populate timestamp information back into ROS header
  auto input_timestamp = composite_message.timestamp;
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp->acqtime % static_cast<uint64_t>(1e9));
  }

  // Get pointer to posetree component
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kPoseTreeEntityName, kPoseTreeComponentName, kPoseTreeComponentTypeName, cid);
  auto maybe_pose_tree_handle =
    nvidia::gxf::Handle<nvidia::isaac::PoseTree>::Create(context, cid);
  if (!maybe_pose_tree_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get pose tree's handle: " <<
      GxfResultStr(maybe_pose_tree_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOdometry"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();
  auto frame_name = pose_tree_handle->getFrameName(composite_message.pose_frame_uid->uid);
  if (frame_name) {
    destination.header.frame_id = frame_name.value();
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosOdometry"), "Setting frame if from NITROS msg");
    // Set NITROS frame id as fallback method of populating frame_id
    // Set frame ID
    destination.header.frame_id = source.frame_id;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosOdometry"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosOdometry,
  nav_msgs::msg::Odometry>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosOdometry::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosOdometry"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // Get pointer to allocator component
  const std::string kAllocatorEntityName = "memory_pool";
  const std::string kAllocatorComponentName = "unbounded_allocator";
  const std::string kAllocatorComponentTypeName = "nvidia::gxf::UnboundedAllocator";
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kAllocatorEntityName, kAllocatorComponentName, kAllocatorComponentTypeName, cid);
  auto maybe_allocator_handle =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, cid);
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
    context, allocator_handle, 1, DifferentialBaseEgoMotionIndices);
  if (!maybe_composite_message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get maybe_composite_message gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_composite_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOdometry"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto composite_message = maybe_composite_message.value();

  // Get pointer to composite_schema_server component
  const std::string kSchemaServerEntityName = "global_pose_tree";
  const std::string kSchemaServerComponentName = "composite_schema_server";
  const std::string kSchemaServerComponentTypeName = "nvidia::isaac::CompositeSchemaServer";
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kSchemaServerEntityName, kSchemaServerComponentName,
    kSchemaServerComponentTypeName, cid);
  auto maybe_schema_server_handle =
    nvidia::gxf::Handle<nvidia::isaac::CompositeSchemaServer>::Create(context, cid);
  if (!maybe_schema_server_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get maybe_schema_server_handle gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_schema_server_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOdometry"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto schema_server_handle = maybe_schema_server_handle.value();

  // Add schema to server if it doesn't already exist
  auto maybe_schema_uid = schema_server_handle->add(
    nvidia::isaac::DifferentialBaseEgoMotionCompositeSchema());
  if (!maybe_schema_uid) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Cannot add schema to server" <<
      GxfResultStr(maybe_composite_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOdometry"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  // Set schema uid for the gxf msg
  composite_message.composite_schema_uid->uid = maybe_schema_uid.value();

  // Populate odometry data of the gxf msg from ROS msg
  composite_message.view(0, kPositionXIndx) = source.pose.pose.position.x;
  composite_message.view(0, kPositionYIndx) = source.pose.pose.position.y;
  composite_message.view(0, kLinearSpeedIndx) = source.twist.twist.linear.x;
  composite_message.view(0, kAngularSpeedIndx) = source.twist.twist.angular.z;
  tf2::Quaternion q(
    source.pose.pose.orientation.x,
    source.pose.pose.orientation.y,
    source.pose.pose.orientation.z,
    source.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  composite_message.view(0, kHeadingIndx) = yaw;

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  composite_message.timestamp->acqtime = input_timestamp;

  // Get pointer to posetree component
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kPoseTreeEntityName, kPoseTreeComponentName, kPoseTreeComponentTypeName, cid);
  auto maybe_pose_tree_handle =
    nvidia::gxf::Handle<nvidia::isaac::PoseTree>::Create(context, cid);
  if (!maybe_pose_tree_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get pose tree's handle: " <<
      GxfResultStr(maybe_pose_tree_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOdometry"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();
  auto maybe_composite_msg_frame_uid = pose_tree_handle->findOrCreateFrame(
    source.header.frame_id.c_str());
  if (maybe_composite_msg_frame_uid) {
    composite_message.pose_frame_uid->uid = maybe_composite_msg_frame_uid.value();
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("NitrosOdometry"), "Could not create Pose Tree Frame");
  }

  // Set Entity Id
  destination.handle = composite_message.message.eid();
  GxfEntityRefCountInc(context, composite_message.message.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosOdometry"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
