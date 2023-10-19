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
#include "gems/pose_tree/pose_tree.hpp"
#include "gxf/std/allocator.hpp"
#include "imu_message.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_imu_type/nitros_imu.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";
constexpr char kPoseTreeEntityName[] = "global_pose_tree";
constexpr char kPoseTreeComponentName[] = "pose_tree";
constexpr char kPoseTreeComponentTypeName[] = "nvidia::isaac::PoseTree";
constexpr int kCovarianceArrSize = 9;
const std::array<double, kCovarianceArrSize> invalidCovMat = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
const std::array<double, kCovarianceArrSize> emptyCovMat = {0, 0, 0, 0, 0, 0, 0, 0, 0};
}  // namespace

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosImu,
  sensor_msgs::msg::Imu>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosImu::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosImu"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto maybe_imu_parts = nvidia::isaac::GetImuMessage(
    msg_entity.value());
  if (!maybe_imu_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get imu gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_imu_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImu"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto imu_parts = maybe_imu_parts.value();
  destination.angular_velocity.x = imu_parts.imu->angular_velocity_x;
  destination.angular_velocity.y = imu_parts.imu->angular_velocity_y;
  destination.angular_velocity.z = imu_parts.imu->angular_velocity_z;
  destination.linear_acceleration.x = imu_parts.imu->linear_acceleration_x;
  destination.linear_acceleration.y = imu_parts.imu->linear_acceleration_y;
  destination.linear_acceleration.z = imu_parts.imu->linear_acceleration_z;

  // https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg
  // Ang Vel and Lin Accel covariances set to 0.0 since data is unavailable in gxf
  destination.angular_velocity_covariance = emptyCovMat;
  destination.linear_acceleration_covariance = emptyCovMat;
  // Orientation covariances set to -1 since orientation data not populated
  destination.orientation_covariance = invalidCovMat;

  // Populate timestamp information back into ROS header
  auto input_timestamp = imu_parts.timestamp;
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
      rclcpp::get_logger("NitrosImu"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();
  auto frame_name = pose_tree_handle->getFrameName(imu_parts.pose_frame_uid->uid);
  if (frame_name) {
    destination.header.frame_id = frame_name.value();
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosImu"),
      "Setting frame_id=%s from NITROS msg",
      source.frame_id.c_str());
    // Set NITROS frame id as fallback method of populating frame_id
    destination.header.frame_id = source.frame_id;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosImu"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosImu,
  sensor_msgs::msg::Imu>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosImu::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosImu"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // Get pointer to allocator component
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kEntityName, kComponentName, kComponentTypeName, cid);
  auto maybe_allocator_handle =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, cid);
  if (!maybe_allocator_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get allocator's handle: " <<
      GxfResultStr(maybe_allocator_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImu"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  // create imu gxf msg
  auto maybe_imu_parts = nvidia::isaac::CreateImuMessage(context);
  if (!maybe_imu_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_gxf_message] Failed to create imu gxf msg " << GxfResultStr(
      maybe_imu_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImu"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto imu_parts = maybe_imu_parts.value();

  // Add imu data
  imu_parts.imu->angular_velocity_x = source.angular_velocity.x;
  imu_parts.imu->angular_velocity_y = source.angular_velocity.y;
  imu_parts.imu->angular_velocity_z = source.angular_velocity.z;
  imu_parts.imu->linear_acceleration_x = source.linear_acceleration.x;
  imu_parts.imu->linear_acceleration_y = source.linear_acceleration.y;
  imu_parts.imu->linear_acceleration_z = source.linear_acceleration.z;

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  imu_parts.timestamp->acqtime = input_timestamp;

  // Set frame ID
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
      rclcpp::get_logger("NitrosImu"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();
  auto maybe_imu_frame_uid = pose_tree_handle->findOrCreateFrame(
    source.header.frame_id.c_str());
  if (maybe_imu_frame_uid) {
    imu_parts.pose_frame_uid->uid = maybe_imu_frame_uid.value();
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("NitrosImu"), "Could not create Pose Tree Frame");
  }

  // Set NITROS frame id as fallback method of populating frame_id
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = imu_parts.message.eid();
  GxfEntityRefCountInc(context, imu_parts.message.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosImu"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
