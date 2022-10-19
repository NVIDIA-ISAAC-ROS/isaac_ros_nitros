/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <cuda_runtime.h>

#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_occupancy_grid_type/nitros_occupancy_grid.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";

constexpr int kExpectedPoseAsTensorSize = (3 + 4);  // Translation (XYZ) and Rotation (XYZW)

constexpr char kResolutionName[] = "resolution";
constexpr char kWidthName[] = "width";
constexpr char kHeightName[] = "height";
constexpr char kOriginName[] = "origin";
constexpr char kDataName[] = "data";

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosOccupancyGrid,
  nav_msgs::msg::OccupancyGrid>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosOccupancyGrid::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosOccupancyGrid"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  // Primitive metadata
  auto maybe_gxf_resolution = msg_entity->get<float>(kResolutionName);
  if (!maybe_gxf_resolution) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get the resolution: " <<
      GxfResultStr(maybe_gxf_resolution.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  destination.info.resolution = *maybe_gxf_resolution.value().get();

  auto maybe_gxf_width = msg_entity->get<int>(kWidthName);
  if (!maybe_gxf_width) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get the width: " <<
      GxfResultStr(maybe_gxf_width.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  destination.info.width = *maybe_gxf_width.value().get();

  auto maybe_gxf_height = msg_entity->get<int>(kHeightName);
  if (!maybe_gxf_height) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get the height: " <<
      GxfResultStr(maybe_gxf_height.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  destination.info.height = *maybe_gxf_height.value().get();

  auto maybe_gxf_pose_tensor = msg_entity->get<nvidia::gxf::Tensor>(kOriginName);
  if (!maybe_gxf_pose_tensor) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get the origin: " <<
      GxfResultStr(maybe_gxf_pose_tensor.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto gxf_pose_tensor = maybe_gxf_pose_tensor.value();

  if (gxf_pose_tensor->shape() != nvidia::gxf::Shape{kExpectedPoseAsTensorSize}) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Pose Tensor was not the correct shape: " <<
      gxf_pose_tensor->size();
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Allocate array for copying GXF Pose Tensor data as contiguous block
  std::array<double, kExpectedPoseAsTensorSize> ros_pose_tensor{};

  // Copy pose tensor off device to CPU memory
  switch (gxf_pose_tensor->storage_type()) {
    case nvidia::gxf::MemoryStorageType::kHost:
      {
        std::memcpy(ros_pose_tensor.data(), gxf_pose_tensor->pointer(), gxf_pose_tensor->size());
      }
      break;
    case nvidia::gxf::MemoryStorageType::kDevice:
      {
        const cudaError_t cuda_error = cudaMemcpy(
          ros_pose_tensor.data(), gxf_pose_tensor->pointer(),
          gxf_pose_tensor->size(), cudaMemcpyDeviceToHost);
        if (cuda_error != cudaSuccess) {
          std::stringstream error_msg;
          error_msg <<
            "[convert_to_ros_message] cudaMemcpy failed for conversion from "
            "gxf::Tensor to ROS Pose: " <<
            cudaGetErrorName(cuda_error) <<
            " (" << cudaGetErrorString(cuda_error) << ")";
          RCLCPP_ERROR(
            rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
          throw std::runtime_error(error_msg.str().c_str());
        }
      }
      break;
    default:
      std::string error_msg =
        "[convert_to_ros_message] MemoryStorageType not supported: conversion from "
        "gxf::Tensor to ROS Pose failed!";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.c_str());
      throw std::runtime_error(error_msg.c_str());
  }

  // Populate the message object's origin fields
  destination.info.origin.position.x = ros_pose_tensor.at(0);
  destination.info.origin.position.y = ros_pose_tensor.at(1);
  destination.info.origin.position.z = ros_pose_tensor.at(2);

  destination.info.origin.orientation.x = ros_pose_tensor.at(3);
  destination.info.origin.orientation.y = ros_pose_tensor.at(4);
  destination.info.origin.orientation.z = ros_pose_tensor.at(5);
  destination.info.origin.orientation.w = ros_pose_tensor.at(6);

  auto maybe_gxf_data_tensor = msg_entity->get<nvidia::gxf::Tensor>(kDataName);
  if (!maybe_gxf_data_tensor) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get the data: " <<
      GxfResultStr(maybe_gxf_data_tensor.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto gxf_data_tensor = maybe_gxf_data_tensor.value();

  // Resize before copying GXF cell data as contiguous block
  destination.data.resize(gxf_data_tensor->size());

  // Copy data tensor off device to CPU memory
  switch (gxf_data_tensor->storage_type()) {
    case nvidia::gxf::MemoryStorageType::kHost:
      {
        std::memcpy(destination.data.data(), gxf_data_tensor->pointer(), gxf_data_tensor->size());
      }
      break;
    case nvidia::gxf::MemoryStorageType::kDevice:
      {
        const cudaError_t cuda_error = cudaMemcpy(
          destination.data.data(), gxf_data_tensor->pointer(),
          gxf_data_tensor->size(), cudaMemcpyDeviceToHost);
        if (cuda_error != cudaSuccess) {
          std::stringstream error_msg;
          error_msg <<
            "[convert_to_ros_message] cudaMemcpy failed for conversion from "
            "gxf::Tensor to ROS int8 array: " <<
            cudaGetErrorName(cuda_error) <<
            " (" << cudaGetErrorString(cuda_error) << ")";
          RCLCPP_ERROR(
            rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
          throw std::runtime_error(error_msg.str().c_str());
        }
      }
      break;
    default:
      std::string error_msg =
        "[convert_to_ros_message] MemoryStorageType not supported: conversion from "
        "gxf::Tensor to ROS int8 array failed!";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.c_str());
      throw std::runtime_error(error_msg.c_str());
  }

  // Populate timestamp information back into ROS header
  auto input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  if (!input_timestamp) {    // Fallback to label 'timestamp'
    input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  }
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp.value()->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp.value()->acqtime % static_cast<uint64_t>(1e9));
  }

  // Set frame ID
  destination.header.frame_id = source.frame_id;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosOccupancyGrid"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosOccupancyGrid,
  nav_msgs::msg::OccupancyGrid>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosOccupancyGrid::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosOccupancyGrid"),
    "[convert_to_custom] Conversion started");

  // Get pointer to allocator component
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kEntityName, kComponentName, kComponentTypeName, cid);

  auto maybe_allocator_handle =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(), cid);
  if (!maybe_allocator_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get allocator's handle: " <<
      GxfResultStr(maybe_allocator_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  auto message = nvidia::gxf::Entity::New(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext());
  if (!message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Primitive metadata
  auto gxf_resolution = message->add<float>(kResolutionName);
  *gxf_resolution.value() = source.info.resolution;

  auto gxf_width = message->add<int>(kWidthName);
  *gxf_width.value() = source.info.width;

  auto gxf_height = message->add<int>(kHeightName);
  *gxf_height.value() = source.info.height;

  // Initializing GXF tensor
  auto gxf_pose_tensor = message->add<nvidia::gxf::Tensor>(kOriginName);
  auto result = gxf_pose_tensor.value()->reshape<double>(
    nvidia::gxf::Shape{kExpectedPoseAsTensorSize}, nvidia::gxf::MemoryStorageType::kDevice,
    allocator_handle);

  if (!result) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error initializing GXF pose tensor of shape (" <<
      kExpectedPoseAsTensorSize << ",): " <<
      GxfResultStr(result.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Get pointer to first element of pose tensor
  auto maybe_pose_data = gxf_pose_tensor.value()->data<double>();
  if (!maybe_pose_data) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Could not get data pointer from Pose Tensor: " <<
      gxf_pose_tensor.value();
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Allocate array for copying ROS Pose data as contiguous block
  std::array<double, kExpectedPoseAsTensorSize> ros_pose_tensor{
    source.info.origin.position.x,
    source.info.origin.position.y,
    source.info.origin.position.z,

    source.info.origin.orientation.x,
    source.info.origin.orientation.y,
    source.info.origin.orientation.z,
    source.info.origin.orientation.w
  };

  // Populate ROS Pose data into GXF Tensor
  cudaError_t cuda_error = cudaMemcpy(
    gxf_pose_tensor.value()->pointer(),
    ros_pose_tensor.data(),
    gxf_pose_tensor.value()->size(),
    cudaMemcpyHostToDevice
  );
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMemcpy failed for copying data from "
      "ROS Pose Tensor to GXF Tensor: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPoseArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Initializing GXF tensor
  auto gxf_data_tensor = message->add<nvidia::gxf::Tensor>(kDataName);
  result = gxf_data_tensor.value()->reshape<int>(
    nvidia::gxf::Shape{static_cast<int>(source.data.size())},
    nvidia::gxf::MemoryStorageType::kDevice,
    allocator_handle);

  if (!result) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error initializing GXF data tensor of shape (" <<
      source.data.size() << ",): " <<
      GxfResultStr(result.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Get pointer to first element of data tensor
  auto maybe_data = gxf_data_tensor.value()->data<int>();
  if (!maybe_data) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Could not get data pointer from data tensor: " <<
      gxf_data_tensor.value();
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Populate ROS data into GXF Tensor
  cuda_error = cudaMemcpy(
    gxf_data_tensor.value()->pointer(),
    source.data.data(),
    gxf_data_tensor.value()->size(),
    cudaMemcpyHostToDevice
  );
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMemcpy failed for copying data from "
      "ROS data tensor to GXF tensor: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  auto output_timestamp = message->add<nvidia::gxf::Timestamp>("timestamp");
  if (!output_timestamp) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Failed to add a timestamp component to message: " <<
      GxfResultStr(output_timestamp.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosOccupancyGrid"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  output_timestamp.value()->acqtime = input_timestamp;

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set message entity
  destination.handle = message->eid();
  GxfEntityRefCountInc(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(), message->eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosOccupancyGrid"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
