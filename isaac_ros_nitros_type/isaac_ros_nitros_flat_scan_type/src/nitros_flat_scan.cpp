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
#include <cuda_runtime.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "flat_scan_message.hpp"
#include "gems/pose_tree/pose_tree.hpp"
#include "extensions/atlas/pose_tree_frame.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_flat_scan_type/nitros_flat_scan.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr char kMemoryEntityName[] = "memory_pool";
constexpr char kMemoryComponentName[] = "unbounded_allocator";
constexpr char kMemoryComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";
constexpr char kPoseTreeEntityName[] = "global_pose_tree";
constexpr char kPoseTreeComponentName[] = "pose_tree";
constexpr char kPoseTreeComponentTypeName[] = "nvidia::isaac::PoseTree";
constexpr char kNameBeamsDevice[] = "beams";
constexpr int kFlatscanAngleIndx = 0;
constexpr int kFlatscanRangeIndx = 2;
constexpr int kNFieldsFlatscanMsg = 5;
}  // namespace

template<typename Deleter>
using unique_p = std::unique_ptr<double[], Deleter>;

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosFlatScan,
  isaac_ros_pointcloud_interfaces::msg::FlatScan>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosFlatScan::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosFlatScan"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto maybe_flatscan_parts = nvidia::isaac_ros::messages::GetFlatscanMessage(
    msg_entity.value());
  if (!maybe_flatscan_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get flatscan message from message entity: " <<
      GxfResultStr(maybe_flatscan_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto flatscan_parts = maybe_flatscan_parts.value();

  // Extract GXF Tensor
  auto beams_tensor = flatscan_parts.beams;
  const auto & beams_tensor_shape = beams_tensor->shape();

  // Tensor checks
  if (beams_tensor->rank() != 2) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Flatscan tensor expected to be of rank 2, but got rank " <<
      beams_tensor->rank();
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  if (beams_tensor_shape.dimension(1) != kNFieldsFlatscanMsg) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Flatscan tensor shape dimension(1) expected to be" <<
      kNFieldsFlatscanMsg <<
      "but got" << beams_tensor_shape.dimension(1);
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Tensor is assumed to be of shape (num_points,kNFieldsFlatscanMsg)
  size_t num_points = beams_tensor_shape.dimension(0);

  // allocate CPU memory in case tensor for use
  // in case tensor is GPU based
  // using char to represnt 1 byte pointer
  auto deleter = [](double * ptr) {cudaFreeHost(ptr);};
  auto pointerCudaMalloc = [](size_t mySize) {
      void * ptr; cudaMallocHost(reinterpret_cast<void **>(&ptr), mySize); return ptr;
    };
  unique_p<decltype(deleter)> beams_cpu_pointer(
    reinterpret_cast<double *>(pointerCudaMalloc(beams_tensor->size())), deleter);

  ::nvidia::isaac::CpuTensorView2d beams_tensor_view;
  switch (beams_tensor->storage_type()) {
    case nvidia::gxf::MemoryStorageType::kHost:
      {
        // CPU based tensor
        beams_tensor_view = ::nvidia::isaac::CreateCpuTensorViewFromData<double,
            2>(
          beams_tensor->data<double>().value(), beams_tensor_shape.size(),
          ::nvidia::isaac::Vector2i(
            beams_tensor_shape.dimension(0), beams_tensor_shape.dimension(1)));
      }
      break;
    case nvidia::gxf::MemoryStorageType::kDevice:
      {
        // GPU based tensor
        // Copy beams tensor off device to CPU memory
        const cudaError_t cuda_error = cudaMemcpy(
          beams_cpu_pointer.get(), beams_tensor->pointer(),
          beams_tensor->size(), cudaMemcpyDeviceToHost);
        if (cuda_error != cudaSuccess) {
          std::stringstream error_msg;
          error_msg <<
            "[convert_to_ros_message] cudaMemcpy failed for conversion from "
            "gxf::Tensor to ROS FlatScan: " <<
            cudaGetErrorName(cuda_error) <<
            " (" << cudaGetErrorString(cuda_error) << ")";
          RCLCPP_ERROR(
            rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
          throw std::runtime_error(error_msg.str().c_str());
        }
        beams_tensor_view =
          ::nvidia::isaac::CreateCpuTensorViewFromData<double, 2>(
          beams_cpu_pointer.get(), beams_tensor_shape.size(),
          ::nvidia::isaac::Vector2i(
            beams_tensor_shape.dimension(0), beams_tensor_shape.dimension(1)));
      }
      break;
    default:
      std::string error_msg =
        "[convert_to_ros_message] MemoryStorageType not supported: conversion from "
        "gxf::Tensor to ROS FlatScan!";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosFlatScan"), error_msg.c_str());
      throw std::runtime_error(error_msg.c_str());
  }

  // iterate through each point and convert float64 beam range and intensity to float32
  // This is required since the data is stored as a double in the isaac message but as a float32
  // in the ros message
  for (size_t i = 0; i < num_points; i += 1) {
    // index 0 corresponds to angle refer flatscan_types.hpp
    destination.ranges.push_back(static_cast<float>(beams_tensor_view(i, kFlatscanRangeIndx)));
    // index 2 corresponds to range refer flatscan_types.hpp
    destination.angles.push_back(static_cast<float>(beams_tensor_view(i, kFlatscanAngleIndx)));
  }

  destination.range_max = static_cast<float>(flatscan_parts.info->out_of_range);
  destination.range_min = 0.0;  // range_min not present in gxf flatscan message

  // Populate timestamp information back into ROS header
  auto input_timestamp = flatscan_parts.timestamp;
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
      rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();
  auto frame_name = pose_tree_handle->getFrameName(flatscan_parts.pose_frame_uid->uid);
  if (frame_name) {
    destination.header.frame_id = frame_name.value();
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("NitrosFlatScan"), "Setting frame if from NITROS msg");
    // Set NITROS frame id as fallback method of populating frame_id
    // Set frame ID
    destination.header.frame_id = source.frame_id;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosFlatScan"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosFlatScan,
  isaac_ros_pointcloud_interfaces::msg::FlatScan>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosFlatScan::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosFlatScan"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // Get pointer to allocator component
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kMemoryEntityName, kMemoryComponentName, kMemoryComponentTypeName, cid);
  auto maybe_allocator_handle =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, cid);
  if (!maybe_allocator_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get allocator's handle: " <<
      GxfResultStr(maybe_allocator_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  if (source.ranges.size() != source.angles.size()) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Ranges and Angles array do not have the same size" <<
      "Ranges array size " << source.ranges.size() <<
      "Angles array size " << source.angles.size();
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  const int n_points = source.ranges.size();

  auto maybe_flatscan_parts = nvidia::isaac_ros::messages::CreateFlatscanMessage(
    context, allocator_handle, n_points, false);
  if (!maybe_flatscan_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to create CreatePointCloudMessage " << GxfResultStr(
      maybe_flatscan_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto flatscan_parts = maybe_flatscan_parts.value();

  auto beams_tensor = flatscan_parts.beams;

  // allocate CPU memory in case tensor for use
  // in case tensor is GPU based
  // using char to represnt 1 byte pointer
  size_t beams_size_bytes = n_points * sizeof(double) * kNFieldsFlatscanMsg;
  auto deleter = [](double * ptr) {cudaFreeHost(ptr);};
  unique_p<decltype(deleter)> beams_cpu_pointer(new double[n_points], deleter);
  cudaMallocHost(reinterpret_cast<void **>(&beams_cpu_pointer), beams_size_bytes);
  ::nvidia::isaac::CpuTensorView2d beams_tensor_view =
    ::nvidia::isaac::CreateCpuTensorViewFromData<double, 2>(
    beams_cpu_pointer.get(), beams_size_bytes,
    ::nvidia::isaac::Vector2i(n_points, kNFieldsFlatscanMsg));
  for (int point_index = 0; point_index < n_points; point_index++) {
    beams_tensor_view(point_index, kFlatscanAngleIndx) = source.angles[point_index];
    beams_tensor_view(point_index, kFlatscanRangeIndx) = source.ranges[point_index];
  }
  switch (beams_tensor->storage_type()) {
    case nvidia::gxf::MemoryStorageType::kHost:
      {
        // CPU based tensor
        // Copy data from CPU to CPU backed gxf tensor.
        const cudaError_t cuda_error = cudaMemcpy(
          beams_tensor->data<double>().value(),
          beams_cpu_pointer.get(),
          beams_size_bytes, cudaMemcpyHostToHost);
        if (cuda_error != cudaSuccess) {
          std::stringstream error_msg;
          error_msg <<
            "[convert_to_custom] cudaMemcpy failed for copying data from "
            "CPU to CPU: " <<
            cudaGetErrorName(cuda_error) <<
            " (" << cudaGetErrorString(cuda_error) << ")";
          RCLCPP_ERROR(
            rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
          throw std::runtime_error(error_msg.str().c_str());
        }
      }
      break;
    case nvidia::gxf::MemoryStorageType::kDevice:
      {
        // GPU based tensor
        // Copy data from CPU to GPU backed gxf tensor.
        const cudaError_t cuda_error = cudaMemcpy(
          beams_tensor->data<double>().value(),
          beams_cpu_pointer.get(),
          beams_size_bytes, cudaMemcpyHostToDevice);
        if (cuda_error != cudaSuccess) {
          std::stringstream error_msg;
          error_msg <<
            "[convert_to_custom] cudaMemcpy failed for copying data from "
            "CPU to GPU: " <<
            cudaGetErrorName(cuda_error) <<
            " (" << cudaGetErrorString(cuda_error) << ")";
          RCLCPP_ERROR(
            rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
          throw std::runtime_error(error_msg.str().c_str());
        }
      }
      break;
    default:
      std::string error_msg =
        "[convert_to_ros_message] MemoryStorageType not supported: conversion from "
        "ROS FlatScan to gxf::Tensor failed!";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosFlatScan"), error_msg.c_str());
      throw std::runtime_error(error_msg.c_str());
  }

  flatscan_parts.info->out_of_range = source.range_max;

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  flatscan_parts.timestamp->acqtime = input_timestamp;

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
      rclcpp::get_logger("NitrosFlatScan"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();
  auto maybe_flat_scan_frame_uid = pose_tree_handle->findOrCreateFrame(
    source.header.frame_id.c_str());
  if (maybe_flat_scan_frame_uid) {
    flatscan_parts.pose_frame_uid->uid = maybe_flat_scan_frame_uid.value();
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("NitrosFlatScan"), "Could not create Pose Tree Frame");
  }

  // Set NITROS frame id as fallback method of populating frame_id
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = flatscan_parts.entity.eid();
  GxfEntityRefCountInc(context, flatscan_parts.entity.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosFlatScan"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
