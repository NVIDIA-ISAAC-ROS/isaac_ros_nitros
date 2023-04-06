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

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"


constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosTensorList,
  isaac_ros_tensor_list_interfaces::msg::TensorList>::convert_to_ros_message(
  const custom_type & source,
  ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosTensorList::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorList"),
    "[convert_to_ros_message] Conversion started for handle = %ld", source.handle);

  auto msg_entity = nvidia::gxf::Entity::Shared(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(), source.handle);

  auto gxf_tensors = msg_entity->findAll<nvidia::gxf::Tensor>();
  if (!gxf_tensors) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] failed to get all GXF tensors: " <<
      GxfResultStr(gxf_tensors.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  for (auto gxf_tensor_handle : gxf_tensors.value()) {
    auto gxf_tensor = gxf_tensor_handle.value();
    // Create ROS 2 Tensor and populate the message object's fields
    auto ros_tensor = isaac_ros_tensor_list_interfaces::msg::Tensor();
    ros_tensor.name = gxf_tensor.name();
    ros_tensor.data_type = static_cast<int32_t>(gxf_tensor->element_type());
    ros_tensor.shape.rank = gxf_tensor->shape().rank();

    // Moving data from GXF tensor to ROS tensor message
    ros_tensor.data.resize(gxf_tensor->size());

    switch (gxf_tensor->storage_type()) {
      case nvidia::gxf::MemoryStorageType::kHost:
        {
          std::memcpy(ros_tensor.data.data(), gxf_tensor->pointer(), gxf_tensor->size());
        }
        break;
      case nvidia::gxf::MemoryStorageType::kDevice:
        {
          const cudaError_t cuda_error = cudaMemcpy(
            ros_tensor.data.data(), gxf_tensor->pointer(),
            gxf_tensor->size(), cudaMemcpyDeviceToHost);
          if (cuda_error != cudaSuccess) {
            std::stringstream error_msg;
            error_msg <<
              "[convert_to_ros_message] cudaMemcpy failed for conversion from "
              "gxf::Tensor to ROS Tensor: " <<
              cudaGetErrorName(cuda_error) <<
              " (" << cudaGetErrorString(cuda_error) << ")";
            RCLCPP_ERROR(
              rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
            throw std::runtime_error(error_msg.str().c_str());
          }
        }
        break;
      default:
        std::string error_msg =
          "[convert_to_ros_message] MemoryStorageType not supported: conversion from "
          "gxf::Tensor to ROS Tensor failed!";
        RCLCPP_ERROR(
          rclcpp::get_logger("NitrosTensorList"), error_msg.c_str());
        throw std::runtime_error(error_msg.c_str());
    }

    for (size_t i = 0; i < gxf_tensor->shape().rank(); i++) {
      ros_tensor.shape.dims.push_back(gxf_tensor->shape().dimension(i));
      ros_tensor.strides.push_back(gxf_tensor->stride(i));
    }
    destination.tensors.push_back(ros_tensor);
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
    rclcpp::get_logger("NitrosTensorList"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosTensorList,
  isaac_ros_tensor_list_interfaces::msg::TensorList>::convert_to_custom(
  const ros_message_type & source, custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosTensorList::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorList"),
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
      rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
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
      rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  for (size_t i = 0; i < source.tensors.size(); i++) {
    auto ros_tensor = source.tensors[i];
    auto gxf_tensor = message->add<nvidia::gxf::Tensor>(ros_tensor.name.c_str());
    std::array<int32_t, nvidia::gxf::Shape::kMaxRank> dims;
    std::copy(
      std::begin(ros_tensor.shape.dims), std::end(ros_tensor.shape.dims),
      std::begin(dims));

    nvidia::gxf::Expected<void> result;
    nvidia::gxf::MemoryStorageType storage_type = nvidia::gxf::MemoryStorageType::kDevice;
    // Initializing GXF tensor
    nvidia::gxf::PrimitiveType type =
      static_cast<nvidia::gxf::PrimitiveType>(ros_tensor.data_type);
    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosTensorList"),
      "[convert_to_custom] dims[0]=%d, rank=%d, storage_type=%d",
      dims[0], ros_tensor.shape.rank, (int)storage_type);
    switch (type) {
      case nvidia::gxf::PrimitiveType::kUnsigned8:
        result = gxf_tensor.value()->reshape<uint8_t>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kInt8:
        result = gxf_tensor.value()->reshape<int8_t>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kUnsigned16:
        result = gxf_tensor.value()->reshape<uint16_t>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kInt16:
        result = gxf_tensor.value()->reshape<int16_t>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kUnsigned32:
        result = gxf_tensor.value()->reshape<uint32_t>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kInt32:
        result = gxf_tensor.value()->reshape<int32_t>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kUnsigned64:
        result = gxf_tensor.value()->reshape<uint64_t>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kInt64:
        result = gxf_tensor.value()->reshape<int32_t>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kFloat32:
        result = gxf_tensor.value()->reshape<float>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      case nvidia::gxf::PrimitiveType::kFloat64:
        result = gxf_tensor.value()->reshape<double>(
          nvidia::gxf::Shape(dims, ros_tensor.shape.rank), storage_type, allocator_handle);
        break;
      default:
        std::string error_msg = "[convert_to_custom] Tensor data type not supported.";
        RCLCPP_ERROR(
          rclcpp::get_logger("NitrosTensorList"), error_msg.c_str());
        throw std::runtime_error(error_msg.c_str());
    }
    if (!result) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_custom] Error initializing GXF tensor of type " <<
        static_cast<int>(type) << ": " <<
        GxfResultStr(result.error());
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    const cudaMemcpyKind operation = cudaMemcpyHostToDevice;
    const cudaError_t cuda_error = cudaMemcpy(
      gxf_tensor.value()->pointer(),
      ros_tensor.data.data(),
      gxf_tensor.value()->size(),
      operation);

    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_custom] cudaMemcpy failed for copying data from "
        "ROS Tensor to GXF Tensor: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
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
      rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
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
    rclcpp::get_logger("NitrosTensorList"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
