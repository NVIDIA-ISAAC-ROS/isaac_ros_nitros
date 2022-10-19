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
#include <unordered_map>
#include <vector>
#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/multimedia/camera.hpp"
#include "gxf/std/timestamp.hpp"
#include "gxf/std/tensor.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_compressed_image_type/nitros_compressed_image.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"
#include "rclcpp/rclcpp.hpp"


namespace
{
constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";
}  // namespace


void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosCompressedImage,
  sensor_msgs::msg::CompressedImage>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosCompressedImage::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosCompressedImage"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto maybe_gxf_tensor = msg_entity->get<nvidia::gxf::Tensor>();
  if (!maybe_gxf_tensor) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get tensor component from message entity: " <<
      GxfResultStr(maybe_gxf_tensor.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  auto gxf_tensor = maybe_gxf_tensor.value();

  destination.format = "h264";
  destination.data.resize(gxf_tensor->size());

  // compressed results are reside on CPU
  const cudaError_t cuda_error = cudaMemcpy(
    destination.data.data(),
    gxf_tensor->data<uint8_t>().value(), gxf_tensor->size(),
    cudaMemcpyHostToHost);

  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaMemcpy failed for conversion from "
      "NitrosCompressedImage to sensor_msgs::msg::CompressedImage: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Populate timestamp information back into ROS header
  auto input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  if (!input_timestamp) {    // Fallback to any 'timestamp'
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
    rclcpp::get_logger("NitrosCompressedImage"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosCompressedImage,
  sensor_msgs::msg::CompressedImage>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosCompressedImage::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosCompressedImage"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  auto message = nvidia::gxf::Entity::New(context);
  if (!message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  auto gxf_tensor = message->add<nvidia::gxf::Tensor>("compressed_image");

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
      rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  // Allocate memory of gxf tensor.
  auto gxf_tensor_result = gxf_tensor.value()->reshape<uint8_t>(
    nvidia::gxf::Shape{static_cast<int>(source.data.size())}, nvidia::gxf::MemoryStorageType::kHost,
    allocator_handle);
  if (!gxf_tensor_result) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error Creating tensors: " <<
      GxfResultStr(gxf_tensor_result.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Copy data from compressed image msg to gxf tensor.
  const cudaError_t cuda_error = cudaMemcpy(
    gxf_tensor.value()->data<uint8_t>().value(),
    source.data.data(), source.data.size(), cudaMemcpyHostToHost);

  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMemcpy failed for copying data from "
      "sensor_msgs::msg::CompressedImage to NitrosCompressedImage: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
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
      rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  output_timestamp.value()->acqtime = input_timestamp;

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = message->eid();
  GxfEntityRefCountInc(context, message->eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosCompressedImage"),
    "[convert_to_custom] Conversion completed (resulting handle=%ld)", message->eid());

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
