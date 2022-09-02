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

#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"


namespace
{
constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";

using DistortionType = nvidia::gxf::DistortionType;
const std::unordered_map<std::string, DistortionType> g_ros_to_gxf_distortion_model({
    {"plumb_bob", DistortionType::Brown},
    {"rational_polynomial", DistortionType::Polynomial},
    {"equidistant", DistortionType::FisheyeEquidistant}
  });

const std::unordered_map<DistortionType, std::string> g_gxf_to_ros_distortion_model({
    {DistortionType::Brown, "plumb_bob"},
    {DistortionType::Polynomial, "rational_polynomial"},
    {DistortionType::FisheyeEquidistant, "equidistant"}
  });
}  // namespace


void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPointCloud,
  sensor_msgs::msg::PointCloud2>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosPointCloud::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
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
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto gxf_tensor = maybe_gxf_tensor.value();

  auto use_color = *(msg_entity->get<bool>("use_color")->get());

  const auto tensorShape = gxf_tensor->shape();

  destination.height = tensorShape.dimension(0);
  destination.width = tensorShape.dimension(1);
  destination.is_bigendian = false;
  destination.is_dense = false;

  sensor_msgs::PointCloud2Modifier pc2_modifier(destination);

  if (use_color) {
    // Data format: x,y,z,rgb; 16 bytes per point
    pc2_modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  } else {
    // Data format: x,y,z; 12 bytes per point
    pc2_modifier.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  }

  const cudaError_t cuda_error = cudaMemcpy(
    destination.data.data(),
    gxf_tensor->data<float>().value(), destination.row_step * destination.height,
    cudaMemcpyDeviceToHost);

  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaMemcpy failed for conversion from "
      "NitrosPointCloud to sensor_msgs::msg::PointCloud2: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_ros_message] "
    "row_step: %d, point_step: %d, x_offset: %d, y_offset: %d, y_offset: %d",
    destination.row_step,
    destination.point_step,
    destination.fields[0].offset,
    destination.fields[1].offset,
    destination.fields[2].offset);

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
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPointCloud,
  sensor_msgs::msg::PointCloud2>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosPointCloud::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  auto message = nvidia::gxf::Entity::New(context);
  if (!message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  int32_t height = source.height;
  int32_t width = source.width;
  int32_t point_float_count = source.point_step / sizeof(float);

  auto gxf_tensor = message->add<nvidia::gxf::Tensor>("point_clouds");
  auto gxf_use_color = message->add<bool>("use_color");

  *gxf_use_color->get() = (point_float_count == 4);  // {x, y, z, RGB}

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_custom] height: %d, width: %d, point_float_count: %d, use_color: %d",
    height, width, point_float_count, *gxf_use_color->get());

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
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  // Allocate memory of gxf tensor.
  auto gxf_tensor_result = gxf_tensor.value()->reshape<float>(
    nvidia::gxf::Shape{height, width, point_float_count}, nvidia::gxf::MemoryStorageType::kDevice,
    allocator_handle);
  if (!gxf_tensor_result) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error Creating tensors: " <<
      GxfResultStr(gxf_tensor_result.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Copy data from point cloud msg to gxf tensor.
  const cudaError_t cuda_error = cudaMemcpy(
    gxf_tensor.value()->data<float>().value(),
    source.data.data(), source.row_step * source.height, cudaMemcpyHostToDevice);
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMemcpy failed for copying data from "
      "sensor_msgs::msg::PointCloud2 to NitrosPointCloud: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
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
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  output_timestamp.value()->acqtime = input_timestamp;

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = message->eid();
  GxfEntityRefCountInc(context, message->eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosCameraInfo"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
