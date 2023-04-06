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
#include <typeinfo>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/multimedia/video.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_disparity_image_type/nitros_disparity_image.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"


namespace
{
const char kEntityName[] = "memory_pool";
const char kComponentName[] = "unbounded_allocator";
const char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";

using VideoFormat = nvidia::gxf::VideoFormat;
// Map to store the ROS format encoding to Nitros format encoding
const std::unordered_map<std::string, VideoFormat> g_ros_to_gxf_video_format({
    {"bgr8", VideoFormat::GXF_VIDEO_FORMAT_BGR},
    {"32FC1", VideoFormat::GXF_VIDEO_FORMAT_GRAY32}
  });

// Map to store the Nitros format encoding to ROS format encoding
const std::unordered_map<VideoFormat, std::string> g_gxf_to_ros_video_format({
    {VideoFormat::GXF_VIDEO_FORMAT_BGR, "bgr8"},
    {VideoFormat::GXF_VIDEO_FORMAT_GRAY32, "32FC1"}
  });

// Get step size for ROS Image
uint32_t GetStepSize(const nvidia::gxf::VideoBufferInfo & video_buff_info)
{
  return video_buff_info.width * video_buff_info.color_planes[0].bytes_per_pixel;
}

// Allocate buffer for Nitros Image
void AllocateVideoBuffer(
  const sensor_msgs::msg::Image & source,
  const nvidia::gxf::Handle<nvidia::gxf::VideoBuffer> & video_buff,
  const nvidia::gxf::Handle<nvidia::gxf::Allocator> & allocator_handle)
{
  auto color_fmt = g_ros_to_gxf_video_format.find(source.encoding);
  if (color_fmt == std::end(g_ros_to_gxf_video_format)) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Unsupported encoding from ROS: " <<
      source.encoding.c_str();
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  constexpr auto surface_layout = nvidia::gxf::SurfaceLayout::GXF_SURFACE_LAYOUT_PITCH_LINEAR;
  constexpr auto storage_type = nvidia::gxf::MemoryStorageType::kDevice;
  switch (color_fmt->second) {
    case VideoFormat::GXF_VIDEO_FORMAT_GRAY32:
      video_buff->resize<VideoFormat::GXF_VIDEO_FORMAT_GRAY32>(
        source.width, source.height, surface_layout, storage_type, allocator_handle);
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_BGR:
      video_buff->resize<VideoFormat::GXF_VIDEO_FORMAT_BGR>(
        source.width, source.height, surface_layout, storage_type, allocator_handle);
      break;
    default:
      break;
  }
}
}  // namespace


void rclcpp::TypeAdapter<nvidia::isaac_ros::nitros::NitrosDisparityImage,
  stereo_msgs::msg::DisparityImage>::convert_to_ros_message(
  const custom_type & source,
  ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosDisparityImage::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImage"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto gxf_video_buffer = msg_entity->get<nvidia::gxf::VideoBuffer>();
  if (!gxf_video_buffer) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] [convert_to_ros_message] Failed to get the existing "
      "VideoBuffer object: " <<
      GxfResultStr(gxf_video_buffer.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Setting Image data from gxf VideoBuffer
  auto video_buffer_info = gxf_video_buffer.value()->video_frame_info();
  destination.image.height = video_buffer_info.height;
  destination.image.width = video_buffer_info.width;
  const auto encoding = g_gxf_to_ros_video_format.find(video_buffer_info.color_format);
  if (encoding == std::end(g_gxf_to_ros_video_format)) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Unsupported encoding from GXF: " <<
      static_cast<int>(video_buffer_info.color_format);
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  } else {
    destination.image.encoding = encoding->second;
  }

  // Bigendian or not
  destination.image.is_bigendian = 0;

  // Full row length in bytes
  destination.image.step = GetStepSize(video_buffer_info);

  // Resize the ROS image buffer to the right size
  destination.image.data.resize(destination.image.step * destination.image.height);

  // Copy data from Device to Host
  const cudaError_t cuda_error = cudaMemcpy2D(
    destination.image.data.data(),
    destination.image.step,
    gxf_video_buffer.value()->pointer(),
    video_buffer_info.color_planes[0].stride,
    destination.image.step,
    destination.image.height,
    cudaMemcpyDeviceToHost);

  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaMemcpy2D failed for conversion from "
      "NitrosDisparityImage to stereo_msgs::msg::Image: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  auto gxf_disparity_parameters = msg_entity->findAll<float>();
  if (!gxf_disparity_parameters) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] failed to get all GXF tensors: " <<
      GxfResultStr(gxf_disparity_parameters.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  for (const auto & parameter_handle : gxf_disparity_parameters.value()) {
    auto parameter = parameter_handle.value();
    if (!std::strcmp(parameter.name(), "f")) {
      destination.f = *parameter.get();
    } else if (!std::strcmp(parameter.name(), "t")) {
      destination.t = *parameter.get();
    } else if (!std::strcmp(parameter.name(), "min_disparity")) {
      destination.min_disparity = *parameter.get();
    } else if (!std::strcmp(parameter.name(), "max_disparity")) {
      destination.max_disparity = *parameter.get();
    } else {
      continue;
    }
  }

  // Populate timestamp information back into ROS header
  auto input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  if (!input_timestamp) {    // Fallback to any 'timestamp'
    input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  }
  if (input_timestamp) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosDisparityImage"),
      "[convert_to_ros_message] Enter the timestamp section");
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp.value()->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp.value()->acqtime % static_cast<uint64_t>(1e9));
    destination.image.header = destination.header;
  }

  // Set frame ID
  destination.header.frame_id = source.frame_id;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImage"),
    "[convert_to_ros_message] Conversion completed for handle=%ld", source.handle);

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}


void rclcpp::TypeAdapter<nvidia::isaac_ros::nitros::NitrosDisparityImage,
  stereo_msgs::msg::DisparityImage>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosDisparityImage::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImage"),
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
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  auto message = nvidia::gxf::Entity::New(context);
  if (!message) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDIsparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  auto gxf_video_buffer = message->add<nvidia::gxf::VideoBuffer>(source.header.frame_id.c_str());
  if (!gxf_video_buffer) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Failed to create a VideoBuffer object: " <<
      GxfResultStr(gxf_video_buffer.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Allocate video buffer
  AllocateVideoBuffer(source.image, gxf_video_buffer.value(), allocator_handle);

  auto video_buffer_info = gxf_video_buffer.value()->video_frame_info();

  // Copy data from Host to Device
  const cudaError_t cuda_error = cudaMemcpy2D(
    gxf_video_buffer.value()->pointer(),
    video_buffer_info.color_planes[0].stride,
    source.image.data.data(),
    source.image.step,
    video_buffer_info.width * video_buffer_info.color_planes[0].bytes_per_pixel,
    video_buffer_info.height,
    cudaMemcpyHostToDevice);

  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMemcpy2D failed for conversion from "
      "stereo_msgs::msg::DisparityImage to NitrosDisparityImage: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
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
      rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  output_timestamp.value()->acqtime = input_timestamp;

  // Set focal and baseline
  *(message->add<float>("t")->get()) = source.t;
  *(message->add<float>("f")->get()) = source.f;
  *(message->add<float>("min_disparity")->get()) = source.min_disparity;
  *(message->add<float>("max_disparity")->get()) = source.max_disparity;

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = message->eid();
  GxfEntityRefCountInc(context, message->eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImage"),
    "[convert_to_custom] Conversion completed (resulting handle=%ld)", message->eid());

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
