// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "extensions/messages/camera_message.hpp"
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
    {"32FC1", VideoFormat::GXF_VIDEO_FORMAT_D32F}
  });

// Map to store the Nitros format encoding to ROS format encoding
const std::unordered_map<VideoFormat, std::string> g_gxf_to_ros_video_format({
    {VideoFormat::GXF_VIDEO_FORMAT_D32F, "32FC1"},
    {VideoFormat::GXF_VIDEO_FORMAT_GRAY32, "32FC1"}
  });

// Get step size for ROS Image
uint32_t GetStepSize(const nvidia::gxf::VideoBufferInfo & video_buff_info)
{
  return video_buff_info.width * video_buff_info.color_planes[0].bytes_per_pixel;
}

// Allocate buffer for Nitros Image
nvidia::gxf::Expected<nvidia::isaac::CameraMessageParts> CreateCameraMessage(
  const stereo_msgs::msg::DisparityImage & source,
  const nvidia::gxf::Handle<nvidia::gxf::Allocator> & allocator_handle,
  gxf_context_t context)
{
  auto color_fmt = g_ros_to_gxf_video_format.find(source.image.encoding);
  if (color_fmt == std::end(g_ros_to_gxf_video_format)) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Unsupported encoding from ROS: " <<
      source.image.encoding.c_str();
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  constexpr auto surface_layout = nvidia::gxf::SurfaceLayout::GXF_SURFACE_LAYOUT_PITCH_LINEAR;
  constexpr auto storage_type = nvidia::gxf::MemoryStorageType::kDevice;
  switch (color_fmt->second) {
    case VideoFormat::GXF_VIDEO_FORMAT_D32F:
      return nvidia::isaac::CreateCameraMessage<VideoFormat::GXF_VIDEO_FORMAT_D32F>(
        context, static_cast<uint32_t>(source.image.width),
        static_cast<uint32_t>(source.image.height), surface_layout,
        storage_type, allocator_handle, false);
      break;
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosDisparityImage"),
        "[convert_to_custom] Unsupported encoding from ROS [%s].", source.image.encoding.c_str());
      throw std::runtime_error("[convert_to_custom] Unsupported encoding from ROS.");
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

  auto maybe_disparity_image = nvidia::isaac::GetCameraMessage(
    msg_entity.value());
  if (!maybe_disparity_image) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get disparity image from message entity: " <<
      GxfResultStr(maybe_disparity_image.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto disparity_image = maybe_disparity_image.value();

  // Setting Image data from gxf VideoBuffer
  auto video_buffer_info = disparity_image.frame->video_frame_info();
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
    disparity_image.frame->pointer(),
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

  // extract float parameters if available
  auto gxf_disparity_parameters = disparity_image.entity.findAll<float>();
  if (!gxf_disparity_parameters) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] failed to get all floats: " <<
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

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImage"),
    "[convert_to_ros_message] Enter the timestamp section");
  destination.header.stamp.sec = static_cast<int32_t>(
    disparity_image.timestamp->acqtime / static_cast<uint64_t>(1e9));
  destination.header.stamp.nanosec = static_cast<uint32_t>(
    disparity_image.timestamp->acqtime % static_cast<uint64_t>(1e9));
  destination.image.header = destination.header;

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

  auto maybe_disparity_image = CreateCameraMessage(source, allocator_handle, context);
  if (!maybe_disparity_image) {
    std::string error_msg =
      "[convert_to_ros_message] Failed to create CameraMessage object";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImage"), error_msg.c_str());
    throw std::runtime_error(error_msg.c_str());
  }
  auto disparity_image = maybe_disparity_image.value();

  auto video_buffer_info = disparity_image.frame->video_frame_info();

  // Copy data from Host to Device
  const cudaError_t cuda_error = cudaMemcpy2D(
    disparity_image.frame->pointer(),
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

  // Passthrough baseline, focal length,
  // min_disparity and max_disparity as an additional component to the entity
  // passthrough both as extrinsics and standalone float entities
  disparity_image.intrinsics->focal_length.x = source.f;
  disparity_image.extrinsics->translation[0] = source.t;
  *(disparity_image.entity.add<float>("t")->get()) = source.t;
  *(disparity_image.entity.add<float>("f")->get()) = source.f;
  *(disparity_image.entity.add<float>("min_disparity")->get()) = source.min_disparity;
  *(disparity_image.entity.add<float>("max_disparity")->get()) = source.max_disparity;

  // Add timestamp to the disparity_image
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  disparity_image.timestamp->acqtime = input_timestamp;

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = disparity_image.entity.eid();
  GxfEntityRefCountInc(context, disparity_image.entity.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImage"),
    "[convert_to_custom] Conversion completed (resulting handle=%ld)",
    disparity_image.entity.eid());


  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
