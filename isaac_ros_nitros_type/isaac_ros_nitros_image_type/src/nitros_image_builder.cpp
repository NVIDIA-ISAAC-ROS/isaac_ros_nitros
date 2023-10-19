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
#include <vector>

#include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/multimedia/video.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"


namespace
{
constexpr uint64_t kNanosecondsInSeconds = 1e9;

template<VideoFormat T>
void create_image(
  const uint32_t width, const uint32_t height,
  nvidia::gxf::Expected<nvidia::gxf::Entity> & msg_entity, void * data, const std::string & name)
{
  if (width % 2 != 0 || height % 2 != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImageBuilder"),
      "[create_image] Image width/height must be even for creation of gxf::VideoBuffer");
    throw std::runtime_error("[create_image] Odd Image width or height.");
  }

  auto gxf_image = msg_entity->add<nvidia::gxf::VideoBuffer>(name.c_str());
  NoPaddingColorPlanes<T> nopadding_planes(width);
  nvidia::gxf::VideoFormatSize<T> format_size;
  uint64_t size = format_size.size(width, height, nopadding_planes.planes);

  std::vector<nvidia::gxf::ColorPlane> color_planes{nopadding_planes.planes.begin(),
    nopadding_planes.planes.end()};

  constexpr auto surface_layout = nvidia::gxf::SurfaceLayout::GXF_SURFACE_LAYOUT_PITCH_LINEAR;
  constexpr auto storage_type = nvidia::gxf::MemoryStorageType::kDevice;

  nvidia::gxf::VideoBufferInfo buffer_info{width, height, T, color_planes, surface_layout};


  gxf_image.value()->wrapMemory(
    buffer_info, size, storage_type, data,
    [](void * ptr) {
      RCLCPP_INFO(
        rclcpp::get_logger("NitrosImageBuilder"),
        "[create_image] Freed the cuda memory [%p]", ptr);
      cudaFree(ptr);
      return nvidia::gxf::Success;
    });
}
}  // namespace

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosImageBuilder::NitrosImageBuilder()
: nitros_image_{}
{
  auto message = gxf::Entity::New(GetTypeAdapterNitrosContext().getContext());
  if (!message) {
    std::stringstream error_msg;
    error_msg <<
      "[constructor] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImageBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  nitros_image_.handle = message->eid();
  GxfEntityRefCountInc(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(), message->eid());

  RCLCPP_DEBUG(rclcpp::get_logger("NitrosImageBuilder"), "[constructor] NitrosImage initialized");
}

NitrosImageBuilder::NitrosImageBuilder(NitrosImageBuilder && other)
{
  nitros_image_ = other.nitros_image_;
  encoding_ = other.encoding_;
  height_ = other.height_;
  width_ = other.width_;
  data_ = other.data_;
  event_ = other.event_;

  // Reset other
  other.encoding_ = "";
  other.height_ = 0;
  other.width_ = 0;
  other.data_ = nullptr;
}

NitrosImageBuilder & NitrosImageBuilder::operator=(NitrosImageBuilder && other)
{
  // In case other is this, then nothing should be done.
  if (&other == this) {
    return *this;
  }
  nitros_image_ = other.nitros_image_;
  encoding_ = other.encoding_;
  height_ = other.height_;
  width_ = other.width_;
  data_ = other.data_;
  event_ = other.event_;
  return *this;
}

void NitrosImageBuilder::Validate()
{
  bool failure = false;
  std::stringstream error_msg;
  if (height_ == 0 || width_ == 0) {
    error_msg << "Dimensions are not set! Call WithDimension method before Build. \n";
    failure = true;
  }
  if (encoding_ == "") {
    error_msg << "Encoding is not set! Call WithEncoding method before Build. \n";
    failure = true;
  }
  if (data_ == nullptr) {
    error_msg << "Data buffer is not set! Call WithGpuData method before Build. \n";
    failure = true;
  }

  if (failure) {
    RCLCPP_ERROR(rclcpp::get_logger("NitrosImageBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
}

NitrosImageBuilder & NitrosImageBuilder::WithHeader(std_msgs::msg::Header header)
{
  auto message = gxf::Entity::Shared(
    GetTypeAdapterNitrosContext().getContext(), nitros_image_.handle);

  // Set timestamp
  auto output_timestamp = message->add<gxf::Timestamp>("timestamp");
  if (!output_timestamp) {
    std::stringstream error_msg;
    error_msg << "[WithHeader] Failed to add a timestamp component to message: " <<
      GxfResultStr(output_timestamp.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImageBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  output_timestamp.value()->acqtime = header.stamp.sec * kNanosecondsInSeconds +
    header.stamp.nanosec;

  // Set frame ID
  nitros_image_.frame_id = header.frame_id;

  return *this;
}

NitrosImageBuilder & NitrosImageBuilder::WithEncoding(std::string encoding)
{
  encoding_ = encoding;
  return *this;
}

NitrosImageBuilder & NitrosImageBuilder::WithDimensions(uint32_t height, uint32_t width)
{
  height_ = height;
  width_ = width;
  return *this;
}

NitrosImageBuilder & NitrosImageBuilder::WithGpuData(void * data)
{
  data_ = data;
  return *this;
}

NitrosImage NitrosImageBuilder::Build()
{
  // Validate all data is present before building the NitrosImage
  Validate();

  auto message = gxf::Entity::Shared(
    GetTypeAdapterNitrosContext().getContext(), nitros_image_.handle);

  // If CUDA event provided, synchronize on that event before building
  if (event_) {
    cudaError_t cuda_error = cudaEventSynchronize(event_);

    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventSynchronize failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosImageBuilder"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    cuda_error = cudaEventDestroy(event_);
    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventDestroy failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosImageBuilder"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  auto color_fmt = g_ros_to_gxf_video_format.find(encoding_);
  if (color_fmt == std::end(g_ros_to_gxf_video_format)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImageBuilder"), "Unsupported encoding [%s].", encoding_.c_str());
    throw std::runtime_error("[NitrosImageBuilder] Unsupported encoding.");
  }

  switch (color_fmt->second) {
    case VideoFormat::GXF_VIDEO_FORMAT_RGB:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_RGB>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_RGBA:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_RGBA>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_RGB16:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_RGB16>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGR:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_BGR>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGRA:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_BGRA>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGR16:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_BGR16>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_GRAY:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_GRAY>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_GRAY16:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_GRAY16>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_GRAY32:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_GRAY32>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_NV24:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_NV24>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_NV12:
      create_image<VideoFormat::GXF_VIDEO_FORMAT_NV12>(
        width_, height_, message, data_, nitros_image_.frame_id);
      break;

    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosImageBuilder"), "Unsupported encoding [%s].", encoding_.c_str());
      throw std::runtime_error("[convert_to_custom] Unsupported encoding from ROS.");
      break;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("NitrosImageBuilder"), "[Build] Image built");

  // Reseting data after it is done building
  data_ = nullptr;
  return nitros_image_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
