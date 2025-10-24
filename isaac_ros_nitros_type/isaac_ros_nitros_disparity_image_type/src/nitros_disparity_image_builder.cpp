// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <array>

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

#include "isaac_ros_nitros_disparity_image_type/nitros_disparity_image_builder.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

namespace
{
constexpr uint64_t kNanosecondsInSeconds = 1e9;

// NoPaddingColorPlanes specialization for D32F format
template<nvidia::gxf::VideoFormat T>
struct NoPaddingColorPlanes {};

template<>
struct NoPaddingColorPlanes<nvidia::gxf::VideoFormat::GXF_VIDEO_FORMAT_D32F>
{
  explicit NoPaddingColorPlanes(uint32_t width)
  : planes({nvidia::gxf::ColorPlane("D", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

nvidia::gxf::Expected<void> ReleaseImageCallback(
  std::function<void()> release_callback,
  void * ptr)
{
  if (release_callback) {
    release_callback();
  } else {
    cudaFree(ptr);
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImageBuilder"),
    "[ReleaseImageCallback] Released the cuda memory [%p]", ptr);
  return nvidia::gxf::Success;
}

nvidia::gxf::Expected<nvidia::isaac::CameraMessageParts> CreateDisparityImage(
  const uint32_t width, const uint32_t height,
  gxf_context_t context,
  void * data,
  std::function<void()> release_callback)
{
  if (width % 2 != 0 || height % 2 != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImageBuilder"),
      "[CreateDisparityImage] Image width/height must be even for creation of gxf::VideoBuffer");
    throw std::runtime_error("[CreateDisparityImage] Odd Image width or height.");
  }

  // Create proper CameraMessage structure
  auto maybe_camera_message = nvidia::isaac::InitializeCameraMessage(context);
  if (!maybe_camera_message) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImageBuilder"),
      "[CreateDisparityImage] Failed to initialize CameraMessage");
    return nvidia::gxf::ForwardError(maybe_camera_message);
  }

  auto camera_message = maybe_camera_message.value();

  // Setup VideoBuffer with the provided data
  NoPaddingColorPlanes<nvidia::gxf::VideoFormat::GXF_VIDEO_FORMAT_D32F> nopadding_planes(width);
  nvidia::gxf::VideoFormatSize<nvidia::gxf::VideoFormat::GXF_VIDEO_FORMAT_D32F> format_size;
  uint64_t size = format_size.size(width, height, nopadding_planes.planes);

  std::vector<nvidia::gxf::ColorPlane> color_planes{nopadding_planes.planes.begin(),
    nopadding_planes.planes.end()};

  constexpr auto surface_layout = nvidia::gxf::SurfaceLayout::GXF_SURFACE_LAYOUT_PITCH_LINEAR;
  constexpr auto storage_type = nvidia::gxf::MemoryStorageType::kDevice;

  nvidia::gxf::VideoBufferInfo buffer_info{
    width, height,
    nvidia::gxf::VideoFormat::GXF_VIDEO_FORMAT_D32F,
    color_planes,
    surface_layout
  };

  camera_message.frame->wrapMemory(
    buffer_info, size, storage_type, data,
    std::bind(&ReleaseImageCallback, release_callback, std::placeholders::_1));

  return camera_message;
}
}  // namespace

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosDisparityImageBuilder::NitrosDisparityImageBuilder()
: nitros_disparity_image_{}
{
  // Entity will be created during Build()
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImageBuilder"),
    "[constructor] NitrosDisparityImageBuilder initialized");
}

NitrosDisparityImageBuilder::NitrosDisparityImageBuilder(NitrosDisparityImageBuilder && other)
{
  nitros_disparity_image_ = other.nitros_disparity_image_;
  height_ = other.height_;
  width_ = other.width_;
  data_ = other.data_;
  release_callback_ = other.release_callback_;
  f_ = other.f_;
  t_ = other.t_;
  min_disparity_ = other.min_disparity_;
  max_disparity_ = other.max_disparity_;
  header_ = other.header_;

  // Reset other
  other.height_ = 0;
  other.width_ = 0;
  other.data_ = nullptr;
  other.release_callback_ = nullptr;
  other.f_ = 0.0f;
  other.t_ = 0.0f;
  other.min_disparity_ = 0.0f;
  other.max_disparity_ = 0.0f;
  other.header_ = std_msgs::msg::Header{};
}

NitrosDisparityImageBuilder &
NitrosDisparityImageBuilder::operator=(NitrosDisparityImageBuilder && other)
{
  // In case other is this, then nothing should be done.
  if (&other == this) {
    return *this;
  }
  nitros_disparity_image_ = other.nitros_disparity_image_;
  height_ = other.height_;
  width_ = other.width_;
  data_ = other.data_;
  release_callback_ = other.release_callback_;
  f_ = other.f_;
  t_ = other.t_;
  min_disparity_ = other.min_disparity_;
  max_disparity_ = other.max_disparity_;
  header_ = other.header_;

  // Reset other
  other.height_ = 0;
  other.width_ = 0;
  other.data_ = nullptr;
  other.release_callback_ = nullptr;
  other.f_ = 0.0f;
  other.t_ = 0.0f;
  other.min_disparity_ = 0.0f;
  other.max_disparity_ = 0.0f;
  other.header_ = std_msgs::msg::Header{};

  return *this;
}

NitrosDisparityImageBuilder & NitrosDisparityImageBuilder::WithHeader(std_msgs::msg::Header header)
{
  // Store header information to be applied during Build()
  header_ = header;
  nitros_disparity_image_.frame_id = header.frame_id;

  return *this;
}

NitrosDisparityImageBuilder &
NitrosDisparityImageBuilder::WithDimensions(uint32_t height, uint32_t width)
{
  height_ = height;
  width_ = width;
  return *this;
}

NitrosDisparityImageBuilder & NitrosDisparityImageBuilder::WithGpuData(void * data)
{
  data_ = data;
  return *this;
}

NitrosDisparityImageBuilder & NitrosDisparityImageBuilder::WithReleaseCallback(
  std::function<void()> release_callback)
{
  release_callback_ = release_callback;
  return *this;
}

NitrosDisparityImageBuilder &
NitrosDisparityImageBuilder::WithDisparityParameters(
  float f, float t, float min_disparity, float max_disparity)
{
  f_ = f;
  t_ = t;
  min_disparity_ = min_disparity;
  max_disparity_ = max_disparity;
  return *this;
}

void NitrosDisparityImageBuilder::Validate()
{
  bool failure = false;
  std::stringstream error_msg;
  if (height_ == 0 || width_ == 0) {
    error_msg << "Dimensions are not set! Call WithDimension method before Build. \n";
    failure = true;
  }
  if (data_ == nullptr) {
    error_msg << "Data buffer is not set! Call WithGpuData method before Build. \n";
    failure = true;
  }

  if (failure) {
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImageBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
}

NitrosDisparityImage NitrosDisparityImageBuilder::Build()
{
  // Validate all data is present before building the NitrosDisparityImage
  Validate();

  auto context = GetTypeAdapterNitrosContext().getContext();

  // Create the CameraMessage structure with proper components
  auto maybe_camera_message = CreateDisparityImage(
    width_, height_, context, data_, release_callback_);

  if (!maybe_camera_message) {
    std::stringstream error_msg;
    error_msg << "[Build] Failed to create CameraMessage: " <<
      GxfResultStr(maybe_camera_message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDisparityImageBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  auto camera_message = maybe_camera_message.value();

  // Set disparity parameters in intrinsics and extrinsics for compatibility with disparity_to_depth
  camera_message.intrinsics->focal_length.x = f_;
  camera_message.extrinsics->translation[0] = t_;

  // Add disparity parameters as float components for backward compatibility
  *(camera_message.entity.add<float>("f")->get()) = f_;
  *(camera_message.entity.add<float>("t")->get()) = t_;
  *(camera_message.entity.add<float>("min_disparity")->get()) = min_disparity_;
  *(camera_message.entity.add<float>("max_disparity")->get()) = max_disparity_;

  // Set timestamp from header if provided
  if (header_.stamp.sec != 0 || header_.stamp.nanosec != 0) {
    uint64_t input_timestamp = header_.stamp.sec * kNanosecondsInSeconds + header_.stamp.nanosec;
    camera_message.timestamp->acqtime = input_timestamp;
  }

  // Set the handle and frame_id
  nitros_disparity_image_.handle = camera_message.entity.eid();
  GxfEntityRefCountInc(context, camera_message.entity.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImageBuilder"),
    "[Build] Disparity image built");

  // Resetting data after it is done building
  data_ = nullptr;
  return nitros_disparity_image_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
