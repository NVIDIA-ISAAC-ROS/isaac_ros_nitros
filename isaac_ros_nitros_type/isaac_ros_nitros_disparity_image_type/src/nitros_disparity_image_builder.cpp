// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_disparity_image_type/nitros_disparity_image_builder.hpp"

#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"

namespace
{
constexpr uint64_t kNanosecondsInSeconds = 1e9;
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
  nitros_disparity_image_.width = width_;
  nitros_disparity_image_.height = height_;
  nitros_disparity_image_.step = sizeof(float) * width_;
  nitros_disparity_image_.f = f_;
  nitros_disparity_image_.t = t_;
  nitros_disparity_image_.min_disparity = min_disparity_;
  nitros_disparity_image_.max_disparity = max_disparity_;
  nitros_disparity_image_.roi = {0, 0, static_cast<uint32_t>(width_),
    static_cast<uint32_t>(height_)};

  const size_t buffer_size = nitros_disparity_image_.step * height_;
  auto & stream_pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  cudaStream_t stream = stream_pool.acquire();
  auto deleter = [stream, &stream_pool, cb = release_callback_](uint8_t * p){
      if (cb) {
        cb();
      } else if (p) {
        cudaFreeAsync(p, stream);
      }
      stream_pool.release(stream);
    };

  [[maybe_unused]] auto write_handle = nitros_disparity_image_.from_external(
      data_, buffer_size, width_, height_, nitros_disparity_image_.step,
      "32FC1", stream, deleter);
  data_ = nullptr;  // ownership transferred

  // Set timestamp from header if provided
  if (header_.stamp.sec != 0 || header_.stamp.nanosec != 0) {
    nitros_disparity_image_.timestamp_sec = header_.stamp.sec;
    nitros_disparity_image_.timestamp_nsec = header_.stamp.nanosec;
  } else {
    nitros_disparity_image_.timestamp_sec = 0;
    nitros_disparity_image_.timestamp_nsec = 0;
  }
  nitros_disparity_image_.frame_id = header_.frame_id;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImageBuilder"),
    "[Build] Disparity image built");

  return nitros_disparity_image_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
