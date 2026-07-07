// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"
#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosImageBuilder::NitrosImageBuilder()
: nitros_image_{}
{
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
  nitros_image_.timestamp_sec = header.stamp.sec;
  nitros_image_.timestamp_nsec = header.stamp.nanosec;
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

NitrosImageBuilder & NitrosImageBuilder::WithReleaseCallback(std::function<void()> release_callback)
{
  release_callback_ = release_callback;
  return *this;
}

NitrosImage NitrosImageBuilder::Build()
{
  Validate();

  // If CUDA event provided, synchronize on that event before building
  if (event_) {
    cudaError_t cuda_error = cudaEventSynchronize(event_);
    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventSynchronize failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosImageBuilder"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    cuda_error = cudaEventDestroy(event_);
    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventDestroy failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosImageBuilder"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // Compute step size based on encoding
  uint32_t step = 0;
  if (encoding_ == "rgb8" || encoding_ == "bgr8") {
    step = width_ * 3;
  } else if (encoding_ == "rgba8" || encoding_ == "bgra8") {
    step = width_ * 4;
  } else if (encoding_ == "rgb16" || encoding_ == "bgr16") {
    step = width_ * 6;
  } else if (encoding_ == "mono8") {
    step = width_;
  } else if (encoding_ == "mono16") {
    step = width_ * 2;
  } else if (encoding_ == "16UC1") {
    step = width_ * 2;
  } else if (encoding_ == "32FC1") {
    step = width_ * 4;
  } else if (encoding_ == "32FC3") {
    step = width_ * 12;
  } else if (encoding_ == "32FC4") {
    step = width_ * 16;
  } else if (encoding_ == "nv12" || encoding_ == "nv24") {
    step = width_;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImageBuilder"), "Unsupported encoding [%s].", encoding_.c_str());
    throw std::runtime_error("[NitrosImageBuilder] Unsupported encoding.");
  }

  // Compute total buffer size
  size_t buffer_size = step * height_;
  if (encoding_ == "nv12") {
    buffer_size = buffer_size * 3 / 2;
  } else if (encoding_ == "nv24") {
    buffer_size = buffer_size * 3;
  }

  auto & stream_pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  cudaStream_t cuda_stream = stream_pool.acquire();

  auto deleter = [release_callback = release_callback_, &stream_pool, cuda_stream](uint8_t * p) {
      if (p) {
        if (release_callback) {
          release_callback();
        } else {
          cudaFreeAsync(p, cuda_stream);
        }
      }
      stream_pool.release(cuda_stream);
    };

  [[maybe_unused]] auto write_handle = nitros_image_.from_external(
    data_, buffer_size, width_, height_, step, encoding_, cuda_stream, deleter);

  RCLCPP_DEBUG(rclcpp::get_logger("NitrosImageBuilder"), "[Build] Image built");

  data_ = nullptr;
  return nitros_image_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
