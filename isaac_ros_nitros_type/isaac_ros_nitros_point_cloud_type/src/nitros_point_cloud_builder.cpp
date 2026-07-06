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

#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud_builder.hpp"

#include <sstream>

#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosPointCloudBuilder::NitrosPointCloudBuilder()
: nitros_point_cloud_{}
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloudBuilder"),
    "[constructor] NitrosPointCloudBuilder initialized");
}

NitrosPointCloudBuilder::NitrosPointCloudBuilder(NitrosPointCloudBuilder && other)
{
  nitros_point_cloud_ = other.nitros_point_cloud_;
  points_data_ = other.points_data_;
  num_points_ = other.num_points_;
  use_color_ = other.use_color_;
  header_ = other.header_;
  event_ = other.event_;
  release_callback_ = other.release_callback_;

  // Reset other
  other.points_data_ = nullptr;
  other.num_points_ = 0;
  other.use_color_ = false;
  other.event_ = {};
  other.release_callback_ = nullptr;
}

NitrosPointCloudBuilder & NitrosPointCloudBuilder::operator=(NitrosPointCloudBuilder && other)
{
  // In case other is this, then nothing should be done.
  if (&other == this) {
    return *this;
  }
  nitros_point_cloud_ = other.nitros_point_cloud_;
  points_data_ = other.points_data_;
  num_points_ = other.num_points_;
  use_color_ = other.use_color_;
  header_ = other.header_;
  event_ = other.event_;
  release_callback_ = other.release_callback_;

  // Reset other
  other.points_data_ = nullptr;
  other.num_points_ = 0;
  other.use_color_ = false;
  other.event_ = {};
  other.release_callback_ = nullptr;

  return *this;
}

void NitrosPointCloudBuilder::Validate()
{
  bool failure = false;
  std::stringstream error_msg;
  if (num_points_ <= 0) {
    error_msg << "Points count is not set. Call WithPoints method before Build.";
    failure = true;
  }
  if (points_data_ == nullptr) {
    if (failure) {
      error_msg << " ";
    }
    error_msg << "Points data is not set. Call WithPoints method before Build.";
    failure = true;
  }

  if (failure) {
    RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloudBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
}

NitrosPointCloudBuilder & NitrosPointCloudBuilder::WithHeader(const std_msgs::msg::Header & header)
{
  header_ = header;
  return *this;
}

NitrosPointCloudBuilder & NitrosPointCloudBuilder::WithPoints(
  const void * points_data, int32_t num_points, bool use_color)
{
  if (num_points <= 0) {
    throw std::invalid_argument(
      "[WithPoints] num_points must be greater than zero");
  }
  if (points_data == nullptr) {
    throw std::invalid_argument(
      "[WithPoints] points_data must not be null");
  }
  points_data_ = points_data;
  num_points_ = num_points;
  use_color_ = use_color;

  return *this;
}

NitrosPointCloudBuilder & NitrosPointCloudBuilder::WithEvent(cudaEvent_t event)
{
  event_ = event;
  return *this;
}

NitrosPointCloudBuilder & NitrosPointCloudBuilder::WithReleaseCallback(
  std::function<void()> release_callback)
{
  release_callback_ = release_callback;
  return *this;
}

NitrosPointCloud NitrosPointCloudBuilder::Build()
{
  Validate();

  if (event_) {
    cudaError_t cuda_error = cudaEventSynchronize(event_);
    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventSynchronize failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosPointCloudBuilder"), error_msg.str().c_str());
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
        rclcpp::get_logger("NitrosPointCloudBuilder"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  const uint32_t width = static_cast<uint32_t>(num_points_);
  const uint32_t height = 1u;
  const uint32_t point_step = use_color_ ? 16u : 12u;
  const uint32_t row_step = point_step * width;
  const size_t buffer_size = static_cast<size_t>(point_step) * width * height;

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

  [[maybe_unused]] auto write_handle = nitros_point_cloud_.from_external(
    const_cast<void *>(points_data_), buffer_size,
    width, height, point_step, row_step,
    false /* is_bigendian */, use_color_,
    cuda_stream, deleter);

  nitros_point_cloud_.frame_id = header_.frame_id;
  nitros_point_cloud_.timestamp_sec = header_.stamp.sec;
  nitros_point_cloud_.timestamp_nsec = header_.stamp.nanosec;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloudBuilder"),
    "[Build] Point cloud built with %d points, use_color: %d", num_points_, use_color_);

  points_data_ = nullptr;
  return nitros_point_cloud_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
