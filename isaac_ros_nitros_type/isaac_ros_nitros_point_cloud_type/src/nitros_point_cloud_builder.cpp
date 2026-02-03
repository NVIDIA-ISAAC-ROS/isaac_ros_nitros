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

#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud_builder.hpp"

#include <sstream>

#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/allocator.hpp"
#include "gxf/std/timestamp.hpp"
#include "messages/point_cloud_message.hpp"
#pragma GCC diagnostic pop

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
  // NOTE: we defer creation of the msg until the Build function
  // which is done to use CreatePointCloudMessage
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
  // Validate all data is present before building the NitrosPointCloud
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

  // Get allocator handle
  gxf_uid_t cid;
  GetTypeAdapterNitrosContext().getCid(
    "memory_pool", "unbounded_allocator", "nvidia::gxf::UnboundedAllocator", cid);
  auto maybe_allocator_handle =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(
      GetTypeAdapterNitrosContext().getContext(), cid);
  if (!maybe_allocator_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[Build] Failed to get allocator's handle: " <<
      GxfResultStr(maybe_allocator_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloudBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  // Create point cloud message
  auto maybe_point_cloud_message_parts = nvidia::isaac_ros::messages::CreatePointCloudMessage(
    GetTypeAdapterNitrosContext().getContext(), allocator_handle, num_points_, use_color_);
  if (!maybe_point_cloud_message_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[Build] Failed to create point cloud message: " << GxfResultStr(
      maybe_point_cloud_message_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloudBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto point_cloud_message_parts = maybe_point_cloud_message_parts.value();

  // Set point cloud info
  point_cloud_message_parts.info->use_color = use_color_;
  point_cloud_message_parts.info->is_bigendian = false;

  // Apply header (timestamp and frame_id)
  point_cloud_message_parts.timestamp->acqtime =
    header_.stamp.sec * static_cast<uint64_t>(1e9) + header_.stamp.nanosec;
  nitros_point_cloud_.frame_id = header_.frame_id;

  // Copy data to the tensor
  auto nitros_cuda_stream =
    GetTypeAdapterNitrosContext().getCudaStreamFromNitrosGraph();

  const size_t data_size = num_points_ * (use_color_ ? 4 : 3) * sizeof(float);

  auto maybe_dst = point_cloud_message_parts.points->data<float>();
  if (!maybe_dst) {
    throw std::runtime_error("[Build] Tensor returned no data pointer");
  }

  const cudaError_t cuda_error = cudaMemcpyAsync(
    *maybe_dst,
    points_data_, data_size, cudaMemcpyDeviceToDevice,
    nitros_cuda_stream);
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[Build] cudaMemcpyAsync failed for copying point cloud data: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloudBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  cudaError_t cuda_result = cudaStreamSynchronize(nitros_cuda_stream);
  if (cuda_result != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[Build] Stream was not able to be synchronized: " <<
      cudaGetErrorName(cuda_result) <<
      " (" << cudaGetErrorString(cuda_result) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloudBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  nitros_point_cloud_.handle = point_cloud_message_parts.message.eid();
  GxfEntityRefCountInc(
    GetTypeAdapterNitrosContext().getContext(),
    point_cloud_message_parts.message.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloudBuilder"),
    "[Build] Point cloud built with %d points, use_color: %d", num_points_, use_color_);

  // Free the input buffer since we copied the data
  if (release_callback_) {
    release_callback_();
    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosPointCloudBuilder"),
      "[Build] Release callback invoked to free input buffer");
  } else {
    // Default: free GPU memory with cudaFree
    cudaError_t free_error = cudaFree(const_cast<void *>(points_data_));
    if (free_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaFree failed: " <<
        cudaGetErrorName(free_error) <<
        " (" << cudaGetErrorString(free_error) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosPointCloudBuilder"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosPointCloudBuilder"),
      "[Build] Input buffer freed with cudaFree");
  }

  // Resetting data after it is done building
  points_data_ = nullptr;
  return nitros_point_cloud_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
