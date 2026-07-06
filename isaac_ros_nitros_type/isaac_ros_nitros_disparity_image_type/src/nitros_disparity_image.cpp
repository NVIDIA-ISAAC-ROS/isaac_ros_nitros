// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
#include "isaac_ros_nitros_disparity_image_type/nitros_disparity_image.hpp"

namespace
{
size_t nitros_compute_total_disparity_image_bytes(
  uint32_t width,
  uint32_t height)
{
  return width * height * sizeof(float);
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

  // RAII handle: declared BEFORE read_handle so reverse-destruction order
  // lets read_handle record its CUDA event before the stream returns to the
  // pool. Returns the stream to the pool on every exit path (success and
  // exception), fixing a leak where the success path never released the
  // stream and the pool was permanently exhausted after ~32 messages.
  auto stream_handle =
    nvidia::isaac_ros::nitros::CudaStreamPool::instance().get_stream_handle();
  cudaStream_t stream = stream_handle.get();

  // Use read handle for proper event synchronization
  auto read_handle = source.get_read_handle(stream);
  const uint8_t * dev_ptr = read_handle.get_ptr();
  if (dev_ptr == nullptr) {
    throw std::invalid_argument("NitrosDisparityImage device pointer is nullptr");
  }

  const size_t total_bytes = nitros_compute_total_disparity_image_bytes(
    source.width, source.height);

  destination.image.data.resize(total_bytes);
  cudaError_t cuda_err = cudaMemcpyAsync(
    destination.image.data.data(), dev_ptr, total_bytes,
    cudaMemcpyDeviceToHost, stream);
  if (cuda_err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaMemcpyAsync D2H failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::invalid_argument(error_msg.str().c_str());
  }

  // Ensure all device-to-host copies are complete before publishing the CPU message
  cuda_err = cudaStreamSynchronize(stream);
  if (cuda_err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaStreamSynchronize failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Bigendian or not
  destination.image.is_bigendian = 0;
  destination.image.height = source.height;
  destination.image.width = source.width;
  destination.image.encoding = source.encoding;
  destination.image.step = source.step;

  // disparity image parameters
  destination.f = source.f;
  destination.t = source.t;
  destination.min_disparity = source.min_disparity;
  destination.max_disparity = source.max_disparity;
  destination.valid_window.x_offset = source.roi[0];
  destination.valid_window.y_offset = source.roi[1];
  destination.valid_window.width = source.roi[2];
  destination.valid_window.height = source.roi[3];
  destination.delta_d = source.delta_disparity;

  destination.header.stamp.sec = source.timestamp_sec;
  destination.header.stamp.nanosec = source.timestamp_nsec;
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

  auto & stream_pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  cudaStream_t stream = stream_pool.acquire();

  nvidia::isaac_ros::nitros::NitrosDisparityImage disparity_image;
  const size_t total_bytes = nitros_compute_total_disparity_image_bytes(source.image.width,
    source.image.height);
  uint8_t * dptr = nullptr;
  cudaError_t cuda_err = cudaMallocAsync(reinterpret_cast<void **>(&dptr), total_bytes, stream);
  if (cuda_err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMallocAsync failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
    cudaStreamDestroy(stream);
    throw std::runtime_error(error_msg.str().c_str());
  }

  {
    auto deleter = [stream, &stream_pool](uint8_t * p){
        if (p) {
          cudaFreeAsync(p, stream);
        }
        stream_pool.release(stream);
      };

    auto write_handle = disparity_image.from_external(
      dptr, total_bytes, source.image.width, source.image.height, source.image.step,
      source.image.encoding, stream, deleter);

    cuda_err = cudaMemcpyAsync(write_handle.get_ptr(), source.image.data.data(), total_bytes,
      cudaMemcpyHostToDevice, stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_custom] cudaMemcpyAsync H2D failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosDisparityImage"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  destination = std::move(disparity_image);
  destination.width = source.image.width;
  destination.height = source.image.height;
  destination.step = source.image.step;
  destination.encoding = source.image.encoding;

  // Passthrough baseline, focal length,
  // min_disparity and max_disparity
  destination.f = source.f;
  destination.t = source.t;
  destination.min_disparity = source.min_disparity;
  destination.max_disparity = source.max_disparity;
  destination.roi[0] = source.valid_window.x_offset;
  destination.roi[1] = source.valid_window.y_offset;
  destination.roi[2] = source.valid_window.width;
  destination.roi[3] = source.valid_window.height;
  destination.delta_disparity = source.delta_d;

  destination.timestamp_sec = source.header.stamp.sec;
  destination.timestamp_nsec = source.header.stamp.nanosec;
  destination.frame_id = source.header.frame_id.c_str();

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDisparityImage"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
