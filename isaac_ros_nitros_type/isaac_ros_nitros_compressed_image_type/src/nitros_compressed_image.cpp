// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_compressed_image_type/nitros_compressed_image.hpp"

#include <cuda_runtime.h>

#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"


void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosCompressedImage,
  sensor_msgs::msg::CompressedImage>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  destination.format = source.get_format();
  destination.data.resize(source.size());

  auto & pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  auto stream_handle = pool.get_stream_handle();
  cudaStream_t stream = stream_handle.get();

  {
    auto read_handle = source.get_read_handle(stream);
    cudaError_t cuda_err = cudaMemcpyAsync(
      destination.data.data(),
      read_handle.get_ptr(),
      source.size(),
      cudaMemcpyDeviceToHost,
      stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_ros_message] cudaMemcpyAsync D2H failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
    cuda_err = cudaStreamSynchronize(stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_ros_message] cudaStreamSynchronize failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  destination.header.stamp.sec = source.timestamp_sec;
  destination.header.stamp.nanosec = source.timestamp_nsec;
  destination.header.frame_id = source.frame_id;
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosCompressedImage,
  sensor_msgs::msg::CompressedImage>::convert_to_custom(
  const ros_message_type & source, custom_type & destination)
{
  size_t size = source.data.size();

  auto & pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  cudaStream_t stream = pool.acquire();

  nvidia::isaac_ros::nitros::NitrosCompressedImage msg_temp;
  uint8_t * device_ptr = nullptr;
  cudaError_t cuda_err = cudaMallocAsync(reinterpret_cast<void **>(&device_ptr), size, stream);
  if (cuda_err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMallocAsync failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
    pool.release(stream);
    throw std::runtime_error(error_msg.str().c_str());
  }

  {
    auto deleter = [&pool, stream](uint8_t * p) {
        if (p) {
          cudaFreeAsync(p, stream);
        }
        pool.release(stream);
      };

    auto write_handle = msg_temp.from_external(
      device_ptr, size, size, source.format, stream, deleter);

    cuda_err = cudaMemcpyAsync(
      write_handle.get_ptr(), source.data.data(), size,
      cudaMemcpyHostToDevice, stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_custom] cudaMemcpyAsync H2D failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosCompressedImage"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  destination = std::move(msg_temp);
  destination.timestamp_sec = source.header.stamp.sec;
  destination.timestamp_nsec = source.header.stamp.nanosec;
  destination.frame_id = source.header.frame_id;
}
