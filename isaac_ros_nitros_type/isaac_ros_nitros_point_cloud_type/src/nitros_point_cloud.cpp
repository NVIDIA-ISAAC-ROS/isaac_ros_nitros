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

#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "isaac_ros_common/cuda_stream.hpp"
#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"

#include "rclcpp/rclcpp.hpp"

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPointCloud,
  sensor_msgs::msg::PointCloud2>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosPointCloud::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
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
    throw std::invalid_argument("NitrosPointCloud device pointer is nullptr");
  }

  destination.height = source.height;
  destination.width = source.width;
  destination.point_step = source.point_step;
  destination.row_step = source.row_step;
  destination.is_bigendian = source.is_bigendian;
  destination.is_dense = false;

  sensor_msgs::PointCloud2Modifier pc2_modifier(destination);
  if (source.use_color) {
    // Data format: x,y,z,rgb; 16 bytes per point
    pc2_modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  } else {
    // Data format: x,y,z; 12 bytes per point
    pc2_modifier.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  }

  destination.data.resize(destination.row_step * destination.height);
  const cudaError_t cuda_error = cudaMemcpyAsync(
    destination.data.data(),
    dev_ptr, destination.row_step * destination.height,
    cudaMemcpyDeviceToHost, stream);
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaMemcpy failed for conversion from "
      "NitrosPointCloud to sensor_msgs::msg::PointCloud2: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  cudaError_t cuda_err = cudaStreamSynchronize(stream);
  if (cuda_err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg << "[convert_to_ros_message] cudaStreamSynchronize failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_ros_message] "
    "height: %u, width: %u, row_step: %u, point_step: %u, x_offset: %u, y_offset: %u, z_offset: %u",
    destination.height,
    destination.width,
    destination.row_step,
    destination.point_step,
    destination.fields[0].offset,
    destination.fields[1].offset,
    destination.fields[2].offset);

  // Populate timestamp information back into ROS header
  destination.header.stamp.sec = source.timestamp_sec;
  destination.header.stamp.nanosec = source.timestamp_nsec;
  destination.header.frame_id = source.frame_id;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPointCloud,
  sensor_msgs::msg::PointCloud2>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosPointCloud::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_custom] Conversion started");

  auto & stream_pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  cudaStream_t stream = stream_pool.acquire();

  const int32_t height = source.height;
  const int32_t width = source.width;
  const int32_t point_float_count = source.point_step / sizeof(float);
  const bool use_color = (point_float_count == 4);  // {x, y, z, RGB}
  const size_t total_bytes = height * width * point_float_count * sizeof(float);

  nvidia::isaac_ros::nitros::NitrosPointCloud point_cloud;
  uint8_t * dptr = nullptr;
  cudaError_t cuda_err = cudaMallocAsync(reinterpret_cast<void **>(&dptr), total_bytes, stream);
  if (cuda_err != cudaSuccess) {
    stream_pool.release(stream);
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMallocAsync failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  {
    auto deleter = [stream, &stream_pool](uint8_t * p){
        if (p) {
          cudaFreeAsync(p, stream);
        }
        if (stream) {
          stream_pool.release(stream);
        }
      };

    auto write_handle = point_cloud.from_external(
      dptr, total_bytes, width, height, source.point_step, source.row_step, source.is_bigendian,
      use_color, stream, deleter);

    cuda_err = cudaMemcpyAsync(write_handle.get_ptr(), source.data.data(), total_bytes,
      cudaMemcpyHostToDevice, stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_custom] cudaMemcpyAsync H2D failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  destination = std::move(point_cloud);
  destination.width = source.width;
  destination.height = source.height;
  destination.point_step = source.point_step;
  destination.row_step = source.row_step;
  destination.is_bigendian = source.is_bigendian;
  destination.use_color = use_color;

  destination.timestamp_sec = source.header.stamp.sec;
  destination.timestamp_nsec = source.header.stamp.nanosec;
  destination.frame_id = source.header.frame_id;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
