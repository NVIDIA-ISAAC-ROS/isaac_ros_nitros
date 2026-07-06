// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_flat_scan_type/nitros_flat_scan.hpp"

#include <cuda_runtime.h>

#include <climits>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"

namespace
{
constexpr char kLogTag[] = "NitrosFlatScan";
}  // namespace

// NitrosFlatScan (GPU buffer, SoA) -> ROS FlatScan (CPU vectors)
// D2H copy: device buffer -> host angles + ranges vectors
void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosFlatScan,
  isaac_ros_pointcloud_interfaces::msg::FlatScan>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kLogTag), "[convert_to_ros_message] Conversion started");

  const uint32_t n = source.num_beams;
  destination.angles.resize(n);
  destination.ranges.resize(n);

  if (n > 0) {
    auto stream_handle =
      nvidia::isaac_ros::nitros::CudaStreamPool::instance().get_stream_handle();
    cudaStream_t stream = stream_handle.get();

    auto read_handle = source.get_read_handle(stream);
    const uint8_t * gpu_ptr = read_handle.get_ptr();
    if (gpu_ptr == nullptr) {
      throw std::runtime_error("[convert_to_ros_message] NitrosFlatScan device pointer is nullptr");
    }

    // Stream-ordered D2H copies (buffer holds [angles | ranges] in SoA layout)
    const size_t plane_bytes = n * sizeof(float);

    cudaError_t err = cudaMemcpyAsync(
      destination.angles.data(), gpu_ptr, plane_bytes, cudaMemcpyDeviceToHost, stream);
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("[convert_to_ros_message] cudaMemcpyAsync angles failed: ") +
        cudaGetErrorString(err));
    }
    err = cudaMemcpyAsync(
      destination.ranges.data(), gpu_ptr + plane_bytes, plane_bytes,
      cudaMemcpyDeviceToHost, stream);
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("[convert_to_ros_message] cudaMemcpyAsync ranges failed: ") +
        cudaGetErrorString(err));
    }
    err = cudaStreamSynchronize(stream);
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("[convert_to_ros_message] cudaStreamSynchronize failed: ") +
        cudaGetErrorString(err));
    }
  }

  destination.range_max = source.range_max;
  destination.range_min = source.range_min;
  destination.header.stamp.sec = static_cast<int32_t>(source.timestamp_sec);
  destination.header.stamp.nanosec = source.timestamp_nsec;
  destination.header.frame_id = source.frame_id;

  RCLCPP_DEBUG(rclcpp::get_logger(kLogTag), "[convert_to_ros_message] Conversion completed");
}

// ROS FlatScan (CPU vectors) -> NitrosFlatScan (GPU buffer, SoA)
// H2D copy: host angles + ranges -> device buffer
void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosFlatScan,
  isaac_ros_pointcloud_interfaces::msg::FlatScan>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kLogTag), "[convert_to_custom] Conversion started");

  if (source.angles.size() != source.ranges.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag),
      "[convert_to_custom] angles size (%zu) != ranges size (%zu)",
      source.angles.size(), source.ranges.size());
    throw std::runtime_error("[convert_to_custom] angles/ranges size mismatch");
  }
  if (source.angles.size() > UINT32_MAX) {
    throw std::length_error(
      "[convert_to_custom] FlatScan angles size exceeds uint32_t range");
  }
  const uint32_t n = static_cast<uint32_t>(source.angles.size());
  const size_t plane_bytes = n * sizeof(float);
  const size_t total_bytes = plane_bytes * 2;

  nvidia::isaac_ros::nitros::NitrosFlatScan msg_temp;
  msg_temp.num_beams = n;
  msg_temp.range_max = source.range_max;
  msg_temp.range_min = source.range_min;

  if (n > 0) {
    auto & stream_pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
    cudaStream_t stream = stream_pool.acquire();

    uint8_t * d_ptr = nullptr;
    cudaError_t err = cudaMallocAsync(
      reinterpret_cast<void **>(&d_ptr), total_bytes, stream);
    if (err != cudaSuccess) {
      stream_pool.release(stream);
      throw std::runtime_error(
        std::string("[convert_to_custom] cudaMallocAsync failed: ") +
        cudaGetErrorString(err));
    }

    auto deleter = [&stream_pool, stream](uint8_t * p) {
        if (p) {
          cudaFreeAsync(p, stream);
        }
        stream_pool.release(stream);
      };

    // RAII guard owns (d_ptr, stream) until from_external takes over. If
    // from_external throws (e.g. bad_alloc in make_shared<NitrosBuffer>),
    // the guard runs the same cleanup the NitrosBuffer deleter would have.
    std::unique_ptr<uint8_t, decltype(deleter)> guard(d_ptr, deleter);

    // from_external binds the device buffer to msg_temp and returns a WriteHandle;
    // its dtor records a write event on `stream` that the next get_read_handle() waits on.
    auto write_handle = msg_temp.from_external(
      d_ptr, total_bytes, n, source.range_max, source.range_min, stream, deleter);

    // NitrosBuffer now owns (d_ptr, stream); its embedded deleter is the sole
    // cleanup owner from this point on. Release the guard without running it.
    (void)guard.release();

    // Stream-ordered H2D copies (angles then ranges, SoA layout)
    err = cudaMemcpyAsync(
      write_handle.get_ptr(), source.angles.data(), plane_bytes,
      cudaMemcpyHostToDevice, stream);
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("[convert_to_custom] cudaMemcpyAsync angles failed: ") +
        cudaGetErrorString(err));
    }
    err = cudaMemcpyAsync(
      write_handle.get_ptr() + plane_bytes, source.ranges.data(), plane_bytes,
      cudaMemcpyHostToDevice, stream);
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("[convert_to_custom] cudaMemcpyAsync ranges failed: ") +
        cudaGetErrorString(err));
    }
  }

  destination = std::move(msg_temp);
  destination.timestamp_sec = static_cast<uint32_t>(source.header.stamp.sec);
  destination.timestamp_nsec = source.header.stamp.nanosec;
  destination.frame_id = source.header.frame_id;

  RCLCPP_DEBUG(rclcpp::get_logger(kLogTag), "[convert_to_custom] Conversion completed");
}
