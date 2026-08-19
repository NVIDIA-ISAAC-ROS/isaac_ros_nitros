// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_HPP_
#define ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosFlatScan
 *   ROS type:    isaac_ros_pointcloud_interfaces::msg::FlatScan
 *
 * GPU buffer layout (SoA, float):
 *   [angle_0 .. angle_{N-1}  range_0 .. range_{N-1}]
 *   Total size: 2 * num_beams * sizeof(float)
 */

#include <cuda_runtime.h>

#include <functional>
#include <memory>
#include <string>
#include <cstdint>
#include <utility>
#include <vector>

#include "rclcpp/type_adapter.hpp"
#include "isaac_ros_pointcloud_interfaces/msg/flat_scan.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Forward declaration
class NitrosFlatScan;

// Formats
struct nitros_flat_scan_t
{
  using MsgT = NitrosFlatScan;
  static const inline std::string supported_type_name = "nitros_flat_scan";
};

class NitrosFlatScan : public NitrosTypeBase
{
public:
  // Standard ROS2 message pointer type aliases (required by message_filters)
  using SharedPtr = std::shared_ptr<NitrosFlatScan>;
  using ConstSharedPtr = std::shared_ptr<const NitrosFlatScan>;
  using UniquePtr = std::unique_ptr<NitrosFlatScan>;
  using ConstUniquePtr = std::unique_ptr<const NitrosFlatScan>;
  using WeakPtr = std::weak_ptr<NitrosFlatScan>;
  using ConstWeakPtr = std::weak_ptr<const NitrosFlatScan>;
  using ConstPtr = const NitrosFlatScan *;

  NitrosFlatScan()
  : NitrosTypeBase() {}
  explicit NitrosFlatScan(const NitrosTypeBase & base)
  : NitrosTypeBase(base) {}

  // Timestamp accessors (override from NitrosTypeBase)
  uint32_t get_timestamp_sec() const override {return timestamp_sec;}
  uint32_t get_timestamp_nsec() const override {return timestamp_nsec;}
  const std::string & get_frame_id() const {return frame_id;}
  void set_timestamp_sec(uint32_t sec) override {timestamp_sec = sec;}
  void set_timestamp_nsec(uint32_t nsec) override {timestamp_nsec = nsec;}

  /// Get read handle for consuming beam data on the GPU.
  /// The device pointer points to SoA layout:
  ///   [angles (num_beams floats)] [ranges (num_beams floats)]
  nvidia::isaac_ros::nitros::ReadHandle get_read_handle(cudaStream_t stream) const
  {
    return buffer_->get_read_handle(stream);
  }

  /// Allocate beam buffer from a CUDA memory pool and return a write handle.
  /// The caller writes SoA beam data (angles then ranges) through the handle.
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_pool(
    nvidia::isaac_ros::nitros::CUDAMemoryPool & pool,
    uint32_t num_beams_in,
    float range_max_in,
    float range_min_in,
    cudaStream_t stream)
  {
    uint8_t * ptr = nullptr;
    const cudaError_t err = pool.acquire(&ptr);
    if (err != cudaSuccess) {
      throw std::runtime_error("CUDAMemoryPool exhausted for NitrosFlatScan");
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(
      ptr, pool.block_size(), pool.deleter());
    num_beams = num_beams_in;
    range_max = range_max_in;
    range_min = range_min_in;
    return buffer_->get_write_handle(stream);
  }

  /// Wrap an external device pointer and return a write handle.
  /// The buffer takes ownership via the provided deleter (defaults to cudaFree).
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_external(
    void * device_ptr,
    size_t bytes,
    uint32_t num_beams_in,
    float range_max_in,
    float range_min_in,
    cudaStream_t stream,
    std::function<void(uint8_t *)> deleter = nullptr)
  {
    if (!deleter) {
      deleter = [](uint8_t * p) {if (p) {cudaFree(p);}};
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(
      device_ptr, bytes, std::move(deleter));
    num_beams = num_beams_in;
    range_max = range_max_in;
    range_min = range_min_in;
    return buffer_->get_write_handle(stream);
  }

  /// Size of the GPU buffer in bytes: 2 * num_beams * sizeof(float)
  size_t buffer_size() const {return static_cast<size_t>(num_beams) * 2 * sizeof(float);}

  // Core message data (public like ROS2 messages)
  uint32_t num_beams{0};
  float range_max{0.0f};
  float range_min{0.0f};
  uint32_t timestamp_sec{0};
  uint32_t timestamp_nsec{0};
  std::string frame_id;

private:
  friend class nvidia::isaac_ros::nitros::NitrosBufferAccessor<NitrosFlatScan>;
  friend struct rclcpp::TypeAdapter<NitrosFlatScan,
    isaac_ros_pointcloud_interfaces::msg::FlatScan>;

  std::shared_ptr<nvidia::isaac_ros::nitros::NitrosBuffer> buffer_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosFlatScan,
  isaac_ros_pointcloud_interfaces::msg::FlatScan>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosFlatScan;
  using ros_message_type = isaac_ros_pointcloud_interfaces::msg::FlatScan;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosFlatScan,
  isaac_ros_pointcloud_interfaces::msg::FlatScan);

#endif  // ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_HPP_
