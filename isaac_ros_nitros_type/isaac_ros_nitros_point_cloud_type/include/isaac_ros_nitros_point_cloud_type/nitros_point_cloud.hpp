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

#ifndef ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_HPP_
#define ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosPointCloud
 *   ROS type:    sensor_msgs::msg::PointCloud2
 */

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type forward declaration
struct NitrosPointCloud;

class NitrosPointCloud : public NitrosTypeBase
{
public:
  // Standard ROS2 message pointer type aliases (required by message_filters)
  using SharedPtr = std::shared_ptr<NitrosPointCloud>;
  using ConstSharedPtr = std::shared_ptr<const NitrosPointCloud>;
  using UniquePtr = std::unique_ptr<NitrosPointCloud>;
  using ConstUniquePtr = std::unique_ptr<const NitrosPointCloud>;
  using WeakPtr = std::weak_ptr<NitrosPointCloud>;
  using ConstWeakPtr = std::weak_ptr<const NitrosPointCloud>;
  using ConstPtr = const NitrosPointCloud *;

  NitrosPointCloud()
  : NitrosTypeBase()
  {
    handle = -1;
  }
  explicit NitrosPointCloud(const NitrosTypeBase & base)
  : NitrosTypeBase(base) {}

  // Message metadata accessors
  uint32_t get_width() const {return width;}
  uint32_t get_height() const {return height;}
  uint32_t get_point_step() const {return point_step;}

  // Timestamp accessors
  uint32_t get_timestamp_sec() const override {return timestamp_sec;}
  uint32_t get_timestamp_nsec() const override {return timestamp_nsec;}
  const std::string & get_frame_id() const {return frame_id;}

  // Timestamp setters (override virtual methods from NitrosTypeBase)
  void set_timestamp_sec(uint32_t sec) override {timestamp_sec = sec;}
  void set_timestamp_nsec(uint32_t nsec) override {timestamp_nsec = nsec;}

  // Get read handle for consuming image data on the specified stream
  nvidia::isaac_ros::nitros::ReadHandle get_read_handle(cudaStream_t stream) const
  {
    return buffer_->get_read_handle(stream);
  }

  // Initialize from pool storage and return write handle for producer
  // Memory is acquired from the pool and will be recycled when buffer is destroyed
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_pool(
    nvidia::isaac_ros::nitros::CUDAMemoryPool & pool,
    uint32_t width,
    uint32_t height,
    uint32_t point_step,
    uint32_t row_step,
    bool is_bigendian,
    bool use_color,
    cudaStream_t stream)
  {
    uint8_t * ptr = nullptr;
    const cudaError_t acquire_err = pool.acquire(&ptr);
    if (acquire_err != cudaSuccess) {
      throw std::runtime_error("CUDAMemoryPool exhausted");
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(ptr, pool.block_size(),
            pool.deleter());
    this->width = width;
    this->height = height;
    this->point_step = point_step;
    this->row_step = row_step;
    this->is_bigendian = is_bigendian;
    this->use_color = use_color;
    handle = -1;
    return buffer_->get_write_handle(stream);
  }

  // Initialize from an external device pointer and return write handle for producer
  // Memory ownership is transferred to the buffer with the provided deleter
  // If no deleter provided, defaults to cudaFree
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_external(
    void * device_ptr,
    size_t bytes,
    uint32_t width,
    uint32_t height,
    uint32_t point_step,
    uint32_t row_step,
    bool is_bigendian,
    bool use_color,
    cudaStream_t stream,
    std::function<void(uint8_t *)> deleter = nullptr)
  {
    if (!deleter) {
      deleter = [](uint8_t * p){if (p) {cudaFree(p);}};
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(device_ptr, bytes,
            std::move(deleter));
    this->width = width;
    this->height = height;
    this->point_step = point_step;
    this->row_step = row_step;
    this->is_bigendian = is_bigendian;
    this->use_color = use_color;
    handle = -1;
    return buffer_->get_write_handle(stream);
  }

  // Core message data (public like ROS2 messages)
  uint32_t width{0};
  uint32_t height{0};
  uint32_t point_step{0};
  uint32_t row_step{0};
  bool is_bigendian{false};
  bool use_color{false};
  uint32_t timestamp_sec{0};
  uint32_t timestamp_nsec{0};
  std::string frame_id;

private:
  friend class nvidia::isaac_ros::nitros::NitrosBufferAccessor<NitrosPointCloud>;
  friend struct rclcpp::TypeAdapter<NitrosPointCloud, sensor_msgs::msg::PointCloud2>;

  // Implementation details (private, accessed via NitrosBufferAccessor)
  std::shared_ptr<nvidia::isaac_ros::nitros::NitrosBuffer> buffer_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPointCloud,
  sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosPointCloud;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosPointCloud,
  sensor_msgs::msg::PointCloud2);

#endif  // ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_HPP_
