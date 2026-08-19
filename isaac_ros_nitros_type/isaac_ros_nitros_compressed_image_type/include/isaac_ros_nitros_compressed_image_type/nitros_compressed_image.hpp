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

#ifndef ISAAC_ROS_NITROS_COMPRESSED_IMAGE_TYPE__NITROS_COMPRESSED_IMAGE_HPP_
#define ISAAC_ROS_NITROS_COMPRESSED_IMAGE_TYPE__NITROS_COMPRESSED_IMAGE_HPP_

#include <cuda_runtime.h>

#include <cstring>
#include <memory>
#include <string>
#include <cstdint>
#include <utility>
#include <vector>
#include <functional>

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

struct NitrosCompressedImage;

struct nitros_compressed_image_t
{
  using MsgT = NitrosCompressedImage;
  static const inline std::string supported_type_name = "nitros_compressed_image";
};

class NitrosCompressedImage : public NitrosTypeBase
{
public:
  using SharedPtr = std::shared_ptr<NitrosCompressedImage>;
  using ConstSharedPtr = std::shared_ptr<const NitrosCompressedImage>;
  using UniquePtr = std::unique_ptr<NitrosCompressedImage>;
  using ConstUniquePtr = std::unique_ptr<const NitrosCompressedImage>;
  using WeakPtr = std::weak_ptr<NitrosCompressedImage>;
  using ConstWeakPtr = std::weak_ptr<const NitrosCompressedImage>;
  using ConstPtr = const NitrosCompressedImage *;

  NitrosCompressedImage()
  : NitrosTypeBase()
  {
    handle = -1;
  }
  explicit NitrosCompressedImage(const NitrosTypeBase & base)
  : NitrosTypeBase(base) {}

  // Timestamp accessors
  uint32_t get_timestamp_sec() const override {return timestamp_sec;}
  uint32_t get_timestamp_nsec() const override {return timestamp_nsec;}
  const std::string & get_frame_id() const override {return frame_id;}

  // Timestamp setters
  void set_timestamp_sec(uint32_t sec) override {timestamp_sec = sec;}
  void set_timestamp_nsec(uint32_t nsec) override {timestamp_nsec = nsec;}

  // Get read handle for consuming compressed data on the specified stream
  nvidia::isaac_ros::nitros::ReadHandle get_read_handle(cudaStream_t stream) const
  {
    return buffer_->get_read_handle(stream);
  }

  // Initialize from pool storage and return write handle for producer
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_pool(
    nvidia::isaac_ros::nitros::CUDAMemoryPool & pool,
    size_t size,
    const std::string & fmt,
    cudaStream_t stream)
  {
    uint8_t * ptr = nullptr;
    const cudaError_t acquire_err = pool.acquire(&ptr);
    if (acquire_err != cudaSuccess) {
      throw std::runtime_error("CUDAMemoryPool exhausted");
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(
      ptr, pool.block_size(), pool.deleter());
    size_ = size;
    format_ = fmt;
    handle = -1;
    return buffer_->get_write_handle(stream);
  }

  // Initialize from an external device pointer and return write handle
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_external(
    void * device_ptr,
    size_t bytes,
    size_t data_size,
    const std::string & fmt,
    cudaStream_t stream,
    std::function<void(uint8_t *)> deleter = nullptr)
  {
    if (!deleter) {
      deleter = [](uint8_t * p){if (p) {cudaFree(p);}};
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(
      device_ptr, bytes, std::move(deleter));
    size_ = data_size;
    format_ = fmt;
    handle = -1;
    return buffer_->get_write_handle(stream);
  }

  // Data accessors
  size_t size() const {return size_;}
  const std::string & get_format() const {return format_;}
  void set_format(const std::string & fmt) {format_ = fmt;}

  // Public message metadata
  uint32_t timestamp_sec{0};
  uint32_t timestamp_nsec{0};
  std::string frame_id;

private:
  friend struct rclcpp::TypeAdapter<NitrosCompressedImage, sensor_msgs::msg::CompressedImage>;

  std::shared_ptr<nvidia::isaac_ros::nitros::NitrosBuffer> buffer_;
  size_t size_{0};
  std::string format_{"h264"};
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosCompressedImage,
  sensor_msgs::msg::CompressedImage>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosCompressedImage;
  using ros_message_type = sensor_msgs::msg::CompressedImage;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosCompressedImage,
  sensor_msgs::msg::CompressedImage);

#endif  // ISAAC_ROS_NITROS_COMPRESSED_IMAGE_TYPE__NITROS_COMPRESSED_IMAGE_HPP_
