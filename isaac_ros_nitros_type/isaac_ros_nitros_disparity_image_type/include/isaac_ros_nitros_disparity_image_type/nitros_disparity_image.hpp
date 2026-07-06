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

#ifndef ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_HPP_
#define ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosDisparityImage
 *   ROS type:    stereo_msgs::msg::DisparityImage
 */

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <map>

#include "isaac_ros_nitros/types/nitros_format_agent.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"
#include "rclcpp/type_adapter.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type forward declaration
struct NitrosDisparityImage;

// Format descriptor for NitrosTypeManager / negotiated API (GXF-less path)
struct nitros_disparity_image_32FC1_t
{
  using MsgT = NitrosDisparityImage;
  static const inline std::string supported_type_name = "nitros_disparity_image_32FC1";
};

class NitrosDisparityImage : public NitrosTypeBase
{
public:
  // Need to be removed after the GXF compat mode is removed
  static std::string GetDefaultCompatibleFormat()
  {
    constexpr const char * kDefaultCompatibleFormat = "nitros_disparity_image_32FC1";
    return kDefaultCompatibleFormat;
  }
  static std::map<std::string, NitrosFormatCallbacks> GetFormatCallbacks()
  {
    std::map<std::string, NitrosFormatCallbacks> format_callback_map;
    format_callback_map.emplace(
      nitros_disparity_image_32FC1_t::supported_type_name,
      NitrosFormatAgent<nitros_disparity_image_32FC1_t>::GetFormatCallbacks());
    return format_callback_map;
  }
  static std::vector<std::pair<std::string, std::string>> GetExtensions() {return {};}

  // Standard ROS2 message pointer type aliases (required by message_filters)
  using SharedPtr = std::shared_ptr<NitrosDisparityImage>;
  using ConstSharedPtr = std::shared_ptr<const NitrosDisparityImage>;
  using UniquePtr = std::unique_ptr<NitrosDisparityImage>;
  using ConstUniquePtr = std::unique_ptr<const NitrosDisparityImage>;
  using WeakPtr = std::weak_ptr<NitrosDisparityImage>;
  using ConstWeakPtr = std::weak_ptr<const NitrosDisparityImage>;
  using ConstPtr = const NitrosDisparityImage *;

  NitrosDisparityImage()
  : NitrosTypeBase()
  {
    handle = -1;
  }
  explicit NitrosDisparityImage(const NitrosTypeBase & base)
  : NitrosTypeBase(base)
  {
    handle = -1;
  }

  // Message metadata accessors
  const std::string & get_encoding() const {return encoding;}
  uint32_t get_width() const {return width;}
  uint32_t get_height() const {return height;}
  uint32_t get_step() const {return step;}
  // For nitros_disparity_image_32FC1, one plane of float32 array
  uint32_t get_num_planes() const {return 1;}

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
    uint32_t width_in,
    uint32_t height_in,
    uint32_t step_bytes,
    const std::string & encoding_in,
    cudaStream_t stream)
  {
    uint8_t * ptr = nullptr;
    const cudaError_t acquire_err = pool.acquire(&ptr);
    if (acquire_err != cudaSuccess) {
      throw std::runtime_error("CUDAMemoryPool exhausted");
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(ptr, pool.block_size(),
            pool.deleter());
    this->width = width_in;
    this->height = height_in;
    this->step = step_bytes;
    this->encoding = encoding_in;
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
    uint32_t step_bytes,
    const std::string & encoding,
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
    this->step = step_bytes;
    this->encoding = encoding;
    handle = -1;
    return buffer_->get_write_handle(stream);
  }

  // Core message data (public like ROS2 messages)
  uint32_t width{0};
  uint32_t height{0};
  uint32_t step{0};
  std::string encoding;
  uint32_t timestamp_sec{0};
  uint32_t timestamp_nsec{0};
  std::string frame_id;

  // Stereo geometry parameters
  float f{0.0f};
  float t{0.0f};
  float min_disparity{0.0f};
  float max_disparity{0.0f};
  std::array<uint32_t, 4> roi{0, 0, 0, 0};
  float delta_disparity{0.0f};

private:
  friend class nvidia::isaac_ros::nitros::NitrosBufferAccessor<NitrosDisparityImage>;
  friend struct rclcpp::TypeAdapter<NitrosDisparityImage, stereo_msgs::msg::DisparityImage>;

  // Implementation details (private, accessed via NitrosBufferAccessor)
  std::shared_ptr<nvidia::isaac_ros::nitros::NitrosBuffer> buffer_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<nvidia::isaac_ros::nitros::NitrosDisparityImage,
  stereo_msgs::msg::DisparityImage>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosDisparityImage;
  using ros_message_type = stereo_msgs::msg::DisparityImage;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosDisparityImage,
  stereo_msgs::msg::DisparityImage);

#endif  // ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_HPP_
