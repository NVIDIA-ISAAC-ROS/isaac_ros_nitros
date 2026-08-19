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

#ifndef ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_HPP_
#define ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_HPP_

#include <cuda_runtime.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <vector>
#include <cstddef>
#include <utility>

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"
#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Forward declaration
struct NitrosImage;
struct nitros_image_rgb8_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_rgb8";
};

class NitrosImage : public NitrosTypeBase
{
public:
  static std::string GetDefaultCompatibleFormat() {return nitros_image_rgb8_t::supported_type_name;}

  // Standard ROS2 message pointer type aliases (required by message_filters)
  using SharedPtr = std::shared_ptr<NitrosImage>;
  using ConstSharedPtr = std::shared_ptr<const NitrosImage>;
  using UniquePtr = std::unique_ptr<NitrosImage>;
  using ConstUniquePtr = std::unique_ptr<const NitrosImage>;
  using WeakPtr = std::weak_ptr<NitrosImage>;
  using ConstWeakPtr = std::weak_ptr<const NitrosImage>;
  using ConstPtr = const NitrosImage *;

  struct ColorPlane
  {
    uint32_t width{0};
    uint32_t height{0};
    uint32_t stride{0};
    size_t size{0};
    size_t offset{0};
  };

  NitrosImage()
  : NitrosTypeBase()
  {
  }
  explicit NitrosImage(const NitrosTypeBase & base)
  : NitrosTypeBase(base) {}

  // Message metadata accessors
  // Color planes describe per-plane layout for multi-plane formats (e.g., NV12, NV24)
  const std::vector<ColorPlane> & get_color_planes() const {return color_planes_;}
  size_t num_planes() const {return color_planes_.size();}
  const ColorPlane & get_plane(size_t i) const {return color_planes_.at(i);}
  uint64_t get_plane_offset(size_t i) const {return color_planes_.at(i).offset;}
  uint8_t * get_plane_ptr(size_t i) const
  {
    if (!buffer_) {
      throw std::runtime_error("NitrosImage: buffer is null (get_plane_ptr)");
    }
    return const_cast<uint8_t *>(buffer_->get_data() + get_plane_offset(i));
  }
  size_t get_data_size() const
  {
    size_t total = 0;
    for (size_t i = 0; i < num_planes(); i++) {
      total += get_plane(i).size;
    }
    return total;
  }

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
    if (!buffer_) {
      throw std::runtime_error(
        "NitrosImage: buffer is null (get_read_handle). "
        "Ensure the message has GPU data (e.g. not received from another process).");
    }
    return buffer_->get_read_handle(stream);
  }

  // Initialize from pool storage and return write handle for producer
  // Memory is acquired from the pool and will be recycled when buffer is destroyed
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_pool(
    nvidia::isaac_ros::nitros::CUDAMemoryPool & pool,
    uint32_t width,
    uint32_t height,
    uint32_t step_bytes,
    const std::string & encoding,
    cudaStream_t stream)
  {
    if (step_bytes == 0) {
      throw std::invalid_argument("step_bytes must be non-zero");
    }
    const size_t required_bytes = get_total_image_bytes(encoding, height, step_bytes);
    if (required_bytes > pool.block_size()) {
      throw std::runtime_error(
        "Pool block too small: need " + std::to_string(required_bytes) +
        " bytes, pool block is " + std::to_string(pool.block_size()) + " bytes");
    }
    uint8_t * ptr = nullptr;
    const cudaError_t acquire_err = pool.acquire(&ptr);
    if (acquire_err != cudaSuccess) {
      throw std::runtime_error("CUDAMemoryPool exhausted");
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(ptr, pool.block_size(),
            pool.deleter());
    this->width = width;
    this->height = height;
    this->step = step_bytes;
    this->encoding = encoding;
    setup_planes(width, height, step_bytes, encoding);
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
    setup_planes(width, height, step_bytes, encoding);
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

private:
  friend class nvidia::isaac_ros::nitros::NitrosBufferAccessor<NitrosImage>;
  friend struct rclcpp::TypeAdapter<NitrosImage, sensor_msgs::msg::Image>;

  // Implementation details (private, accessed via NitrosBufferAccessor)
  std::shared_ptr<nvidia::isaac_ros::nitros::NitrosBuffer> buffer_;
  std::vector<ColorPlane> color_planes_{};

  std::vector<uint32_t> get_bytes_per_pixel(std::string encoding) const
  {
    if (encoding == "rgb8" || encoding == "bgr8") {
      return {3};
    } else if (encoding == "rgba8" || encoding == "bgra8") {
      return {4};
    } else if (encoding == "rgb16" || encoding == "bgr16") {
      return {6};
    } else if (encoding == "mono8") {
      return {1};
    } else if (encoding == "mono16") {
      return {2};
    } else if (encoding == "16UC1") {
      return {2};
    } else if (encoding == "32FC3") {
      return {12};
    } else if (encoding == "32FC1") {
      return {4};
    } else if (encoding == "nv12") {
      return {1, 2};
    } else if (encoding == "nv24") {
      return {1, 2};
    } else if (encoding == "32FC4") {
      return {16};
    } else {
      throw std::runtime_error("get_bytes_per_pixel: Unsupported encoding: " + encoding);
    }
  }

  static size_t get_total_image_bytes(
    const std::string & encoding, uint32_t height, uint32_t step_bytes)
  {
    // ROS image messages represent NV12/NV24 as compact buffers with no gap
    // between planes. Producers must remove any hardware-specific inter-plane
    // padding before calling from_pool() or from_external().
    const size_t y_bytes = static_cast<size_t>(step_bytes) * height;
    if (encoding == "nv12") {
      return y_bytes * 3 / 2;
    }
    if (encoding == "nv24") {
      return y_bytes * 3;
    }
    return y_bytes;
  }

  // Lay out per-plane geometry conforming to ROS/V4L2 compatibility.
  // The single ROS2 sensor_msgs/Image `step` field defines the full row length in bytes
  // for the y row stride:
  //   https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg
  // For the multi-planar encodings in ROS2, chroma stride is derived from luma stride per V4L2's
  // contiguous-plane expectation that ROS2's image_encodings.hpp explicitly references:
  //   https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-yuv-planar.html
  // Therefore, we assume NV12 uv stride == y stride and NV24 uv stride == 2 * y stride.
  // Padding for y and uv strides also follows these expectations. If the input image does not
  // follow these expectations, you will need to adjust the image to maintain ROS/V4L2
  // compatibility.
  void setup_planes(uint32_t w, uint32_t h, uint32_t step_bytes, const std::string & enc)
  {
    std::vector<uint32_t> bytes_per_pixel = get_bytes_per_pixel(enc);
    const uint32_t min_y_row_bytes = bytes_per_pixel[0] * w;
    if (step_bytes != 0 && step_bytes < min_y_row_bytes) {
      throw std::invalid_argument(
              "NitrosImage: step_bytes (" + std::to_string(step_bytes) +
              ") is smaller than the minimum luma row size for encoding '" + enc +
              "' (width=" + std::to_string(w) + " * bytes_per_pixel=" +
              std::to_string(bytes_per_pixel[0]) + " = " +
              std::to_string(min_y_row_bytes) +
              "). Pass step >= width * bytes_per_pixel; ROS2 sensor_msgs/Image "
              "defines step as the full row length in bytes.");
    }

    color_planes_.clear();
    ColorPlane y{};
    y.width = w;
    y.height = h;
    y.stride = step_bytes > 0 ? step_bytes : min_y_row_bytes;
    y.size = static_cast<size_t>(y.stride) * y.height;
    y.offset = 0;
    color_planes_.push_back(y);
    if (enc == "nv12") {
      // NV12 `odd width height would silently truncate a chroma row/column. Reject early
      // rather than producing an under-sized chroma plane.
      if ((w % 2) != 0 || (h % 2) != 0) {
        throw std::invalid_argument(
                "NitrosImage: NV12 requires even width and height (4:2:0 chroma "
                "subsampling), got " + std::to_string(w) + "x" + std::to_string(h) +
                ". Pad the image to the next even dimension before constructing "
                "the message.");
      }
      ColorPlane uv{};
      uv.width = w / 2;
      uv.height = h / 2;
      uv.stride = y.stride;  // UV pitch equals Y pitch for NV12
      uv.size = static_cast<size_t>(uv.stride) * uv.height;
      uv.offset = y.size;
      color_planes_.push_back(uv);
    }
    if (enc == "nv24") {
      ColorPlane uv{};
      uv.width = w;
      uv.height = h;
      uv.stride = y.stride * 2;  // UV pitch equals 2 X Y pitch for NV24
      uv.size = static_cast<size_t>(uv.stride) * uv.height;
      uv.offset = y.size;
      color_planes_.push_back(uv);
    }
  }
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosImage,
  sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosImage;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosImage,
  sensor_msgs::msg::Image);

#endif  // ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_HPP_
