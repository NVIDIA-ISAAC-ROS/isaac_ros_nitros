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

#ifndef ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_HPP_
#define ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_HPP_

#ifndef NITROS_GXF_COMPAT_MODE
#define NITROS_GXF_COMPAT_MODE
#endif

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_data_type.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_shape.hpp"
#include "rclcpp/type_adapter.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{
class NitrosTensor : public NitrosTypeBase {
public:
  // Standard ROS2 message pointer type aliases (required by message_filters)
  using SharedPtr = std::shared_ptr<NitrosTensor>;
  using ConstSharedPtr = std::shared_ptr<const NitrosTensor>;
  using UniquePtr = std::unique_ptr<NitrosTensor>;
  using ConstUniquePtr = std::unique_ptr<const NitrosTensor>;
  using WeakPtr = std::weak_ptr<NitrosTensor>;
  using ConstWeakPtr = std::weak_ptr<const NitrosTensor>;
  using ConstPtr = const NitrosTensor *;

  NitrosTensor() = default;

  explicit NitrosTensor(const NitrosTypeBase & base)
  : NitrosTypeBase(base) {}

  explicit NitrosTensor(
    const NitrosTensorShape & shape,
    const NitrosDataType & data_type)
  : shape_(shape), data_type_(data_type)
  {
#ifndef NITROS_GXF_COMPAT_MODE
    handle = -1;
#endif
  }

  explicit NitrosTensor(
    const std::string & name, const NitrosTensorShape & shape,
    const NitrosDataType & data_type)
  : name_(name), shape_(shape), data_type_(data_type)
  {
#ifndef NITROS_GXF_COMPAT_MODE
    handle = -1;
#endif
  }

  // Message metadata accessors
  nvidia::isaac_ros::nitros::NitrosTensorShape shape() const {return shape_;}
  nvidia::isaac_ros::nitros::NitrosDataType data_type() const {return data_type_;}
  const std::string & get_name() const {return name_;}
  void set_name(const std::string & name) {name_ = name;}
  std::vector<uint64_t> strides() const {return strides_;}
  size_t bytes_per_element() const
  {
    switch (data_type_) {
      case nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned8:
      case nvidia::isaac_ros::nitros::NitrosDataType::kInt8:
        return 1;
      case nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned16:
      case nvidia::isaac_ros::nitros::NitrosDataType::kInt16:
        return 2;
      case nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned32:
      case nvidia::isaac_ros::nitros::NitrosDataType::kInt32:
      case nvidia::isaac_ros::nitros::NitrosDataType::kFloat32:
        return 4;
      case nvidia::isaac_ros::nitros::NitrosDataType::kInt64:
      case nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned64:
      case nvidia::isaac_ros::nitros::NitrosDataType::kFloat64:
        return 8;
      default:
        throw std::invalid_argument("Unsupported NitrosDataType in bytes_per_element");
    }
  }
  size_t element_count() const
  {
    size_t num_elements = 1;
    for (uint32_t i = 0; i < shape_.rank(); i++) {
      num_elements *= shape_.dims()[i];
    }
    return num_elements;
  }
  size_t tensor_size() const
  {
    return element_count() * bytes_per_element();
  }

  // Backward compatibility functions, once Nodes are migrated, these can be removed
  size_t GetTensorSize() const
  {
    return tensor_size();
  }
  const uint8_t * GetBuffer(cudaStream_t stream) const
  {
    return buffer_->get_read_handle(stream).get_ptr();
  }
  uint32_t GetRank() const
  {
    return shape_.rank();
  }
  NitrosTensorShape GetShape() const
  {
    return shape_;
  }

  // Get read handle for consuming image data on the specified stream
  nvidia::isaac_ros::nitros::ReadHandle get_read_handle(cudaStream_t stream) const
  {
    return buffer_->get_read_handle(stream);
  }

  // Initialize from pool storage and return write handle for producer
  // Memory is acquired from the pool and will be recycled when buffer is destroyed
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_pool(
    const std::string & name,
    nvidia::isaac_ros::nitros::CUDAMemoryPool & pool,
    const nvidia::isaac_ros::nitros::NitrosTensorShape & shape,
    const nvidia::isaac_ros::nitros::NitrosDataType & data_type,
    // const NitrosTensorLayout & layout,
    cudaStream_t stream)
  {
    uint8_t * ptr = nullptr;
    const cudaError_t acquire_err = pool.acquire(&ptr);
    if (acquire_err != cudaSuccess) {
      throw std::invalid_argument("CUDAMemoryPool exhausted");
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(ptr, pool.block_size(),
            pool.deleter());
    this->name_ = name;
    this->shape_ = shape;
    this->data_type_ = data_type;
    this->strides_.resize(shape_.rank());
    this->strides_[shape_.rank() - 1] = bytes_per_element();
    for (int32_t i = shape_.rank() - 2; i >= 0; i--) {
      this->strides_[i] = this->strides_[i + 1] * shape_.dims()[i + 1];
    }
#ifndef NITROS_GXF_COMPAT_MODE
    handle = -1;
#endif
    return buffer_->get_write_handle(stream);
  }

  // Initialize from an external device pointer and return write handle for producer
  // Memory ownership is transferred to the buffer with the provided deleter
  // If no deleter provided, defaults to cudaFree
  [[nodiscard]] nvidia::isaac_ros::nitros::WriteHandle from_external(
    const std::string & name,
    void * device_ptr,
    size_t bytes,
    const nvidia::isaac_ros::nitros::NitrosTensorShape & shape,
    const nvidia::isaac_ros::nitros::NitrosDataType & data_type,
    cudaStream_t stream,
    std::function<void(uint8_t *)> deleter = nullptr)
  {
    if (!deleter) {
      deleter = [](uint8_t * p){if (p) {cudaFree(p);}};
    }
    buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(device_ptr, bytes,
            std::move(deleter));
    this->name_ = name;
    this->shape_ = shape;
    this->data_type_ = data_type;
    this->strides_.resize(shape_.rank());
    this->strides_[shape_.rank() - 1] = bytes_per_element();
    for (int32_t i = shape_.rank() - 2; i >= 0; i--) {
      this->strides_[i] = this->strides_[i + 1] * shape_.dims()[i + 1];
    }
#ifndef NITROS_GXF_COMPAT_MODE
    handle = -1;
#endif
    return buffer_->get_write_handle(stream);
  }

private:
  friend class nvidia::isaac_ros::nitros::NitrosBufferAccessor<NitrosTensor>;

  // NitrosBuffer instance for tensor data
  std::shared_ptr<nvidia::isaac_ros::nitros::NitrosBuffer> buffer_;

  // Core message data (public like ROS2 messages)
  std::string name_;
  nvidia::isaac_ros::nitros::NitrosTensorShape shape_;
  nvidia::isaac_ros::nitros::NitrosDataType data_type_{nvidia::isaac_ros::nitros::NitrosDataType::
    kUnsigned8};
  std::vector<uint64_t> strides_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_HPP_
