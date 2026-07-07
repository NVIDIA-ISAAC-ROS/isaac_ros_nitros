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
#include <stdexcept>
#include <string>

#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

namespace
{
size_t BytesPerElement(NitrosDataType data_type)
{
  switch (data_type) {
    case NitrosDataType::kUnsigned8:
    case NitrosDataType::kInt8:
      return 1;
    case NitrosDataType::kUnsigned16:
    case NitrosDataType::kInt16:
      return 2;
    case NitrosDataType::kUnsigned32:
    case NitrosDataType::kInt32:
    case NitrosDataType::kFloat32:
      return 4;
    case NitrosDataType::kInt64:
    case NitrosDataType::kUnsigned64:
    case NitrosDataType::kFloat64:
      return 8;
    default:
      throw std::invalid_argument("Unsupported NitrosDataType in NitrosTensorBuilder::Build");
  }
}
}  // namespace

NitrosTensorBuilder::NitrosTensorBuilder()
: nitros_tensor_{}
{
  RCLCPP_DEBUG(rclcpp::get_logger("NitrosTensorBuilder"), "[constructor] NitrosTensor initialized");
}

NitrosTensorBuilder & NitrosTensorBuilder::WithName(std::string name)
{
  name_ = name;
  return *this;
}

NitrosTensorBuilder & NitrosTensorBuilder::WithShape(NitrosTensorShape tensor_shape)
{
  shape_ = tensor_shape;
  return *this;
}

NitrosTensorBuilder & NitrosTensorBuilder::WithDataType(NitrosDataType data_type)
{
  data_type_ = data_type;
  return *this;
}

NitrosTensorBuilder & NitrosTensorBuilder::WithData(void * data)
{
  data_ = data;
  return *this;
}

NitrosTensorBuilder & NitrosTensorBuilder::WithEvent(cudaEvent_t event)
{
  event_ = event;
  return *this;
}

NitrosTensorBuilder & NitrosTensorBuilder::WithReleaseCallback(
  std::function<void()> release_callback)
{
  release_callback_ = release_callback;
  return *this;
}

bool ReleaseTensorCallback(
  std::function<void()> release_callback,
  void * ptr)
{
  if (release_callback) {
    release_callback();
  } else {
    cudaFree(ptr);
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorBuilder"),
    "[ReleaseTensorCallback] Released the cuda memory [%p]", ptr);
  return true;
}

void NitrosTensorBuilder::Validate()
{
  bool failure = false;
  std::stringstream error_msg;
  if (data_ == nullptr) {
    error_msg << "Data buffer is not set! Call WithData method before Build. \n";
    failure = true;
  }

  if (data_type_ == NitrosDataType::kUnknown) {
    error_msg << "Data type is not set! Call WithDataType method before Build. \n";
    failure = true;
  }
  if (shape_.rank() == 0) {
    error_msg << "Shape is not set! Call WithShape method before Build. \n";
    failure = true;
  }

  if (failure) {
    RCLCPP_ERROR(rclcpp::get_logger("NitrosTensorBuilder"), "%s", error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
}

NitrosTensor NitrosTensorBuilder::Build()
{
  Validate();

  // If CUDA event provided, synchronize on that event before building
  if (event_) {
    cudaError_t cuda_error = cudaEventSynchronize(event_);
    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventSynchronize failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosTensorBuilder"), "%s", error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    cuda_error = cudaEventDestroy(event_);
    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventDestroy failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosTensorBuilder"), "%s", error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // Compute total buffer size from builder's shape and data type
  size_t element_count = 1;
  for (uint32_t i = 0; i < shape_.rank(); i++) {
    element_count *= shape_.dims()[i];
  }
  const size_t bytes_per_element = BytesPerElement(data_type_);
  const size_t buffer_size = element_count * bytes_per_element;

  // Create stream for async operations
  auto & stream_pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  cudaStream_t cuda_stream = stream_pool.acquire();

  // Wrap the external GPU buffer using from_external
  auto deleter = [release_callback = release_callback_, &stream_pool, cuda_stream](uint8_t * p) {
      if (p) {
        if (release_callback) {
          release_callback();
        } else {
          cudaFreeAsync(p, cuda_stream);
        }
      }
      stream_pool.release(cuda_stream);
    };

  [[maybe_unused]] auto write_handle = nitros_tensor_.from_external(
    name_, data_, buffer_size, shape_, data_type_, cuda_stream, deleter);

  RCLCPP_DEBUG(rclcpp::get_logger("NitrosTensorBuilder"), "[Build] Tensor built");

  return nitros_tensor_;
}
}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
