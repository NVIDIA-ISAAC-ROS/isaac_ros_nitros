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
#include <vector>

#include "isaac_ros_common/cuda_stream.hpp"
#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_shape.hpp"
#include "rclcpp/rclcpp.hpp"

size_t nitros_compute_tensor_size(const nvidia::isaac_ros::nitros::NitrosTensor & tensor)
{
  return tensor.bytes_per_element() * tensor.element_count();
}

nvidia::isaac_ros::nitros::NitrosDataType
nvidia::isaac_ros::nitros::convert_to_nitros_data_type(int32_t data_type)
{
  switch (data_type) {
    case 1:
      return nvidia::isaac_ros::nitros::NitrosDataType::kInt8;
    case 2:
      return nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned8;
    case 3:
      return nvidia::isaac_ros::nitros::NitrosDataType::kInt16;
    case 4:
      return nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned16;
    case 5:
      return nvidia::isaac_ros::nitros::NitrosDataType::kInt32;
    case 6:
      return nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned32;
    case 7:
      return nvidia::isaac_ros::nitros::NitrosDataType::kInt64;
    case 8:
      return nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned64;
    case 9:
      return nvidia::isaac_ros::nitros::NitrosDataType::kFloat32;
    case 10:
      return nvidia::isaac_ros::nitros::NitrosDataType::kFloat64;
    default:
      return nvidia::isaac_ros::nitros::NitrosDataType::kUnknown;
  }
}

int32_t nvidia::isaac_ros::nitros::convert_to_ros_data_type(
  nvidia::isaac_ros::nitros::NitrosDataType nitros_data_type)
{
  switch (nitros_data_type) {
    case nvidia::isaac_ros::nitros::NitrosDataType::kInt8:
      return 1;
    case nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned8:
      return 2;
    case nvidia::isaac_ros::nitros::NitrosDataType::kInt16:
      return 3;
    case nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned16:
      return 4;
    case nvidia::isaac_ros::nitros::NitrosDataType::kInt32:
      return 5;
    case nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned32:
      return 6;
    case nvidia::isaac_ros::nitros::NitrosDataType::kInt64:
      return 7;
    case nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned64:
      return 8;
    case nvidia::isaac_ros::nitros::NitrosDataType::kFloat32:
      return 9;
    case nvidia::isaac_ros::nitros::NitrosDataType::kFloat64:
      return 10;
    default:
      return 0;
  }
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosTensorList,
  isaac_ros_tensor_list_interfaces::msg::TensorList>::convert_to_ros_message(
  const custom_type & source,
  ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosTensorList::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorList"),
    "[convert_to_ros_message] Conversion started for NitrosTensorList");

  auto stream_handle = nvidia::isaac_ros::nitros::CudaStreamPool::instance().get_stream_handle();
  cudaStream_t stream = stream_handle.get();
  if (stream == nullptr) {
    throw std::runtime_error("NitrosTensorList stream is nullptr");
  }

  for (auto nitros_tensor : source.get_tensors()) {
    auto ros_tensor = isaac_ros_tensor_list_interfaces::msg::Tensor();
    ros_tensor.name = nitros_tensor.get_name();
    ros_tensor.data_type = nvidia::isaac_ros::nitros::convert_to_ros_data_type(
      nitros_tensor.data_type());
    ros_tensor.shape.rank = nitros_tensor.shape().rank();
    for (size_t i = 0; i < nitros_tensor.shape().rank(); i++) {
      ros_tensor.shape.dims.push_back(nitros_tensor.shape().dims()[i]);
      ros_tensor.strides.push_back(nitros_tensor.strides()[i]);
    }

    switch (source.get_storage_type()) {
      case cudaMemoryTypeHost:
        {
          nvidia::isaac_ros::nitros::ReadHandle read_handle = nitros_tensor.get_read_handle(stream);
          size_t size = nitros_compute_tensor_size(nitros_tensor);
          ros_tensor.data.resize(size);
          std::memcpy(ros_tensor.data.data(), read_handle.get_ptr(), size);
        }
        break;
      case cudaMemoryTypeDevice:
        {
          nvidia::isaac_ros::nitros::ReadHandle read_handle =
            nitros_tensor.get_read_handle(stream);
          size_t size = nitros_compute_tensor_size(nitros_tensor);
          const uint8_t * dev_ptr = read_handle.get_ptr();
          if (dev_ptr == nullptr) {
            throw std::runtime_error("NitrosTensor device pointer is nullptr");
          }
          ros_tensor.data.resize(size);
          cudaError_t cuda_error = cudaMemcpyAsync(
            ros_tensor.data.data(), dev_ptr,
            size, cudaMemcpyDeviceToHost, stream);
          if (cuda_error != cudaSuccess) {
            std::stringstream error_msg;
            error_msg <<
              "[convert_to_ros_message] cudaMemcpy failed for conversion from "
              "NitrosTensor to ROS Tensor: " <<
              cudaGetErrorName(cuda_error) <<
              " (" << cudaGetErrorString(cuda_error) << ")";
            RCLCPP_ERROR(
              rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
            throw std::runtime_error(error_msg.str().c_str());
          }
          cuda_error = cudaStreamSynchronize(stream);
          if (cuda_error != cudaSuccess) {
            std::stringstream error_msg;
            error_msg <<
              "[convert_to_ros_message] cudaStreamSynchronize failed: " <<
              cudaGetErrorName(cuda_error) << " (" << cudaGetErrorString(cuda_error) << ")";
            RCLCPP_ERROR(rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
            throw std::runtime_error(error_msg.str().c_str());
          }
        }
        break;
      default:
        std::string error_msg =
          "[convert_to_ros_message] MemoryStorageType not supported: conversion from "
          "Tensor to ROS Tensor failed!";
        RCLCPP_ERROR(
          rclcpp::get_logger("NitrosTensorList"), error_msg.c_str());
        throw std::runtime_error(error_msg.c_str());
    }
    destination.tensors.push_back(ros_tensor);
  }

  destination.header.stamp.sec = static_cast<int32_t>(
    source.get_timestamp_sec());
  destination.header.stamp.nanosec = static_cast<uint32_t>(
    source.get_timestamp_nsec());
  destination.header.frame_id = source.get_frame_id();

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorList"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosTensorList,
  isaac_ros_tensor_list_interfaces::msg::TensorList>::convert_to_custom(
  const ros_message_type & source, custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosTensorList::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorList"),
    "[convert_to_custom] Conversion started");

  auto & stream_pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  cudaStream_t stream = stream_pool.acquire();

  destination.set_storage_type(cudaMemoryTypeDevice);

  auto stream_releaser = std::shared_ptr<void>(nullptr, [&stream_pool, stream](void *) {
        stream_pool.release(stream);
      });

  for (const auto & ros_tensor : source.tensors) {
    nvidia::isaac_ros::nitros::NitrosTensorShape shape{ros_tensor.shape.dims};
    nvidia::isaac_ros::nitros::NitrosDataType data_type =
      nvidia::isaac_ros::nitros::convert_to_nitros_data_type(
        static_cast<int32_t>(ros_tensor.data_type));
    if (data_type == nvidia::isaac_ros::nitros::NitrosDataType::kUnknown) {
      throw std::invalid_argument("[convert_to_custom] Unknown data type: " +
        std::to_string(ros_tensor.data_type));
    }

    nvidia::isaac_ros::nitros::NitrosTensor nitros_tensor(shape, data_type);
    nitros_tensor.set_name(ros_tensor.name);
    size_t data_size = nitros_compute_tensor_size(nitros_tensor);
    if (data_size == 0) {
      throw std::invalid_argument("[convert_to_custom] Data size is 0; cannot allocate buffer.");
    }

    uint8_t * dptr = nullptr;
    cudaError_t cuda_err = cudaMallocAsync(reinterpret_cast<void **>(&dptr), data_size, stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_custom] cudaMallocAsync failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    auto deleter = [stream, stream_releaser](uint8_t * p){
        if (p) {
          cudaFreeAsync(p, stream);
        }
      };

    auto write_handle = nitros_tensor.from_external(
      ros_tensor.name, dptr, data_size, shape, data_type, stream, deleter);

    cuda_err = cudaMemcpyAsync(write_handle.get_ptr(), ros_tensor.data.data(), data_size,
      cudaMemcpyHostToDevice, stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_custom] cudaMemcpyAsync H2D failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosTensorList"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    destination.tensors_.push_back(nitros_tensor);
  }
  destination.set_timestamp_sec(source.header.stamp.sec);
  destination.set_timestamp_nsec(source.header.stamp.nanosec);
  destination.set_frame_id(source.header.frame_id);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorList"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
