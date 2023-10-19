// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosTensorBuilder::NitrosTensorBuilder()
: nitros_tensor_{}
{
  auto message = nvidia::gxf::Entity::New(GetTypeAdapterNitrosContext().getContext());
  if (!message) {
    std::stringstream error_msg;
    error_msg <<
      "[constructor] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTensorBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  nitros_tensor_.handle = message->eid();
  GxfEntityRefCountInc(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(), message->eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorBuilder"),
    "[constructor] NitrosTensor initialized");
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

NitrosTensor NitrosTensorBuilder::Build()
{
  auto message = nvidia::gxf::Entity::Shared(
    GetTypeAdapterNitrosContext().getContext(), nitros_tensor_.handle);

  auto gxf_tensor = message->add<nvidia::gxf::Tensor>();

  auto gxf_data_type = GetPrimitiveType(data_type_);

  // If CUDA event provided, synchronize on that event before building
  if (event_) {
    cudaError_t cuda_error = cudaEventSynchronize(event_);

    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventSynchronize failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosTensorBuilder"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    cuda_error = cudaEventDestroy(event_);
    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[Build] cudaEventDestroy failed: " <<
        cudaGetErrorName(cuda_error) <<
        " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosTensorBuilder"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  gxf_tensor.value()->wrapMemory(
    shape_.shape(),
    gxf_data_type,
    nvidia::gxf::PrimitiveTypeSize(gxf_data_type),
    nvidia::gxf::Unexpected{GXF_UNINITIALIZED_VALUE},
    nvidia::gxf::MemoryStorageType::kDevice, data_,
    [](void * ptr) {cudaFree(ptr); return nvidia::gxf::Success;});

  RCLCPP_DEBUG(
    rclcpp::get_logger(
      "NitrosTensorBuilder"), "[Build] Tensor built");

  return nitros_tensor_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
