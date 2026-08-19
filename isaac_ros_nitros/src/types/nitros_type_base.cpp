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

#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "rclcpp/rclcpp.hpp"


constexpr uint64_t kDefaultCUDAMemoryPoolSize = 2048ULL * 1024 * 1024;
constexpr char kDisableCUDAMemPoolEnv[] = "DISABLE_NITROS_CUDA_MEM_POOL";
constexpr char kCUDAMemPoolSizeEnv[] = "CUDA_MEM_POOL_SIZE";  // CUDA native memory pool size in MB

namespace
{

bool IsCUDAMemPoolDisabledFromEnv(const char * env_name)
{
  const char * disable_cuda_mem_pool_env = std::getenv(env_name);
  bool disable_cuda_mem_pool = false;
  if (disable_cuda_mem_pool_env != nullptr) {
    auto disable_cuda_mem_pool_str = std::string(disable_cuda_mem_pool_env);
    if (disable_cuda_mem_pool_str == "1" || disable_cuda_mem_pool_str == "true") {
      disable_cuda_mem_pool = true;
    }
  }
  return disable_cuda_mem_pool;
}

uint64_t GetCUDAMemPoolSizeFromEnv(const char * env_name, uint64_t default_size)
{
  const char * cuda_mem_pool_size_env = std::getenv(env_name);
  if (cuda_mem_pool_size_env != nullptr) {
    try {
      return std::stoull(cuda_mem_pool_size_env) * 1024 * 1024;
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("NitrosTypeBase"),
        "Invalid value for %s: %s. Using default size.", env_name, cuda_mem_pool_size_env);
    }
  }
  return default_size;
}

cudaError_t setCUDAMemoryPoolSize(uint64_t cuda_mem_pool_size)
{
  // Set the minimal default CUDA memory pool release threshold to 2 GB
  int n_devices;
  cudaMemPool_t default_cuda_mem_pool;
  cudaError_t cuda_error{cudaSuccess};

  cuda_error = cudaGetDeviceCount(&n_devices);
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      cudaGetErrorName(cuda_error) << " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTypeBase"), error_msg.str().c_str());
    return cuda_error;
  }

  if (n_devices == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTypeBase"), "No device is available");
    return cudaErrorNoDevice;
  }

  cuda_error = cudaDeviceGetDefaultMemPool(&default_cuda_mem_pool, 0);
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      cudaGetErrorName(cuda_error) << " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTypeBase"), error_msg.str().c_str());
    return cuda_error;
  }

  uint64_t cur_release_threshold = 0;
  cuda_error = cudaMemPoolGetAttribute(
    default_cuda_mem_pool, cudaMemPoolAttrReleaseThreshold, &cur_release_threshold);
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      cudaGetErrorName(cuda_error) << " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTypeBase"), error_msg.str().c_str());
    return cuda_error;
  }

  if (cur_release_threshold < cuda_mem_pool_size) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosTypeBase"),
      "Setting CUDA memory pool size to %lu bytes", cuda_mem_pool_size);
    cuda_error = cudaMemPoolSetAttribute(
      default_cuda_mem_pool, cudaMemPoolAttrReleaseThreshold, &cuda_mem_pool_size);
    if (cuda_error != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        cudaGetErrorName(cuda_error) << " (" << cudaGetErrorString(cuda_error) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosTypeBase"), error_msg.str().c_str());
      return cuda_error;
    }
  }

  return cudaSuccess;
}

}  // namespace

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosTypeBase::NitrosTypeBase(
  const int64_t handle,
  const std::string data_format_name,
  const std::string compatible_data_format_name,
  const std::string frame_id,
  const cudaStream_t cuda_stream)
: handle(handle),
  data_format_name(data_format_name),
  compatible_data_format_name(compatible_data_format_name),
  frame_id(frame_id),
  cuda_stream(cuda_stream)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTypeBase"),
    "[Constructor] Creating a Nitros-typed object for handle = %ld",
    handle);

  if (!IsCUDAMemPoolDisabledFromEnv(kDisableCUDAMemPoolEnv)) {
    const uint64_t pool_size = GetCUDAMemPoolSizeFromEnv(
      kCUDAMemPoolSizeEnv, kDefaultCUDAMemoryPoolSize);
    cudaError_t code = setCUDAMemoryPoolSize(pool_size);
    if (code != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        cudaGetErrorName(code) << " (" << cudaGetErrorString(code) << ")";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosTypeBase"),
        "[NitrosTypeBase] setCUDAMemoryPoolSize Error: %s", error_msg.str().c_str());
    }
  }
}

NitrosTypeBase::NitrosTypeBase(const NitrosTypeBase & other)
: handle(other.handle),
  data_format_name(other.data_format_name),
  compatible_data_format_name(other.compatible_data_format_name),
  frame_id(other.frame_id),
  cuda_stream(other.cuda_stream)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTypeBase"),
    "[Copy Constructor] Copying a Nitros-typed object for handle = %ld",
    other.handle);
}

NitrosTypeBase & NitrosTypeBase::operator=(const NitrosTypeBase & other)
{
  if (this != &other) {
    handle = other.handle;
    data_format_name = other.data_format_name;
    compatible_data_format_name = other.compatible_data_format_name;
    frame_id = other.frame_id;
    cuda_stream = other.cuda_stream;
  }
  return *this;
}

NitrosTypeBase::NitrosTypeBase(NitrosTypeBase && other) noexcept
: handle(other.handle),
  data_format_name(std::move(other.data_format_name)),
  compatible_data_format_name(std::move(other.compatible_data_format_name)),
  frame_id(std::move(other.frame_id)),
  cuda_stream(other.cuda_stream)
{
  // Moved-from object must not decrement refcount when destroyed
  other.handle = -1;
}

NitrosTypeBase & NitrosTypeBase::operator=(NitrosTypeBase && other) noexcept
{
  if (this != &other) {
    handle = other.handle;
    data_format_name = std::move(other.data_format_name);
    compatible_data_format_name = std::move(other.compatible_data_format_name);
    frame_id = std::move(other.frame_id);
    cuda_stream = other.cuda_stream;
    other.handle = -1;
  }
  return *this;
}

NitrosTypeBase::~NitrosTypeBase()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTypeBase"),
    "[Destructor]Destroying a Nitros-typed object for handle = %ld",
    handle);
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
