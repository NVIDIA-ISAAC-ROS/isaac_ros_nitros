// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_BRIDGE_ROS1__IPC_BUFFER_MANAGER_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_ROS1__IPC_BUFFER_MANAGER_HPP_

#include <cuda_runtime_api.h>
#include <cuda.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "ros/ros.h"


namespace nvidia
{
namespace isaac_ros
{

namespace nitros_bridge
{

// Size of shared memory object in bytes
constexpr size_t kSharedMemorySize = 1024;

struct DeviceIPCBuffer
{
  CUdeviceptr d_ptr;
  CUmemGenericAllocationHandle generic_allocation_handle;
  int pid;
  int fd;
  std::string uid;
  size_t index;
};

struct HostIPCBuffer
{
  enum class Mode {CREATE, OPEN};
  std::shared_ptr<boost::interprocess::shared_memory_object> shm_object_ = nullptr;
  std::shared_ptr<boost::interprocess::named_mutex> mutex_ = nullptr;
  std::shared_ptr<boost::interprocess::mapped_region> shm_ptr_ = nullptr;
  boost::uuids::uuid * uid_ = nullptr;
  int32_t * refcount_ = nullptr;
  int start_time_ = 0;

  HostIPCBuffer(const std::string & shm_name, const Mode mode)
  {
    if (mode == Mode::CREATE) {
      shm_object_ = std::make_shared<boost::interprocess::shared_memory_object>(
        boost::interprocess::open_or_create,
        shm_name.c_str(), boost::interprocess::read_write);
      shm_object_->truncate(kSharedMemorySize);
      mutex_ = std::make_shared<boost::interprocess::named_mutex>(
        boost::interprocess::open_or_create, (shm_name).c_str());
      shm_ptr_ = std::make_shared<boost::interprocess::mapped_region>(
        *shm_object_, boost::interprocess::read_write);
      refcount_ = static_cast<int *>(shm_ptr_->get_address());
      uid_ = static_cast<boost::uuids::uuid *>(shm_ptr_->get_address() + sizeof(int));

      // Initialize refcount and UID
      *refcount_ = 0;
      *uid_ = boost::uuids::random_generator()();
    } else {
      try {
        shm_object_ = std::make_shared<boost::interprocess::shared_memory_object>(
          boost::interprocess::open_only, shm_name.c_str(), boost::interprocess::read_write);
        mutex_ = std::make_shared<boost::interprocess::named_mutex>(
          boost::interprocess::open_only, (shm_name).c_str());
        shm_ptr_ = std::make_shared<boost::interprocess::mapped_region>(
          *shm_object_, boost::interprocess::read_write);
        refcount_ = static_cast<int *>(shm_ptr_->get_address());
        uid_ = static_cast<boost::uuids::uuid *>(shm_ptr_->get_address() + sizeof(int));
      } catch (const boost::interprocess::interprocess_exception & ex) {
        ROS_ERROR("Failed to open shared memory object %s", ex.what());
        throw std::runtime_error("Failed to open shared memory object");
      }
    }
  }

  bool reset_if_refcount_zero()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
    if (*refcount_ == 0) {
      // Reset the start time and UID
      *uid_ = boost::uuids::random_generator()();
      start_time_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      return true;
    }
    return false;
  }

  bool refcoun_inc_if_uid_match(const std::string & received_uid)
  {
    // The UID compare and refcount increment should be atomic
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
    if (boost::uuids::to_string(*uid_) == received_uid) {
      *refcount_ += 1;
      return true;
    }
    return false;
  }

  void refcount_inc()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
    *refcount_ += 1;
  }

  void refcount_dec()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
    if (refcount_ == 0) {
      ROS_ERROR("Refcount is already zero.");
      return;
    }
    *refcount_ -= 1;
  }

  std::string get_uid()
  {
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
    auto uid = boost::uuids::to_string(*uid_);
    return uid;
  }
};

class IPCBufferManager
{
public:
  IPCBufferManager() = default;

  // Constructor, create device memory buffers and export to FD
  IPCBufferManager(size_t buffer_count, size_t buffer_step, int64_t timeout)
  {
    buffer_count_ = buffer_count;
    buffer_step_ = buffer_step;
    timeout_ = timeout;

    CUmemAllocationProp prop = {};
    prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
    prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    prop.location.id = 0;
    prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
    size_t granularity = 0;
    auto cuda_err = cuMemGetAllocationGranularity(
      &granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR("Failed to call cuMemGetAllocationGranularity %s", error_str);
    }
    alloc_size_ = buffer_step - (buffer_step % granularity) + granularity;

    for (size_t index = 0; index < buffer_count_; index++) {
      // Create shareable device memory buffer
      CUmemGenericAllocationHandle generic_allocation_handle;
      auto cuda_err = cuMemCreate(&generic_allocation_handle, alloc_size_, &prop, 0);
      if (CUDA_SUCCESS != cuda_err) {
        const char * error_str = NULL;
        cuGetErrorString(cuda_err, &error_str);
        ROS_ERROR("Failed to call cuMemCreate %s", error_str);
      }

      int fd = -1;
      cuda_err = cuMemExportToShareableHandle(
        reinterpret_cast<void *>(&fd),
        generic_allocation_handle,
        CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0);
      if (CUDA_SUCCESS != cuda_err) {
        const char * error_str = NULL;
        cuGetErrorString(cuda_err, &error_str);
        ROS_ERROR( "Failed to cuMemExportToShareableHandle %s", error_str);
      }

      CUdeviceptr d_ptr = 0ULL;
      cuda_err = cuMemAddressReserve(&d_ptr, alloc_size_, 0, 0, 0);
      if (CUDA_SUCCESS != cuda_err) {
        const char * error_str = NULL;
        cuGetErrorString(cuda_err, &error_str);
        ROS_ERROR("Failed to call cuMemCreate %s", error_str);
      }

      cuda_err = cuMemMap(d_ptr, alloc_size_, 0, generic_allocation_handle, 0);
      if (CUDA_SUCCESS != cuda_err) {
        const char * error_str = NULL;
        cuGetErrorString(cuda_err, &error_str);
        ROS_ERROR("Failed to call cuMemCreate %s", error_str);
      }

      CUmemAccessDesc accessDesc = {};
      accessDesc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
      accessDesc.location.id = 0;
      accessDesc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
      cuda_err = cuMemSetAccess(d_ptr, alloc_size_, &accessDesc, 1);
      if (CUDA_SUCCESS != cuda_err) {
        const char * error_str = NULL;
        cuGetErrorString(cuda_err, &error_str);
        ROS_ERROR("Failed to call cuMemSetAccess %s", error_str);
      }

      // Create host shared memory object
      std::string shm_name = std::to_string(getpid()) + std::to_string(fd);
      std::shared_ptr<HostIPCBuffer> host_ipc_buffer =
        std::make_shared<HostIPCBuffer>(shm_name, HostIPCBuffer::Mode::CREATE);
      std::shared_ptr<DeviceIPCBuffer> device_ipc_buffer =
        std::make_shared<DeviceIPCBuffer>(
        DeviceIPCBuffer{
              d_ptr, generic_allocation_handle, getpid(),
              fd, host_ipc_buffer->get_uid(), index});
      host_ipc_buffers_.push_back(host_ipc_buffer);
      device_ipc_buffers_.push_back(device_ipc_buffer);
    }
  }

  // Destructor, free the alloacted device memory pool
  ~IPCBufferManager()
  {
    for (size_t i = 0; i < buffer_count_; i++) {
      cuMemRelease(device_ipc_buffers_[i]->generic_allocation_handle);
      cuMemUnmap(device_ipc_buffers_[i]->d_ptr, alloc_size_);
      std::string shm_name =
        std::to_string(device_ipc_buffers_[i]->pid) + std::to_string(device_ipc_buffers_[i]->fd);
      boost::interprocess::named_mutex::remove(shm_name.c_str());
      boost::interprocess::shared_memory_object::remove(shm_name.c_str());
    }
  }

  // Move the index to next available device memory block
  std::shared_ptr<DeviceIPCBuffer> find_next_available_buffer()
  {
    auto last_handle_index = current_handle_index_;
    while (true) {
      auto current_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      // TODO(yuankunz) ：Reset the timeout everytime the refount is 0
      auto start_time = host_ipc_buffers_[current_handle_index_]->start_time_;
      if (current_time - start_time > timeout_) {
        if (host_ipc_buffers_[current_handle_index_]->reset_if_refcount_zero()) {
          device_ipc_buffers_[current_handle_index_]->uid =
            host_ipc_buffers_[current_handle_index_]->get_uid();
          auto ret_device_buffer = device_ipc_buffers_[current_handle_index_];
          current_handle_index_ = (current_handle_index_ + 1) % buffer_count_;
          return ret_device_buffer;
        }
      } else {
        current_handle_index_ = (current_handle_index_ + 1) % buffer_count_;
      }
      if (current_handle_index_ == last_handle_index) {
        ROS_INFO("No available buffer, re-check after 1 ms");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

private:
  // Number of pre-allocated device memory buffers
  size_t buffer_count_;
  // requested size of each device memory buffer
  size_t buffer_step_;
  // Index of the current device memory buffer
  size_t current_handle_index_ = 0;
  // Allocated size of each device memory buffer
  size_t alloc_size_;
  // Timeout in microseconds to reset the buffer
  int64_t timeout_;
  // Vector of device memory buffers
  std::vector<std::shared_ptr<DeviceIPCBuffer>> device_ipc_buffers_;
  // Vector of shared memory buffers contain refcount and UID
  std::vector<std::shared_ptr<HostIPCBuffer>> host_ipc_buffers_;
};

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_BRIDGE_ROS1__IPC_BUFFER_MANAGER_HPP_
