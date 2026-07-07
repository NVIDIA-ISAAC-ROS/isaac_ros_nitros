// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS__TYPES__CUDA_MEMORY_POOL_HPP_
#define ISAAC_ROS_NITROS__TYPES__CUDA_MEMORY_POOL_HPP_

#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <condition_variable>
#include <chrono>
#include <mutex>
#include <utility>
#include <vector>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Logging helper
#define NITROS_POOL_DBG(...) RCLCPP_DEBUG(rclcpp::get_logger("CUDAMemoryPool"), __VA_ARGS__)

// Fixed-size CUDA memory pool
//
// Allocates a contiguous region and carves it into fixed-size blocks.
// Thread-safe acquire/recycle with optional timeout support.
//
// Usage:
//   1. create(block_size, block_count, memory_type) - allocate pool
//   2. acquire(&ptr) or acquire_for(&ptr, timeout) - get a block
//   3. Use the memory
//   4. recycle(ptr) or use deleter() with smart pointers - return block to pool
//   5. destroy() - free all memory (waits for all blocks to be recycled)
//
// Note: Single-ownership enforced (copy/move deleted) to prevent double-free
class CUDAMemoryPool
{
public:
  enum class MemoryType
  {
    Device,     // cudaMalloc / cudaFree
    Managed,    // cudaMallocManaged / cudaFree
    HostPinned  // cudaMallocHost / cudaFreeHost
  };

  CUDAMemoryPool() = default;
  ~CUDAMemoryPool() {destroy();}

  // Delete copy/move enforces single-ownership.
  // Multiple ownership could lead to double-free, memory corruption, etc.
  CUDAMemoryPool(const CUDAMemoryPool &) = delete;
  CUDAMemoryPool & operator=(const CUDAMemoryPool &) = delete;
  CUDAMemoryPool(CUDAMemoryPool &&) = delete;
  CUDAMemoryPool & operator=(CUDAMemoryPool &&) = delete;

  // Allocate pool storage and initialize the free list.
  cudaError_t create(size_t block_size_bytes, size_t block_count, MemoryType memory_type)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
      return cudaErrorInitializationError;
    }

    if (block_size_bytes == 0 || block_count == 0) {
      return cudaErrorInvalidValue;
    }

    memory_type_ = memory_type;
    block_size_ = block_size_bytes;
    block_count_ = block_count;
    const size_t total_bytes = block_size_ * block_count_;

    cudaError_t err = cudaSuccess;
    switch (memory_type_) {
      case MemoryType::Device:
        err = cudaMalloc(&base_ptr_, total_bytes);
        break;
      case MemoryType::Managed:
        err = cudaMallocManaged(&base_ptr_, total_bytes);
        break;
      case MemoryType::HostPinned:
        err = cudaMallocHost(&base_ptr_, total_bytes);
        break;
    }
    if (err != cudaSuccess) {
      base_ptr_ = nullptr;
      block_size_ = 0;
      block_count_ = 0;
      NITROS_POOL_DBG(
        "create failed pool=%p: %s (%s)",
        static_cast<void *>(this), cudaGetErrorName(err), cudaGetErrorString(err));
      return err;
    }

    free_stack_.reserve(block_count_);
    for (size_t i = 0; i < block_count_; ++i) {
      free_stack_.push_back(i);
    }
    in_use_.assign(block_count_, false);
    initialized_ = true;
    shutting_down_ = false;
    NITROS_POOL_DBG(
      "create OK pool=%p: block_size=%zu blocks=%zu type=%d base_ptr=%p",
      static_cast<void *>(this), block_size_, block_count_,
      static_cast<int>(memory_type_), base_ptr_);
    return cudaSuccess;
  }

  // Acquire the next available block. Returns cudaSuccess on success;
  // cudaErrorMemoryAllocation if exhausted.
  cudaError_t acquire(uint8_t ** out_ptr)
  {
    if (out_ptr == nullptr) {
      return cudaErrorInvalidValue;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || shutting_down_) {
      *out_ptr = nullptr;
      return cudaErrorInitializationError;
    }
    if (free_stack_.empty()) {
      *out_ptr = nullptr;
      NITROS_POOL_DBG("acquire pool=%p: no blocks available", static_cast<void *>(this));
      return cudaErrorMemoryAllocation;
    }
    const size_t idx = free_stack_.back();
    free_stack_.pop_back();
    in_use_[idx] = true;
    *out_ptr = block_ptr_from_index(idx);
    NITROS_POOL_DBG("acquire pool=%p: idx=%zu ptr=%p", static_cast<void *>(this), idx, *out_ptr);
    return cudaSuccess;
  }

  // Acquire with timeout. Waits up to 'timeout' for a free block; returns nullptr on timeout.
  template<typename Rep, typename Period>
  cudaError_t acquire_for(uint8_t ** out_ptr, const std::chrono::duration<Rep, Period> & timeout)
  {
    if (out_ptr == nullptr) {
      return cudaErrorInvalidValue;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (!initialized_ || shutting_down_) {
      *out_ptr = nullptr;
      return cudaErrorInitializationError;
    }
    if (free_stack_.empty()) {
      NITROS_POOL_DBG("acquire_for pool=%p: waiting", static_cast<void *>(this));
      const bool ready = cv_.wait_for(lock, timeout, [this]() {
            return !free_stack_.empty() || !initialized_ || shutting_down_;
      });
      if (!ready || !initialized_ || shutting_down_) {
        *out_ptr = nullptr;
        NITROS_POOL_DBG(
          "acquire_for pool=%p: timed out or uninitialized",
          static_cast<void *>(this));
        return cudaErrorNotReady;
      }
    }
    const size_t idx = free_stack_.back();
    free_stack_.pop_back();
    in_use_[idx] = true;
    *out_ptr = block_ptr_from_index(idx);
    NITROS_POOL_DBG(
      "acquire_for pool=%p: idx=%zu ptr=%p",
      static_cast<void *>(this), idx, *out_ptr);
    return cudaSuccess;
  }

  // Recycle a previously acquired block back to the pool.
  // Returns cudaSuccess on success; cudaErrorInvalidValue on invalid inputs.
  cudaError_t recycle(const void * ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || ptr == nullptr) {
      NITROS_POOL_DBG(
        "recycle pool=%p: invalid (initialized=%d ptr=%p)",
        static_cast<void *>(this), initialized_ ? 1 : 0, ptr);
      return cudaErrorInvalidValue;
    }
    const intptr_t diff = static_cast<const uint8_t *>(ptr) - static_cast<uint8_t *>(base_ptr_);
    if (diff < 0) {
      NITROS_POOL_DBG("recycle pool=%p: ptr before base", static_cast<void *>(this));
      return cudaErrorInvalidValue;
    }
    const size_t offset = static_cast<size_t>(diff);
    if (offset % block_size_ != 0) {
      NITROS_POOL_DBG("recycle pool=%p: ptr not block-aligned", static_cast<void *>(this));
      return cudaErrorInvalidValue;
    }
    const size_t idx = offset / block_size_;
    if (idx >= block_count_) {
      NITROS_POOL_DBG("recycle pool=%p: idx out of range", static_cast<void *>(this));
      return cudaErrorInvalidValue;
    }
    if (!in_use_[idx]) {
      NITROS_POOL_DBG(
        "recycle pool=%p: block not in use idx=%zu",
        static_cast<void *>(this), idx);
      return cudaErrorInvalidValue;
    }
    in_use_[idx] = false;
    free_stack_.push_back(idx);
    cv_.notify_one();
    NITROS_POOL_DBG("recycle pool=%p: idx=%zu", static_cast<void *>(this), idx);
    return cudaSuccess;
  }

  // Custom deleter for use with smart pointers.
  // Example: std::shared_ptr<uint8_t> p(pool.acquire(), pool.deleter());
  std::function<void(uint8_t *)> deleter()
  {
    return [this](uint8_t * p) {
             if (p != nullptr) {
               NITROS_POOL_DBG(
          "deleter pool=%p: ptr=%p",
          static_cast<void *>(this), static_cast<void *>(p));
               (void)this->recycle(p);
             }
           };
  }

  // Free underlying storage and reset state.
  cudaError_t destroy()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!initialized_) {
      return cudaSuccess;
    }
    // Prevent new acquires; allow recycle() to proceed and notify
    shutting_down_ = true;
    // Wake up all threads waiting in the wait_for() functions and exit immediately.
    cv_.notify_all();
    // Wait until all blocks are recycled.
    cv_.wait(lock, [this]() {return free_stack_.size() == block_count_;});

    cudaError_t err = cudaSuccess;
    switch (memory_type_) {
      case MemoryType::Device:
      case MemoryType::Managed:
        // Both cudaMalloc and cudaMallocManaged are paired with cudaFree
        // Use synchronized call to ensure all GPU work is completed.
        err = cudaFree(base_ptr_);
        break;
      case MemoryType::HostPinned:
        err = cudaFreeHost(base_ptr_);
        break;
    }

    NITROS_POOL_DBG(
      "destroy pool=%p: base_ptr=%p err=%s",
      static_cast<void *>(this), base_ptr_, cudaGetErrorName(err));
    base_ptr_ = nullptr;
    block_size_ = 0;
    block_count_ = 0;
    initialized_ = false;
    shutting_down_ = false;
    memory_type_ = MemoryType::Device;
    free_stack_.clear();
    in_use_.clear();
    cv_.notify_all();
    return err;
  }

  // Accessors
  bool initialized() const {return initialized_;}
  size_t block_size() const {return block_size_;}
  size_t block_count() const {return block_count_;}
  MemoryType memory_type() const {return memory_type_;}
  size_t available_blocks() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return 0;
    }
    return free_stack_.size();
  }

private:
  uint8_t * block_ptr_from_index(size_t idx) const
  {
    return static_cast<uint8_t *>(base_ptr_) + idx * block_size_;
  }

  void * base_ptr_{nullptr};
  size_t block_size_{0};
  size_t block_count_{0};
  MemoryType memory_type_{MemoryType::Device};
  bool initialized_{false};

  std::vector<size_t> free_stack_;
  std::vector<bool> in_use_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool shutting_down_{false};
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__CUDA_MEMORY_POOL_HPP_
