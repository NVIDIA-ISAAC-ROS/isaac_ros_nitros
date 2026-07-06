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

#ifndef ISAAC_ROS_NITROS__TYPES__CUDA_STREAM_POOL_HPP_
#define ISAAC_ROS_NITROS__TYPES__CUDA_STREAM_POOL_HPP_

#include <cuda_runtime.h>

#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Thread-safe CUDA stream pool. Singleton via CudaStreamPool::instance().
//
// Two borrowing modes:
//
//   StreamHandle (RAII) -- for get_read_handle / from_pool:
//     auto stream_handle = pool.get_stream_handle();
//     auto read_handle = image.get_read_handle(stream_handle.get());
//     Declare stream_handle BEFORE read/write handle so reverse destruction
//     records the CUDA event before the stream returns to the pool.
//
//   Raw acquire/release -- for from_external with async deleter:
//     cudaStream_t s = pool.acquire();
//     auto deleter = [&pool, s](uint8_t* p) { cudaFreeAsync(p, s); pool.release(s); };
//     Stream stays checked out until buffer destruction fires the deleter.
//
// WARNING: This pool is designed for stateless ROS2 type adapter conversions where
// each message is independent. Each acquire() may return a different stream,
// so concurrent calls can run GPU work in parallel with no ordering. If your
// node relies on frame-to-frame serialization (e.g. shared mutable GPU state),
// use a dedicated per-node stream instead.

constexpr size_t kDefaultStreamPoolSize = 32;

class CudaStreamPool
{
public:
  // RAII handle returned by get_stream_handle(). Automatically returns the
  // stream to the pool on destruction. Declare BEFORE any ReadHandle or
  // WriteHandle that uses the stream so that C++ reverse destruction order
  // ensures the handle records its CUDA event before the stream goes back
  // to the pool.
  class StreamHandle
  {
public:
    StreamHandle(const StreamHandle &) = delete;
    StreamHandle & operator=(const StreamHandle &) = delete;
    StreamHandle(StreamHandle && other) noexcept
    : pool_(other.pool_), stream_(other.stream_) {other.stream_ = nullptr;}
    StreamHandle & operator=(StreamHandle &&) = delete;
    ~StreamHandle() {if (stream_) {pool_.release(stream_);}}
    cudaStream_t get() const {return stream_;}

private:
    friend class CudaStreamPool;
    StreamHandle(CudaStreamPool & pool, cudaStream_t s)
    : pool_(pool), stream_(s) {}
    CudaStreamPool & pool_;
    cudaStream_t stream_;
  };

  explicit CudaStreamPool(
    size_t initial_size = kDefaultStreamPoolSize,
    unsigned int flags = cudaStreamNonBlocking)
  : flags_(flags)
  {
    available_.reserve(initial_size);
    for (size_t i = 0; i < initial_size; ++i) {
      cudaStream_t s = nullptr;
      cudaError_t err = cudaStreamCreateWithFlags(&s, flags_);
      if (err != cudaSuccess) {
        clear();
        throw std::runtime_error("CudaStreamPool: failed to create initial stream");
      }
      available_.push_back(s);
    }
  }

  ~CudaStreamPool()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    clear();
  }

  CudaStreamPool(const CudaStreamPool &) = delete;
  CudaStreamPool & operator=(const CudaStreamPool &) = delete;

  // Acquire a stream via RAII handle. Stream is returned when handle is destroyed.
  StreamHandle get_stream_handle() {return StreamHandle(*this, acquire());}

  // Acquire a raw stream. Caller must call release() manually.
  cudaStream_t acquire()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!available_.empty()) {
      cudaStream_t s = available_.back();
      available_.pop_back();
      return s;
    }
    RCLCPP_WARN(
      rclcpp::get_logger("CudaStreamPool"),
      "Pool exhausted, creating overflow stream (consider increasing kDefaultStreamPoolSize)");
    cudaStream_t s = nullptr;
    cudaError_t err = cudaStreamCreateWithFlags(&s, flags_);
    if (err != cudaSuccess) {
      throw std::runtime_error("CudaStreamPool: failed to create overflow stream");
    }
    return s;
  }

  // Return a raw stream to the pool. Null streams are silently ignored.
  void release(cudaStream_t stream)
  {
    if (stream == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    available_.push_back(stream);
  }

  size_t available() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return available_.size();
  }

  static CudaStreamPool & instance()
  {
    static CudaStreamPool pool(kDefaultStreamPoolSize, cudaStreamNonBlocking);
    return pool;
  }

private:
  void clear()
  {
    for (cudaStream_t s : available_) {
      cudaStreamDestroy(s);
    }
    available_.clear();
  }

  mutable std::mutex mutex_;
  std::vector<cudaStream_t> available_;
  unsigned int flags_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__CUDA_STREAM_POOL_HPP_
