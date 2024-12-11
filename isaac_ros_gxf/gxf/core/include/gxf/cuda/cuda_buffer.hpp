// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_CUDA_CUDA_BUFFER_HPP_
#define NVIDIA_GXF_CUDA_CUDA_BUFFER_HPP_

#include <atomic>
#include <utility>

#include "common/byte.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/cuda/cuda_allocator.hpp"
#include "gxf/cuda/cuda_common.hpp"
#include "gxf/cuda/cuda_stream.hpp"
#include "gxf/cuda/cuda_stream_pool.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/memory_buffer.hpp"

namespace nvidia {
namespace gxf {

constexpr const char* kDefaultCudaBufferName = "gxf_cuda_buffer";

/**
 * @brief A container for a single block of cuda memory with support for async
          memory allocations. Each CudaBuffer is type of MemoryBuffer associated with a
          CudaAllocator, CudaStreamPool & CudaStream. Memory is allocated by resizing the
          buffer. Both cuda memory and the cuda stream is released back into the pool
          when a cuda buffer object is destroyed.
 */
class CudaBuffer : public MemoryBuffer {
 public:
  // state of the buffer
  enum class State : int8_t {
    UNSET = 0,                // Initial state not waiting for any cuda event callback
    CALLBACK_REGISTERED,      // Indicates a call back function is registered
    DATA_AVAILABLE,           // Indicates data is ready to be consumed
  };

  CudaBuffer() = default;
  CudaBuffer(const CudaBuffer&) = delete;
  CudaBuffer& operator=(const CudaBuffer&) = delete;

  CudaBuffer(CudaBuffer&& other) noexcept { *this = std::move(other); }

  CudaBuffer& operator=(CudaBuffer&& other) noexcept {
    if (this == &other) {
      return *this;
    }
    state_ = other.state_.load();
    release_func_ =  std::move(other.release_func_);
    stream_ = std::move(other.stream_);
    stream_owner_ = other.stream_owner_.load();
    stream_pool_ = std::move(other.stream_pool_);

    other.state_ = State::UNSET;
    other.stream_owner_ = false;
    other.release_func_ = nullptr;
    return *this;
  }

  // callback function to release memory passed to the CudaBuffer
  using release_function_t = std::function<Expected<void> (void* pointer)>;

  // Host callback function prototype
  using callback_function_t = void(*)(void*);

  static void CUDART_CB cudaHostCallback(void* buffer_ptr) {
    auto buffer = reinterpret_cast<CudaBuffer*>(buffer_ptr);
    GXF_LOG_VERBOSE("Received host callback from cuda stream for cuda buffer");
    auto cb_registered = State::CALLBACK_REGISTERED;
    GXF_ASSERT_TRUE(buffer->state_.compare_exchange_strong(cb_registered, State::DATA_AVAILABLE));
  }

  Expected<void> freeBuffer() {
    auto state = state_.load();
    // If another callback is already registered, buffer should not be freed
    GXF_ASSERT_NE(static_cast<int>(state), static_cast<int>(State::CALLBACK_REGISTERED));
    state_ = State::UNSET;
    if (release_func_ && pointer_) {
      const Expected<void> result = release_func_(pointer_);
      if (!result) {
        return ForwardError(result);
      }
      release_func_ = nullptr;
    }

    return Success;
  }

  ~CudaBuffer() { freeBuffer(); }

  // Resizes the cuda buffer to given size on custom cuda stream. A new stream is allocated
  // from the stream pool if a pre allocated stream is not provided.
  Expected<void> resizeAsync(Handle<CudaAllocator> allocator,
                             Handle<CudaStreamPool> stream_pool,
                             uint64_t size,
                             Handle<CudaStream> stream_handle = Handle<CudaStream>::Null()) {
    if (!allocator) {
      GXF_LOG_ERROR("Invalid cuda allocator provided while resizing a cuda buffer");
      return Unexpected{GXF_ARGUMENT_INVALID};
    }

    if (!stream_pool) {
      GXF_LOG_ERROR("Invalid cuda stream pool provided while resizing a cuda buffer");
      return Unexpected{GXF_ARGUMENT_INVALID};
    }

    const auto code = freeBuffer();
    if (!code) {
      GXF_LOG_ERROR("Failed to free memory. Error code: %s", GxfResultStr(code.error()));
      return ForwardError(code);
    }

    // Allocate new stream if its not pre-allocated
    if (!stream_handle) {
      stream_handle = GXF_UNWRAP_OR_RETURN(stream_pool->allocateStream());
    }
    auto stream = GXF_UNWRAP_OR_RETURN(stream_handle->stream());
    auto result = GXF_UNWRAP_OR_RETURN(allocator->allocate_async(size, stream));

    GXF_LOG_VERBOSE("Registering callback for cuda buffer");
    auto unset = State::UNSET;
    GXF_ASSERT_TRUE(state_.compare_exchange_strong(unset, State::CALLBACK_REGISTERED));
    cudaError_t cuda_result = cudaLaunchHostFunc(stream, cudaHostCallback, this);
    CHECK_CUDA_ERROR(cuda_result,
      "Unable to register host function using cudaLaunchHostFunc");

    storage_type_ = MemoryStorageType::kDevice;
    pointer_ = result;
    size_ = size;

    // TODO(chandrahasj) should allow custom stream to release the data ?
    release_func_ = [allocator, stream_pool, stream_handle, this] (void *data) {
      auto stream = stream_handle->stream().value();
      allocator->free_async(reinterpret_cast<byte*>(data), stream);
      if (this->stream_owner_.load()) {
        // synchronize the stream before releasing it back into the pool
        auto code = cudaStreamSynchronize(stream_handle->stream().value());
        if (code != cudaSuccess) {
          GXF_LOG_ERROR("Failed to synchronize cuda stream");
          return ExpectedOrCode(GXF_FAILURE);
        }
        return stream_pool->releaseStream(stream_handle);
      }
      return Success;
    };

    stream_ = stream_handle;
    stream_pool_ = stream_pool;
    return Success;
  }

  // Retrieves cuda buffer state
  State state() const { return state_.load(); }

  // Retrieves CudaStream
  Handle<CudaStream> stream() {
    return stream_;
  }

  // Retrieves CudaStreamPool
  Handle<CudaStreamPool> streamPool() {
    return stream_pool_;
  }

  // Ownership of CudaStream is transferred to the caller.
  // Stream will not be released when this object is destroyed.
  Handle<CudaStream> transferStreamOwnership() {
    stream_owner_ = false;
    return stream_;
  }

  // Register a host function callback on the cuda stream associated with the buffer
  Expected<void> registerCallbackOnStream(callback_function_t func, void* data) {
    if (!stream_) {
      GXF_LOG_ERROR("CudaStream is not set for the CudaBuffer");
      return Unexpected{GXF_FAILURE};
    }

    auto stream = GXF_UNWRAP_OR_RETURN(stream_->stream());
    GXF_LOG_VERBOSE("Registering callback for cuda buffer");
    cudaError_t cuda_result = cudaLaunchHostFunc(stream, func, data);
    CHECK_CUDA_ERROR(cuda_result, "Unable to register host function using cudaLaunchHostFunc");
    return Success;
  }

 private:
  // State of the buffer
  std::atomic<State> state_{State::UNSET};
  // Flag to keep track of shared ownership of the cuda stream
  std::atomic<bool> stream_owner_{true};
  // cuda host callback function on completion of all the queued work
  release_function_t release_func_ = nullptr;
  // CudaStream used to allocate memory for the cuda buffer
  Handle<CudaStream> stream_ = Handle<CudaStream>::Null();
  // CudaStreamPool used to allocate the CudaStream used by the buffer
  Handle<CudaStreamPool> stream_pool_ = Handle<CudaStreamPool>::Null();
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CUDA_CUDA_BUFFER_HPP_
