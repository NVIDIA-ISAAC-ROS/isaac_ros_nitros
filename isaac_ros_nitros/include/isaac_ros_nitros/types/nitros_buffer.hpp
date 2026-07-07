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
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_BUFFER_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_BUFFER_HPP_

#include <cuda_runtime.h>

#include <memory>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>
#include <cstdio>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_common/cuda_stream.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosBuffer;

#define NITROS_BUF_DBG(...) RCLCPP_DEBUG(rclcpp::get_logger("NitrosBuffer"), __VA_ARGS__)

// Check if a CUDA stream is valid and usable (not null, and either ready or has pending work)
inline bool nitros_is_stream_usable(cudaStream_t s)
{
  if (s == nullptr) {return false;}
  const cudaError_t st = cudaStreamQuery(s);
  return (st == cudaSuccess) || (st == cudaErrorNotReady);
}


// State shared between WriteHandle and NitrosBuffer for coordinating event recording
struct HandleState
{
  enum class State
  {
    Unset,     // No write handle created yet
    InUse,     // Write handle is active
    Finalized  // Write event recorded, handle finalized
  };

  std::mutex mtx;
  State state{State::Unset};
  cudaStream_t write_stream{nullptr};
};

// Shared state between NitrosBuffer and ReadHandle(s) so that read events
// and their mutex stay alive as long as any ReadHandle or the buffer exists.
struct ReadEventState
{
  std::mutex mutex;
  std::vector<cudaEvent_t> events;
};

// Provide api to access the read-only device pointer
// Submit write event wait on consumer's stream on construction, record a CUDA event
// to the stream on destruction. The read handle should be kept alive until the
// consumer is done with the data
//
// Stream Synchronization Contract
// The event-based synchronization only works if the consumer uses the SAME stream
// (passed to get_read_handle) for ALL subsequent CUDA operations on the data.
// If the consumer uses a different stream, the NULL stream, or synchronous CUDA APIs
// (e.g., cudaMemcpy without stream), those operations will NOT be synchronized with
// the producer's write and may read incomplete/garbage data.
// If the consumer cannot guarantee stream usage, it must call cudaStreamSynchronize(stream)
class ReadHandle
{
public:
  ReadHandle() = default;
  ReadHandle(const ReadHandle &) = delete;
  ReadHandle & operator=(const ReadHandle &) = delete;
  ReadHandle(ReadHandle && other) noexcept
  : data_ptr_(other.data_ptr_), read_event_state_(std::move(other.read_event_state_)),
    stream_(other.stream_)
  {
    other.data_ptr_ = nullptr;
    other.stream_ = nullptr;
    NITROS_BUF_DBG("ReadHandle move-constructed stream=%p", static_cast<void *>(stream_));
  }
  ReadHandle & operator=(ReadHandle && other) noexcept
  {
    if (this != &other) {
      data_ptr_ = other.data_ptr_;
      read_event_state_ = std::move(other.read_event_state_);
      stream_ = other.stream_;
      other.data_ptr_ = nullptr;
      other.stream_ = nullptr;
    }
    NITROS_BUF_DBG("ReadHandle move-assigned stream=%p", static_cast<void *>(stream_));
    return *this;
  }
  ~ReadHandle()
  {
    if (read_event_state_ && nitros_is_stream_usable(stream_)) {
      cudaEvent_t ev{nullptr};
      if (cudaEventCreateWithFlags(
            &ev, cudaEventDisableTiming | cudaEventBlockingSync) == cudaSuccess)
      {
        try {
          CHECK_CUDA_ERROR(
            cudaEventRecord(ev, stream_),
            "cudaEventRecord in ReadHandle dtor stream=%p event=%p",
            static_cast<void *>(stream_), static_cast<void *>(ev));
          std::lock_guard<std::mutex> lg(read_event_state_->mutex);
          read_event_state_->events.push_back(ev);
          NITROS_BUF_DBG(
            "ReadHandle dtor recorded event=%p on stream=%p",
            static_cast<void *>(ev), static_cast<void *>(stream_));
        } catch (const std::exception & e) {
          NITROS_BUF_DBG("Caught exception in ReadHandle dtor: %s", e.what());
        }
      }
    }
  }

  // Get read-only device pointer
  const uint8_t * get_ptr() const {return data_ptr_;}

private:
  friend class NitrosBuffer;
  explicit ReadHandle(
    const uint8_t * data_ptr, std::shared_ptr<ReadEventState> read_event_state,
    cudaStream_t stream, cudaEvent_t write_event)
  : data_ptr_(data_ptr), read_event_state_(std::move(read_event_state)), stream_(stream)
  {
    if (write_event && nitros_is_stream_usable(stream_)) {
      const cudaError_t cuda_err = cudaStreamWaitEvent(stream_, write_event, 0);
      if (cuda_err != cudaSuccess) {
        std::stringstream ss;
        ss << "ReadHandle cudaStreamWaitEvent failed: " << cudaGetErrorName(cuda_err)
           << " (" << cudaGetErrorString(cuda_err) << ")";
        throw std::runtime_error(ss.str());
      }
      NITROS_BUF_DBG(
        "ReadHandle waited on write_event=%p",
        static_cast<void *>(write_event));
    } else {
      NITROS_BUF_DBG(
        "ReadHandle ctor stream=%p (no write_event, data already synchronized)",
        static_cast<void *>(stream_));
    }
  }

  const uint8_t * data_ptr_{nullptr};
  std::shared_ptr<ReadEventState> read_event_state_;
  cudaStream_t stream_{nullptr};
};

// Provide api to access the mutable device pointer
// Records a CUDA event to the producer's stream on destruction
// The write handle should be kept alive until the producer is done with the data
class WriteHandle
{
public:
  WriteHandle(const WriteHandle &) = delete;
  WriteHandle & operator=(const WriteHandle &) = delete;
  WriteHandle(WriteHandle && other) noexcept
  : data_ptr_(other.data_ptr_), write_event_ptr_(other.write_event_ptr_), stream_(other.stream_),
    state_(std::move(other.state_))
  {
    other.data_ptr_ = nullptr;
    other.write_event_ptr_ = nullptr;
    other.stream_ = nullptr;
    NITROS_BUF_DBG("WriteHandle move-constructed stream=%p", static_cast<void *>(stream_));
  }
  WriteHandle & operator=(WriteHandle && other) noexcept
  {
    if (this != &other) {
      data_ptr_ = other.data_ptr_;
      write_event_ptr_ = other.write_event_ptr_;
      stream_ = other.stream_;
      state_ = std::move(other.state_);
      other.data_ptr_ = nullptr;
      other.write_event_ptr_ = nullptr;
      other.stream_ = nullptr;
    }
    NITROS_BUF_DBG("WriteHandle move-assigned stream=%p", static_cast<void *>(stream_));
    return *this;
  }
  ~WriteHandle()
  {
    if (!state_) {return;}
    std::lock_guard<std::mutex> lock(state_->mtx);
    if (state_->state == HandleState::State::Finalized) {
      NITROS_BUF_DBG("WriteHandle dtor already finalized");
      return;
    }
    cudaStream_t ws = state_->write_stream ? state_->write_stream : stream_;
    if (nitros_is_stream_usable(ws)) {
      cudaEvent_t ev{nullptr};
      if (cudaEventCreateWithFlags(
            &ev, cudaEventDisableTiming | cudaEventBlockingSync) == cudaSuccess)
      {
        try {
          CHECK_CUDA_ERROR(
            cudaEventRecord(ev, ws),
            "cudaEventRecord in WriteHandle dtor stream=%p event=%p",
            static_cast<void *>(ws), static_cast<void *>(ev));
        } catch (const std::exception & e) {
          NITROS_BUF_DBG("Caught exception in WriteHandle dtor: %s", e.what());
        }

        if (write_event_ptr_) {
          if (*write_event_ptr_) {
            try {
              CHECK_CUDA_ERROR(
                cudaEventDestroy(*write_event_ptr_),
                "cudaEventDestroy in WriteHandle dtor event=%p",
                static_cast<void *>(*write_event_ptr_));
            } catch (const std::exception & e) {
              NITROS_BUF_DBG("Caught exception destroying old event: %s", e.what());
            }
          }
          *write_event_ptr_ = ev;
        }

        NITROS_BUF_DBG(
          "WriteHandle dtor set write_event=%p on stream=%p",
          static_cast<void *>(ev), static_cast<void *>(ws));
        state_->state = HandleState::State::Finalized;
        state_->write_stream = nullptr;
      }
    }
  }

  // Get mutable device pointer for writing
  uint8_t * get_ptr() {return data_ptr_;}

private:
  friend class NitrosBuffer;
  explicit WriteHandle(
    uint8_t * data_ptr, cudaEvent_t * write_event_ptr, cudaStream_t stream,
    std::shared_ptr<HandleState> state)
  : data_ptr_(data_ptr), write_event_ptr_(write_event_ptr), stream_(stream), state_(state)
  {
    NITROS_BUF_DBG("WriteHandle ctor stream=%p", static_cast<void *>(stream_));
  }

  uint8_t * data_ptr_{nullptr};
  cudaEvent_t * write_event_ptr_{nullptr};
  cudaStream_t stream_{nullptr};
  std::shared_ptr<HandleState> state_{nullptr};
};

// Device buffer wrapper with event-based synchronization between producers and consumers
//
// Manages CUDA device memory with automatic synchronization via CUDA events.
// Supports multiple concurrent readers and a single writer with proper ordering.
class NitrosBuffer
{
public:
  NitrosBuffer() = default;

  ~NitrosBuffer()
  {
    try {
      release();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("NitrosBuffer"),
        "Exception in ~NitrosBuffer: %s", e.what());
    }
  }

  // Take ownership of an existing device allocation
  // Memory will be freed via cudaFree on destruction
  explicit NitrosBuffer(void * ptr, size_t size)
  {
    device_ptr_.reset(static_cast<uint8_t *>(ptr));
    size_ = size;
    NITROS_BUF_DBG("NitrosBuffer ctor (ptr=%p size=%zu)", ptr, size);
  }

  // Construct with a custom deleter (e.g., from CUDAMemoryPool::deleter())
  // Memory will be freed via the provided deleter on destruction
  NitrosBuffer(void * ptr, size_t size, std::function<void(uint8_t *)> custom_deleter)
  : device_ptr_(static_cast<uint8_t *>(ptr), std::move(custom_deleter)), size_(size)
  {
    NITROS_BUF_DBG("NitrosBuffer ctor+deleter (ptr=%p size=%zu)", ptr, size);
  }

  // Create a read handle for consuming buffer data
  // Waits on write_event_ if present to ensure producer has finished
  ReadHandle get_read_handle(cudaStream_t stream)
  {
    NITROS_BUF_DBG("get_read_handle(stream=%p)", static_cast<void *>(stream));

    if (handle_state_) {
      if (handle_state_->state == HandleState::State::InUse) {
        NITROS_BUF_DBG("finalize_write_handle early");
        finalize_write_handle();
      } else if (handle_state_->state == HandleState::State::Unset) {
        RCLCPP_WARN(
          rclcpp::get_logger("NitrosBuffer"),
          "get_read_handle() called but write handle was never created. "
          "Upstream producer needs to synchronize with the buffer.");
      }
    }

    if (!read_event_state_) {
      read_event_state_ = std::make_shared<ReadEventState>();
    }
    return ReadHandle(
      device_ptr_.get(), read_event_state_, stream, write_event_);
  }

  // Create a write handle for producing buffer data
  // The handle will record write_event_ on destruction to signal completion
  // Write handle can only get once to ensure the GPU write is atomic.
  WriteHandle get_write_handle(cudaStream_t stream)
  {
    NITROS_BUF_DBG("get_write_handle(stream=%p)", static_cast<void *>(stream));
    // Runtime guard: disallow re-acquiring a writer once finalized
    if (handle_state_) {
      std::lock_guard<std::mutex> lk(handle_state_->mtx);
      if (handle_state_->state == HandleState::State::Finalized) {
        throw std::runtime_error(
          "NitrosBuffer: write already finalized; cannot acquire write handle");
      }
    }
    if (read_event_state_) {
      std::lock_guard<std::mutex> lg(read_event_state_->mutex);
      if (!read_event_state_->events.empty()) {
        throw std::runtime_error(
          "NitrosBuffer: read events exist; cannot re-acquire write handle");
      }
    }
    if (!handle_state_) {handle_state_ = std::make_shared<HandleState>();}
    handle_state_->state = HandleState::State::InUse;
    handle_state_->write_stream = stream;
    return WriteHandle(device_ptr_.get(), &write_event_, stream, handle_state_);
  }

  cudaError_t release()
  {
    size_t read_count = 0;
    if (read_event_state_) {
      std::lock_guard<std::mutex> lg(read_event_state_->mutex);
      read_count = read_event_state_->events.size();
    }
    NITROS_BUF_DBG(
      "release() spawning cleanup thread for %zu events",
      read_count + (write_event_ ? 1 : 0));

    cudaEvent_t write_ev = write_event_;
    auto event_state = std::move(read_event_state_);
    auto device_ptr_moved = std::move(device_ptr_);

    write_event_ = nullptr;
    read_event_state_.reset();
    size_ = 0;

    std::thread(
      [write_ev, event_state = std::move(event_state),
      device_ptr_moved = std::move(device_ptr_moved)]() mutable {
        if (write_ev) {
          CHECK_CUDA_ERROR(
            cudaEventSynchronize(write_ev),
            "cudaEventSynchronize in cleanup thread");
          CHECK_CUDA_ERROR(
            cudaEventDestroy(write_ev),
            "cudaEventDestroy in cleanup thread");
        }

        if (event_state) {
          std::lock_guard<std::mutex> lg(event_state->mutex);
          for (cudaEvent_t ev : event_state->events) {
            if (ev) {
              CHECK_CUDA_ERROR(
                cudaEventSynchronize(ev),
                "cudaEventSynchronize on read_event in cleanup thread");
              CHECK_CUDA_ERROR(
                cudaEventDestroy(ev),
                "cudaEventDestroy on read_event in cleanup thread");
            }
          }
          event_state->events.clear();
        }

        device_ptr_moved.reset();
      }).detach();

    return cudaSuccess;
  }

  // Force early recording of the producer completion event on the write stream
  // Use when the producer handle must be finalized before its destructor runs
  // (e.g., when creating a read handle while a write handle is still alive)
  void finalize_write_handle()
  {
    if (handle_state_ == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(handle_state_->mtx);
    if (handle_state_->state != HandleState::State::InUse) {
      return;
    }
    if (nitros_is_stream_usable(handle_state_->write_stream)) {
      cudaEvent_t ev{nullptr};
      if (cudaEventCreateWithFlags(
            &ev, cudaEventDisableTiming | cudaEventBlockingSync) == cudaSuccess)
      {
        CHECK_CUDA_ERROR(
          cudaEventRecord(ev, handle_state_->write_stream),
          "cudaEventRecord in finalize_write_handle stream=%p event=%p",
          static_cast<void *>(handle_state_->write_stream), static_cast<void *>(ev));

        if (write_event_) {
          CHECK_CUDA_ERROR(
            cudaEventDestroy(write_event_),
            "cudaEventDestroy in finalize_write_handle event=%p",
            static_cast<void *>(write_event_));
        }
        write_event_ = ev;

        NITROS_BUF_DBG(
          "finalize set write_event=%p on stream=%p",
          static_cast<void *>(ev), static_cast<void *>(handle_state_->write_stream));
        handle_state_->state = HandleState::State::Finalized;
        handle_state_->write_stream = nullptr;
      }
    }
  }

  size_t size() const {return size_;}

  // Back-door access to the device pointer with synchronization
  // INTERIM: For GXF compatibility only. Will be removed when all nodes are GXF-free
  // WARNING: This method blocks on write_event_ synchronization
  const uint8_t * get_data() const
  {
    if (write_event_) {
      CHECK_CUDA_ERROR(
        cudaEventSynchronize(write_event_),
        "cudaEventSynchronize failed in get_data() event=%p",
        static_cast<void *>(write_event_));
    }
    return device_ptr_.get();
  }

private:
  static void default_cuda_free(uint8_t * p)
  {
    if (p) {
      NITROS_BUF_DBG("default_cuda_free(ptr=%p)", static_cast<void *>(p)); cudaFree(p);
    }
  }
  // Use type-erased deleter so callers can pass a pool's recycle function
  std::unique_ptr<uint8_t, std::function<void(uint8_t *)>> device_ptr_{
    nullptr,
    default_cuda_free
  };
  size_t size_{0};
  cudaEvent_t write_event_{nullptr};
  std::shared_ptr<ReadEventState> read_event_state_;
  std::shared_ptr<HandleState> handle_state_{nullptr};
};

// Template accessor for controlled buffer access from wrapper implementations
template<typename MessageType>
class NitrosBufferAccessor
{
public:
  static std::shared_ptr<NitrosBuffer> get_buffer(const MessageType & msg)
  {
    return msg.buffer_;
  }
  static std::shared_ptr<NitrosBuffer> get_buffer(MessageType & msg)
  {
    return msg.buffer_;
  }
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_BUFFER_HPP_
