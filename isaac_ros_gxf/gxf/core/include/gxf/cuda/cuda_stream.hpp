// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_CUDA_CUDA_STREAM_HPP_
#define NVIDIA_GXF_CUDA_CUDA_STREAM_HPP_

#include <cuda_runtime.h>

#include <functional>
#include <mutex>
#include <queue>
#include <vector>

#include "gxf/core/component.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/cuda/cuda_event.hpp"


namespace nvidia {
namespace gxf {

// Holds and provides access to cudaStream_t. CudaStream is allocated and
// recycled by CudaStreamPool
class CudaStream {
 public:
  CudaStream() = default;
  ~CudaStream();

  CudaStream(const CudaStream&) = delete;
  CudaStream(CudaStream&&) = delete;
  void operator=(const CudaStream&) = delete;

  using EventDestroy = std::function<void(cudaEvent_t)>;
  using SyncedCallback = std::function<void()>;

  // Retrieves cudaSteam_t
  Expected<cudaStream_t> stream() const;
  // Get device id which owns this stream
  int dev_id() const { return dev_id_; }

  // Record event to extend Entity life until event synchronized.
  Expected<void> record(Handle<CudaEvent> event, Entity input_entity,
                        SyncedCallback synced_cb = nullptr);
  // Record event on the stream for an async callback.
  // The callback would be delayed until CudaStreamSync ticks.
  // The Callback usually is used to recycle dependent resources.
  // If record failed, callback would not be called. User need to check return results.
  Expected<void> record(cudaEvent_t event, EventDestroy cb);

  // Reset all events and callback all the hook functions to release resource.
  Expected<void> resetEvents();

  // Sync all streams, meanwhile clean all recorded events and callback recycle functions
  Expected<void> syncStream();

 private:
  friend class CudaStreamPool;
  // Initialize new cuda stream if was not set by external
  Expected<void> initialize(uint32_t flags = 0, int dev_id = -1, int32_t priority = 0);
  Expected<void> deinitialize();

  Expected<void> prepareResourceInternal(int dev_id);

  Expected<void> recordEventInternal(cudaEvent_t e);
  Expected<void> syncEventInternal(cudaEvent_t e);

  Expected<void> resetEventsInternal(std::queue<CudaEvent::EventPtr>& q);

  mutable std::shared_timed_mutex mutex_;
  int dev_id_ = 0;
  cudaStream_t stream_ = 0;

  // store all recorded event with destroy functions.
  std::queue<CudaEvent::EventPtr> recorded_event_queue_;
  // event is defined for for synchronization of stream
  CudaEvent::EventPtr sync_event_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CUDA_CUDA_STREAM_HPP_
