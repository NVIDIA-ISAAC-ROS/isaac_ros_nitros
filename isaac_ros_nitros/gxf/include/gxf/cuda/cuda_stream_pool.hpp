/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_GXF_CUDA_CUDA_STREAM_POOL_HPP_
#define NVIDIA_GXF_CUDA_CUDA_STREAM_POOL_HPP_

#include <cuda_runtime.h>

#include <memory>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>

#include "gxf/core/component.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/cuda/cuda_stream.hpp"
#include "gxf/std/allocator.hpp"

namespace nvidia {
namespace gxf {

// A memory pools which provides a maximum number of equally sized blocks of
// memory.
class CudaStreamPool : public Allocator {
 public:
  CudaStreamPool() = default;
  ~CudaStreamPool();

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t is_available_abi(uint64_t size) override;
  gxf_result_t allocate_abi(uint64_t size, int32_t type, void** pointer) override;
  gxf_result_t free_abi(void* pointer) override;
  gxf_result_t deinitialize() override;

  // Allocate a cudastream for other components
  Expected<Handle<CudaStream>> allocateStream();
  // Free a cudastream
  Expected<void> releaseStream(Handle<CudaStream> stream);

 private:
  Expected<Entity> createNewStreamEntity();
  Expected<void> reserveStreams();

  Parameter<int32_t> dev_id_;
  Parameter<uint32_t> stream_flags_;
  Parameter<int32_t> stream_priority_;
  Parameter<uint32_t> reserved_size_;
  Parameter<uint32_t> max_size_;

  std::mutex mutex_;
  // map of <entity_id, Entity>
  std::unordered_map<gxf_uid_t, std::unique_ptr<Entity>> streams_;
  std::queue<Entity> reserved_streams_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CUDA_CUDA_STREAM_POOL_HPP_
