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
#ifndef NVIDIA_GXF_STD_CUDA_GREEN_CONTEXT_HPP_
#define NVIDIA_GXF_STD_CUDA_GREEN_CONTEXT_HPP_

#include "cuda.h"
#include <cuda_runtime.h>

#include <mutex>
#include <string>

#include "gxf/core/component.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/std/cuda_green_context_pool.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief Holds and provides access to CUgreenCtx. CudaGreenContext is allocated and
 * recycled by CudaGreenContextPool
 */
class CudaGreenContext : public Component {
 public:
  CudaGreenContext() = default;
  ~CudaGreenContext();

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  // Retrieves CUgreenCtx
  Expected<CUgreenCtx> greenContext() const;

  // Retrieves CUcontext that associated with this green context
  Expected<CUcontext> cudaContext();

  uint32_t index() const { return pool_index_; }

  CudaGreenContextPool* cudaGreenContextPool() const {
    return cuda_green_context_pool_.try_get().value();
  }

 private:
  Parameter<int32_t> index_;
  Parameter<Handle<CudaGreenContextPool>> cuda_green_context_pool_;
  Parameter<std::string> nvtx_identifier_;

  CUgreenCtx green_context_ = nullptr;
  CUcontext cuda_context_ = nullptr;
  uint32_t pool_index_ = 0;
  mutable std::shared_timed_mutex mutex_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_CUDA_GREEN_CONTEXT_HPP_
