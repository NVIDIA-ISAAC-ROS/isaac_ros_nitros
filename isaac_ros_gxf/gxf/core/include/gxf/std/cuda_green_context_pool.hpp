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

#ifndef NVIDIA_GXF_STD_CUDA_GREEN_CONTEXT_POOL_HPP_
#define NVIDIA_GXF_STD_CUDA_GREEN_CONTEXT_POOL_HPP_

#include "cuda.h"
#include <cuda_runtime.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "gxf/core/component.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/resources.hpp"

#define CHECK_CUDA_ERROR_RESULT(cu_result, fmt, ...)                      \
  do {                                                                    \
    cudaError_t err = (cu_result);                                        \
    if (err != cudaSuccess) {                                             \
      GXF_LOG_ERROR(fmt ", cuda_error: %s, error_str: %s", ##__VA_ARGS__, \
                    cudaGetErrorName(err), cudaGetErrorString(err));      \
      return GXF_FAILURE;                                                 \
    }                                                                     \
  } while (0)

#define CHECK_CUDA_DRV_ERROR(cu_result, driver_fn_table, fmt, ...)          \
  do {                                                                      \
    if (CUDA_SUCCESS != cu_result) {                                        \
        const char* errorName = nullptr;                                    \
        const char* errorString = nullptr;                                  \
        driver_fn_table.cuGetErrorName(cu_result, &errorName);             \
        driver_fn_table.cuGetErrorString(cu_result, &errorString);         \
        GXF_LOG_ERROR(fmt ", cuda driver API failed with error: %s:%s ",    \
                      errorName, errorString);                              \
      return GXF_FAILURE;                                                   \
    }                                                                       \
  } while (0)

#define GET_CUDA_DRV_API_PTR(func, version, name)                           \
  do {                                                                      \
    cudaDriverEntryPointQueryResult driver_status;                          \
    auto result = cudaGetDriverEntryPointByVersion(name,                    \
        reinterpret_cast<void**>(&func),                                    \
        version,                                                            \
        cudaEnableDefault,                                                  \
        &driver_status);                                                    \
    if (func == nullptr) {                                                  \
      GXF_LOG_ERROR("Failed to load %s from CUDA runtime library, "         \
        "error[%d], driver_status: %d", name, result, driver_status);       \
      return GXF_FAILURE;                                                   \
    }                                                                       \
  } while (0)

namespace nvidia {
namespace gxf {

struct CudaDriverFunctionTable {
  CUresult (*cuDeviceGet)(CUdevice*, int) = nullptr;
  CUresult (*cuGreenCtxGetDevResource)(CUgreenCtx, CUdevResource*, CUdevResourceType) = nullptr;
  CUresult (*cuGreenCtxCreate)(CUgreenCtx*, CUdevResourceDesc, CUdevice, unsigned int) = nullptr;
  CUresult (*cuDevSmResourceSplitByCount)(
      CUdevResource*, unsigned int*, const CUdevResource*, CUdevResource*, unsigned int,
      unsigned int) = nullptr;
  CUresult (*cuCtxFromGreenCtx)(CUcontext*, CUgreenCtx) = nullptr;
  CUresult (*cuDevResourceGenerateDesc)(
      CUdevResourceDesc*, CUdevResource*, unsigned int) = nullptr;
  CUresult (*cuDeviceGetDevResource)(
      CUdevice, CUdevResource*, CUdevResourceType) = nullptr;
  CUresult (*cuGetErrorName)(CUresult, const char**) = nullptr;
  CUresult (*cuGetErrorString)(CUresult, const char**) = nullptr;
};

/**
 * @brief A pool containing a specified number of CUDA Green Contexts on a single device.
 */
class CudaGreenContextPool : public Component {
 public:
  CudaGreenContextPool() = default;
  ~CudaGreenContextPool();

  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;
  gxf_result_t registerInterface(Registrar* registrar) override;

  Expected<CUgreenCtx> getGreenContext(uint32_t index);
  Expected<CUcontext> getCudaContext(uint32_t index);
  Expected<uint32_t> getPartitionSms(uint32_t index);
  Expected<uint32_t> getDefaultContextIndex();
  Expected<CUgreenCtx> getDefaultContext();
  Expected<uint32_t> getDeviceTotalSms(uint32_t index);

 private:
  gxf_result_t reserveGreenContexts(int32_t compute_capability_major);
  gxf_result_t loadCudaDriver();

  Resource<Handle<GPUDevice>> gpu_device_;
  Parameter<uint32_t> green_context_flags_;
  Parameter<uint32_t> num_partitions_;
  Parameter<uint32_t> min_sm_count_;
  Parameter<std::vector<int32_t>> sms_per_partition_;
  Parameter<std::string> nvtx_identifier_;
  Parameter<int32_t> default_context_;

  std::atomic<bool> initialized_{false};
  std::mutex mutex_;
  std::vector<CUgreenCtx> green_context_;
  std::vector<CUcontext> cu_context_;
  uint32_t max_sm_count_ = 0;
  int32_t dev_id_ = -1;
  uint32_t default_index_ = 0;

  // Function table for CUDA driver symbols
  CudaDriverFunctionTable cuda_driver_fn_table_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_CUDA_GREEN_CONTEXT_POOL_HPP_
