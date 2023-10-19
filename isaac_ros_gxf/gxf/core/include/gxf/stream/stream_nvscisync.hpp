// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STREAM_STREAM_NVSCISYNC_HPP_
#define NVIDIA_GXF_STREAM_STREAM_NVSCISYNC_HPP_

#include <nvscisync.h>

#include "gxf/core/component.hpp"
#include "gxf/core/gxf.h"
#include "gxf/stream/stream_nvsci.hpp"

namespace nvidia {
namespace gxf {

class StreamSync: public Stream {
 public:
  StreamSync() = default;
  ~StreamSync() = default;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;
  gxf_result_t allocate_sync_object(SyncType signaler, SyncType waiter, void** syncObj) override;
  gxf_result_t signalSemaphore() override;
  gxf_result_t waitSemaphore() override;
  gxf_result_t setCudaStream(SyncType syncType, cudaStream_t stream) override;
  gxf_result_t importSemaphore(SyncType syncType) override;

 private:
  gxf_result_t importSemaphore(cudaExternalSemaphore_t* semaphore, SyncType syncType);

  NvSciSyncModule         sync_module_{nullptr};
  NvSciSyncAttrList       attr_list_{nullptr};
  NvSciSyncAttrList       reconciled_attr_list_{nullptr};
  NvSciSyncObj            sync_obj_{nullptr};
  NvSciSyncFence*         fence_;
  cudaExternalSemaphore_t signaler_semaphore_;
  cudaExternalSemaphore_t waiter_semaphore_;
  cudaStream_t            signaler_cuda_stream_{};
  cudaStream_t            waiter_cuda_stream_{};
  bool                    is_signaler_semaphore_imported_{false};
  bool                    is_waiter_semaphore_imported_{false};
  int32_t                 num_gpus_{0};

  Parameter<int32_t> signaler_device_id_;
  Parameter<int32_t> waiter_device_id_;
  Parameter<int32_t> signaler_;
  Parameter<int32_t> waiter_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STREAM_STREAM_NVSCISYNC_HPP_
