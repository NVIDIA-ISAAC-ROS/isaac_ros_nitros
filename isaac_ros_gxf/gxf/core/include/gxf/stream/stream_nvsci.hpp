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
#ifndef NVIDIA_GXF_STREAM_STREAM_NVSCI_HPP_
#define NVIDIA_GXF_STREAM_STREAM_NVSCI_HPP_

#include <cuda_runtime.h>

#include "gxf/core/component.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

enum class SyncType {
  GXF_STREAM_SIGNALER_NONE = 0,       // Invalid value for signaler
  GXF_STREAM_SIGNALER_CUDA,           // Indicates CUDA is the signaler.
                                      // Used to indicate when some CUDA job is done.
  GXF_STREAM_WAITER_NONE,             // Invalid value for waiter
  GXF_STREAM_WAITER_CUDA,             // Indicates CUDA is the waiter.
                                      // Used to wait on some CUDA job to be completed.
};

struct Stream : public Component {
  virtual ~Stream() = default;

  // Allocates a nvsci sync object.
  virtual gxf_result_t allocate_sync_object(SyncType signalerList,
                                            SyncType waiterList, void** syncObj) {
    return GXF_NOT_IMPLEMENTED;
  }

  // Signal semaphore with cuda stream
  virtual gxf_result_t signalSemaphore() {
    return GXF_NOT_IMPLEMENTED;
  }

  // Wait semaphore with cuda stream
  virtual gxf_result_t waitSemaphore() {
    return GXF_NOT_IMPLEMENTED;
  }

  // Set cuda stream for the give sync type
  virtual gxf_result_t setCudaStream(SyncType syncType, cudaStream_t stream) {
    return GXF_NOT_IMPLEMENTED;
  }

  // Imports the semaphore which will be used for signalling/waiting based on the sync type
  // Streams preserves the semaphore internally and is not exposed to the clients
  virtual gxf_result_t importSemaphore(SyncType syncType) {
    return GXF_NOT_IMPLEMENTED;
  }
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STREAM_STREAM_NVSCI_HPP_
