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
#ifndef NVIDIA_GXF_CUDA_CUDA_STREAM_SYNC_HPP_
#define NVIDIA_GXF_CUDA_CUDA_STREAM_SYNC_HPP_

#include <cuda_runtime.h>

#include "gxf/cuda/cuda_stream.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"


namespace nvidia {
namespace gxf {

// Get all cuda streams from messages of receiver, synchronize these streams,
// and forward messages to transmitter.
class CudaStreamSync  : public Codelet {
 public:
  virtual ~CudaStreamSync() = default;
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  Parameter<Handle<Receiver>> rx_;
  Parameter<Handle<Transmitter>> tx_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CUDA_CUDA_STREAM_SYNC_HPP_
