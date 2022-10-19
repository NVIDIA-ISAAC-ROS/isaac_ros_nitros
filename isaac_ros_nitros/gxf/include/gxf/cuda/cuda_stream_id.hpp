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
#ifndef NVIDIA_GXF_CUDA_CUDA_STREAM_ID_HPP_
#define NVIDIA_GXF_CUDA_CUDA_STREAM_ID_HPP_

#include "gxf/core/gxf.h"


namespace nvidia {
namespace gxf {

// The Structure indicates cuda stream component ID.
// Message entity carrying CudaStreamId indicates that Tensors will be or
// has been proccessed by corresponding cuda stream. The handle could
// be deduced by Handle<CudaStream>::Create(context, stream_cid).
struct CudaStreamId {
  // component id of CudaStream
  gxf_uid_t stream_cid;
};


}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CUDA_CUDA_STREAM_ID_HPP_
