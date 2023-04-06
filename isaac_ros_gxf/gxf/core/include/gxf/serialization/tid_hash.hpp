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
#ifndef NVIDIA_GXF_SERIALIZATION_TID_HASH_HPP_
#define NVIDIA_GXF_SERIALIZATION_TID_HASH_HPP_

#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

// Hash function for gxf_tid_t that XORs upper 64 bits with lower 64 bits
struct TidHash {
  size_t operator()(const gxf_tid_t& tid) const { return tid.hash1 ^ tid.hash2; }
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_TID_HASH_HPP_
