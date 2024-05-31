// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "dlpack/dlpack.h"
#include "gxf/core/expected.hpp"

namespace nvidia {
namespace gxf {

// Determine the GPU device (if any) associated with a raw pointer
Expected<DLDevice> DLDeviceFromPointer(void* ptr);

/**
 * @brief Fill strides from the given DLTensor object.
 *
 * The following fields are used to fill strides:
 *
 * - ndim
 * - shape
 * - dtype
 *
 * If tensor's strides is nullptr, `strides` argument is filled with the calculated strides of the
 * given DLTensor object. Otherwise, `strides` argument is filled with the given DLTensor object's
 * strides. `strides` vector would be resized to the size of `ndim` field of the given DLTensor
 * object.
 *
 * @param tensor DLTensor object that holds information to fill strides.
 * @param[out] strides Strides to fill.
 * @param to_num_elments If true, the strides in `strides` argument are in number of elements, not
 * bytes (default: false).
 */
void ComputeDLPackStrides(const DLTensor& tensor, std::vector<int64_t>& strides,
                          bool to_num_elements = false);

// Return DLDataType object from a NumPy type string.
Expected<DLDataType> DLDataTypeFromTypeString(const std::string& typestr);

/**
 * @brief Return a string providing the basic type of the homogeneous array in NumPy.
 *
 * Note: This method assumes little-endian for now.
 *
 * @return A const character pointer that represents a string
 */
Expected<const char*> numpyTypestr(const DLDataType dtype);
}  // namespace gxf
}  // namespace nvidia
