// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2019-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "cuda_runtime.h"

namespace nvidia {
namespace isaac {

// Computes a positive integer x such that (x - 1) * b < a <= x * b
template<typename T>
T DivRoundUp(T a, T b) {
  return (a + b - T{1}) / b;
}

// Same as DivRoundUp but for all three elements. This is for example useful in computing
// launch parameters for CUDA kernels.
inline dim3 DivRoundUp(dim3 a, dim3 b) {
  return {
    DivRoundUp(a.x, b.x),
    DivRoundUp(a.y, b.y),
    DivRoundUp(a.z, b.z)
  };
}

}  // namespace isaac
}  // namespace nvidia
