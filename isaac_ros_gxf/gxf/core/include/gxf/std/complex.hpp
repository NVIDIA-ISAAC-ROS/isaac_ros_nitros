/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_STD_COMPLEX_HPP_
#define NVIDIA_GXF_STD_COMPLEX_HPP_

#include <math.h>
#include <cuda_runtime.h>   //NOLINT
#include <cuda/std/complex> //NOLINT

namespace nvidia {
namespace gxf {

using complex64 = cuda::std::complex<float>;
using complex128 = cuda::std::complex<double>;

}  // namespace gxf
}  // namespace nvidia

#endif
