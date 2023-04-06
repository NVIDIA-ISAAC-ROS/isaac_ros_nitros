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
#ifndef NVIDIA_GXF_CUDA_CUDA_COMMON_HPP_
#define NVIDIA_GXF_CUDA_CUDA_COMMON_HPP_

#include <cuda_runtime.h>

#include "common/assert.hpp"
#include "common/logger.hpp"

#define CHECK_CUDA_ERROR(cu_result, fmt, ...)                                  \
    do {                                                                       \
        cudaError_t err = (cu_result);                                          \
        if (err != cudaSuccess) {                                               \
            GXF_LOG_ERROR(fmt ", cuda_error: %s, error_str: %s", ##__VA_ARGS__, \
                cudaGetErrorName(err), cudaGetErrorString(err));                \
            return Unexpected{GXF_FAILURE};                                     \
        }                                                                       \
    } while (0)

#define CONTINUE_CUDA_ERROR(cu_result, fmt, ...)                               \
    do {                                                                       \
        cudaError_t err = (cu_result);                                          \
        if (err != cudaSuccess) {                                               \
            GXF_LOG_ERROR(fmt ", cuda_error: %s, error_str: %s", ##__VA_ARGS__, \
                cudaGetErrorName(err), cudaGetErrorString(err));                \
        }                                                                       \
    } while (0)

#endif  // NVIDIA_GXF_CUDA_CUDA_COMMON_HPP_
