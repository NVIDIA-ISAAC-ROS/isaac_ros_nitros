// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_CUDA_TESTS_CONVOLUTION_HPP
#define NVIDIA_GXF_CUDA_TESTS_CONVOLUTION_HPP

#include <cuda.h>

#define THREADS_PER_BLOCK 32
#define THREADS_PER_BLOCK_1 (THREADS_PER_BLOCK - 1)

// A simple GPU based 2d convolution operation
extern "C" void convolveKernel(float *input, float *kernel,
                               float *output, int width, int height,
                               int kernelSize, cudaStream_t stream);

#endif // NVIDIA_GXF_CUDA_TESTS_CONVOLUTION_HPP
