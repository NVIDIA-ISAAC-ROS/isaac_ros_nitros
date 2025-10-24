// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_CUDA_TESTS_GREEN_CONTEXT_WITH_SMID_HPP
#define NVIDIA_GXF_CUDA_TESTS_GREEN_CONTEXT_WITH_SMID_HPP

#include <cuda.h>
#include <cuda_runtime.h>

#include <vector>
#include <set>

#define THREADS_PER_BLOCK 32
#define THREADS_PER_BLOCK_1 (THREADS_PER_BLOCK - 1)

// Structure to hold thread information including SMID
struct SmidThreadInfo {
    int thread_id;           // Global thread index
    int block_id;            // Block index
    int thread_in_block;     // Thread index within block
    uint32_t smid;           // Streaming Multiprocessor ID
    int block_dim_x;         // Block dimension x
    int grid_dim_x;          // Grid dimension x
};

// CUDA kernel function declarations
extern "C" {
    // Main function to test SMID for every thread
    void test_every_thread_smid(int num_blocks, int threads_per_block, cudaStream_t stream, SmidThreadInfo* h_thread_info);

    // Function to analyze SMID results
    int32_t analyze_smid_results(std::vector<SmidThreadInfo*> ptr_thread_info, int threads_per_block, int num_blocks, int num_streams);

    // Function to test SMID with different configurations
    int32_t test_smid_with_different_configurations(cudaStream_t* streams, int num_streams);

    // Function to collect SM IDs from a single kernel launch
    int32_t collect_smid_from_kernel(int num_blocks, int threads_per_block, cudaStream_t stream, std::set<uint32_t>& unique_smids);
}

#endif // NVIDIA_GXF_CUDA_TESTS_GREEN_CONTEXT_WITH_SMID_HPP