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

#ifndef NVIDIA_GXF_GRAPH_UTILS_HPP_
#define NVIDIA_GXF_GRAPH_UTILS_HPP_

#include <string>

namespace nvidia {
namespace gxf {


static gxf_tid_t generate_tid() {
    std::random_device rd;
    std::mt19937_64 generator(rd());
    std::uniform_int_distribution<uint64_t> dis(0, UINT64_MAX);

    // Generate two 64-bit random numbers to create a UUID
    uint64_t random1 = dis(generator);
    uint64_t random2 = dis(generator);

    // Combine the random numbers to create a UUID string
    std::string uuid_str = std::to_string(random1) + std::to_string(random2);
    GXF_LOG_VERBOSE("UUID generated %s", uuid_str.c_str());

    return gxf_tid_t{random1, random2};
}


}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_UTILS_HPP_
