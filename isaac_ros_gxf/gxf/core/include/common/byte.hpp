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
#ifndef NVIDIA_COMMON_BYTE_HPP_
#define NVIDIA_COMMON_BYTE_HPP_

# if __cplusplus < 201703L
namespace std {
using byte = unsigned char;
}  // namespace std
using std::byte;
# else
namespace nvidia {
    using byte = unsigned char;
}
using nvidia::byte;

#endif

#endif  // NVIDIA_COMMON_BYTE_HPP_
