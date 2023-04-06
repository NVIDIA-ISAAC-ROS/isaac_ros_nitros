/*
 * SPDX-FileCopyrightText: Copyright (c) 202-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_STD_TIMESTAMP_HPP
#define NVIDIA_GXF_STD_TIMESTAMP_HPP

#include <cstdint>

namespace nvidia {
namespace gxf {

// Contains timing information for the data in a message. All times are relative to the global GXF
// clock and in nanoseconds.
struct Timestamp {
  // The timestamp in nanoseconds at which the message was published into the system.
  int64_t pubtime;
  // The timestamp in nanoseconds at the message was acquired. This usually refers to the timestamp
  // of the original sensor data which created the message.
  int64_t acqtime;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_TIMESTAMP_HPP
