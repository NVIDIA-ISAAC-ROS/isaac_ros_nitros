// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_STD_TIMESTAMP_HPP
#define NVIDIA_GXF_STD_TIMESTAMP_HPP

#include <cstdint>
#include <utility>

#include "common/fixed_vector.hpp"
#include "gxf/core/expected.hpp"

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

enum class TimeDomainID : uint8_t {
  // Specifies various sources of timestamp
  TSC = 0,
  NTP,
  PTP,
  TIME_DOMAIN_COUNT   // Used to determine the maximum no of timestamp sources available
};

static constexpr ssize_t MAX_TIME_DOMAINS = static_cast<ssize_t>(TimeDomainID::TIME_DOMAIN_COUNT);

// Associate timestamp with its source
using MultiSourceTimestamp = FixedVector<std::pair<Timestamp, TimeDomainID>, MAX_TIME_DOMAINS>;

// This function retrieves the timestamp corresponding to the time domain ID
Expected<Timestamp> getTimestamp(MultiSourceTimestamp const& timestamps,
                                 TimeDomainID const& timeDomainId);

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_TIMESTAMP_HPP
