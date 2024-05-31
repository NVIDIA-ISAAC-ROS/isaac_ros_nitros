// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_GXF_STD_GEMS_UTILS_TIME_HPP_
#define NVIDIA_GXF_GXF_STD_GEMS_UTILS_TIME_HPP_

#include <chrono>
#include <string>
#include "gxf/core/expected.hpp"

namespace nvidia {
namespace gxf {

/// @brief Converts time in seconds to a timestamp in nanoseconds
int64_t TimeToTimestamp(double time);

/// @brief Converts a timestamp Nanoseconds to seconds
double TimestampToTime(int64_t timestamp);

/// @brief Parses given text to return the desired period in nanoseconds.
///
/// @param text Text containing number and time-units, to be parsed for desired period
/// @param cid cid of component for which text is being parsed
/// @return Period in nanoseconds if successful, or otherwise one of the GXF error codes.
Expected<int64_t> ParseRecessPeriodString(std::string text, const gxf_uid_t& cid);

inline uint64_t getCurrentTimeUs() {
  using std::chrono::duration_cast;
  using std::chrono::system_clock;
  return duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch()).count();
}

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GXF_STD_GEMS_UTILS_TIME_HPP_
