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

#ifndef ISAAC_ROS_NITROS_TOPIC_TOOLS__ISAAC_ROS_NITROS_TOPIC_TOOLS_COMMON_HPP_
#define ISAAC_ROS_NITROS_TOPIC_TOOLS__ISAAC_ROS_NITROS_TOPIC_TOOLS_COMMON_HPP_

#include <string>
#include <unordered_map>

namespace
{
constexpr const char kModeMono[] = "mono";
constexpr const char kModeStereo[] = "stereo";
constexpr const char kModeMonoDepth[] = "mono+depth";
}

// Define enum for different modes
enum class CameraDropMode
{
  Mono,
  Stereo,
  MonoDepth
};

// Map enum values to string representations
const std::unordered_map<CameraDropMode, std::string> modeToStringMap = {
  {CameraDropMode::Mono, kModeMono},
  {CameraDropMode::MonoDepth, kModeMonoDepth},
  {CameraDropMode::Stereo, kModeStereo}};

#endif  // ISAAC_ROS_NITROS_TOPIC_TOOLS__ISAAC_ROS_NITROS_TOPIC_TOOLS_COMMON_HPP_
