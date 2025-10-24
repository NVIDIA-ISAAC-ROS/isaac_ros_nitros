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

#ifndef ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_BUILDER_HPP_
#define ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_BUILDER_HPP_

#include <functional>
#include <string>
#include <cstdint>

#include "isaac_ros_nitros_disparity_image_type/nitros_disparity_image.hpp"
#include "std_msgs/msg/header.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosDisparityImageBuilder
{
public:
  NitrosDisparityImageBuilder();
  NitrosDisparityImageBuilder(const NitrosDisparityImageBuilder &) = delete;
  NitrosDisparityImageBuilder & operator=(const NitrosDisparityImageBuilder &) = delete;
  NitrosDisparityImageBuilder(NitrosDisparityImageBuilder && other);
  NitrosDisparityImageBuilder & operator=(NitrosDisparityImageBuilder && other);

  NitrosDisparityImageBuilder & WithHeader(std_msgs::msg::Header header);
  NitrosDisparityImageBuilder & WithDimensions(uint32_t height, uint32_t width);
  NitrosDisparityImageBuilder & WithGpuData(void * data);
  NitrosDisparityImageBuilder & WithReleaseCallback(std::function<void()> release_callback);
  NitrosDisparityImageBuilder & WithDisparityParameters(
    float f, float t, float min_disparity, float max_disparity);

  NitrosDisparityImage Build();

private:
  void Validate();

  NitrosDisparityImage nitros_disparity_image_;
  uint32_t height_{0};
  uint32_t width_{0};
  void * data_{nullptr};
  std::function<void()> release_callback_{nullptr};
  float f_{0.0f};
  float t_{0.0f};
  float min_disparity_{0.0f};
  float max_disparity_{0.0f};
  std_msgs::msg::Header header_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_BUILDER_HPP_
