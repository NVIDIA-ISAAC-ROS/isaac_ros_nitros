// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_BUILDER_HPP_
#define ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_BUILDER_HPP_

#include <string>

#include "isaac_ros_nitros_image_type/nitros_image.hpp"

#include "std_msgs/msg/header.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosImageBuilder
{
public:
  NitrosImageBuilder();
  NitrosImageBuilder(const NitrosImageBuilder &) = delete;
  NitrosImageBuilder & operator=(const NitrosImageBuilder &) = delete;
  NitrosImageBuilder(NitrosImageBuilder && other);
  NitrosImageBuilder & operator=(NitrosImageBuilder && other);


  NitrosImageBuilder & WithHeader(std_msgs::msg::Header header);

  // This is of type listed in image_encodings.hpp
  NitrosImageBuilder & WithEncoding(std::string encoding);

  NitrosImageBuilder & WithDimensions(uint32_t height, uint32_t width);

  // This has to be GPU buffer
  NitrosImageBuilder & WithGpuData(void * data);

  NitrosImage Build();

private:
  void Validate();
  NitrosImage nitros_image_{};
  std::string encoding_{""};
  uint32_t height_{0};
  uint32_t width_{0};
  void * data_{nullptr};
  cudaEvent_t event_{};
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_BUILDER_HPP_
