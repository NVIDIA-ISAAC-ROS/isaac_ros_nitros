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

#ifndef ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_BUILDER_HPP_
#define ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_BUILDER_HPP_

#include <cuda_runtime.h>
#include <functional>

#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"
#include "std_msgs/msg/header.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosPointCloudBuilder
{
public:
  NitrosPointCloudBuilder();
  NitrosPointCloudBuilder(const NitrosPointCloudBuilder &) = delete;
  NitrosPointCloudBuilder & operator=(const NitrosPointCloudBuilder &) = delete;
  NitrosPointCloudBuilder(NitrosPointCloudBuilder && other);
  NitrosPointCloudBuilder & operator=(NitrosPointCloudBuilder && other);

  NitrosPointCloudBuilder & WithHeader(const std_msgs::msg::Header & header);

  NitrosPointCloudBuilder & WithPoints(
    const void * points_data, int32_t num_points,
    bool use_color = false);

  NitrosPointCloudBuilder & WithEvent(cudaEvent_t event);

  NitrosPointCloudBuilder & WithReleaseCallback(std::function<void()> release_callback);

  NitrosPointCloud Build();

private:
  void Validate();
  NitrosPointCloud nitros_point_cloud_{};
  const void * points_data_{nullptr};
  int32_t num_points_{0};
  bool use_color_{false};
  std_msgs::msg::Header header_{};
  cudaEvent_t event_{};
  std::function<void()> release_callback_{nullptr};
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
#endif  // ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_BUILDER_HPP_
