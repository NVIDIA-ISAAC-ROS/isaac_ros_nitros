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

#ifndef ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_VIEW_HPP_
#define ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_VIEW_HPP_

#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/types/nitros_type_view_factory.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NITROS_TYPE_VIEW_FACTORY_BEGIN(NitrosPointCloud)

MARK_PUBLIC_SECTION()
// Public methods
inline uint32_t GetWidth() const {return width_;}
inline uint32_t GetHeight() const {return height_;}
inline uint32_t GetPointCount() const {return point_count_;}
inline bool HasColor() const {return has_color_;}
inline const float * GetPointsData() const {return points_data_;}
inline uint32_t GetPointStep() const {return point_step_;}
inline uint32_t GetRowStep() const {return row_step_;}
inline bool IsBigEndian() const {return is_bigendian_;}
inline bool IsDense() const {return is_dense_;}

MARK_PRIVATE_SECTION()
void GetPointCloudData();
uint32_t width_{0};
uint32_t height_{0};
uint32_t point_count_{0};
bool has_color_{false};
const float * points_data_{nullptr};
uint32_t point_step_{0};
uint32_t row_step_{0};
bool is_bigendian_{false};
bool is_dense_{false};

NITROS_TYPE_VIEW_FACTORY_END(NitrosPointCloud)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_VIEW_HPP_
