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

#ifndef ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_SHAPE_HPP_
#define ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_SHAPE_HPP_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/std/tensor.hpp"
#pragma GCC diagnostic pop

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosTensorShape
{
public:
  NitrosTensorShape(std::initializer_list<int32_t> dimensions)
  : shape_{dimensions} {}

  const nvidia::gxf::Shape & shape() const {return shape_;}

private:
  nvidia::gxf::Shape shape_{};
};


}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_SHAPE_HPP_
