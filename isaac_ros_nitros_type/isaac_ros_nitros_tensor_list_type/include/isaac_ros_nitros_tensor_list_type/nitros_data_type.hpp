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

#ifndef ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_DATA_TYPE_HPP_
#define ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_DATA_TYPE_HPP_

#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

enum class NitrosDataType
{
  kCustom,
  kInt8,
  kUnsigned8,
  kInt16,
  kUnsigned16,
  kInt32,
  kUnsigned32,
  kInt64,
  kUnsigned64,
  kFloat32,
  kFloat64,
  kComplex64,
  kComplex128,
};

nvidia::gxf::PrimitiveType GetPrimitiveType(NitrosDataType nitros_data_type);


}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_DATA_TYPE_HPP_
