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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_tensor_list_type/nitros_data_type.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

namespace
{
using PrimitiveType = nvidia::gxf::PrimitiveType;
std::unordered_map<NitrosDataType, PrimitiveType> data_type_map{
  {NitrosDataType::kCustom, PrimitiveType::kCustom},
  {NitrosDataType::kInt8, PrimitiveType::kInt8},
  {NitrosDataType::kUnsigned8, PrimitiveType::kUnsigned8},
  {NitrosDataType::kInt16, PrimitiveType::kInt16},
  {NitrosDataType::kUnsigned16, PrimitiveType::kUnsigned16},
  {NitrosDataType::kInt32, PrimitiveType::kInt32},
  {NitrosDataType::kUnsigned32, PrimitiveType::kUnsigned32},
  {NitrosDataType::kInt64, PrimitiveType::kInt64},
  {NitrosDataType::kUnsigned64, PrimitiveType::kUnsigned64},
  {NitrosDataType::kFloat32, PrimitiveType::kFloat32},
  {NitrosDataType::kFloat64, PrimitiveType::kFloat64},
  {NitrosDataType::kComplex64, PrimitiveType::kComplex64},
  {NitrosDataType::kComplex128, PrimitiveType::kComplex128},
};
}  // namespace

nvidia::gxf::PrimitiveType GetPrimitiveType(NitrosDataType nitros_data_type)
{
  return data_type_map.at(nitros_data_type);
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
