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
#ifndef DETECTNET__DETECTION2_D_ARRAY_MESSAGE_HPP_
#define DETECTNET__DETECTION2_D_ARRAY_MESSAGE_HPP_

#include <vector>

#include "gxf/core/entity.hpp"
#include "gxf/std/timestamp.hpp"
#include "detection2_d.hpp"

namespace nvidia
{
namespace isaac_ros
{

// This struct helps parse the data coming from a gxf message. The message entity consists of a
// `Detection2D` component and a `Timestamp` component.
// Note that we do not add the raw type but rather a `Handle` to the type.
struct Detection2DParts
{
  gxf::Entity message;
  gxf::Handle<std::vector<Detection2D>> detection2_d_array;
  gxf::Handle<gxf::Timestamp> timestamp;
};

// This function creates a new entity and adds the (default-initialized) components from the parts
// struct. It returns the created parts struct s.t. we can then use this to modify the data
// contained in the entity.
gxf::Expected<Detection2DParts> CreateDetection2DList(gxf_context_t context);

// This function allows to parse an entity and returns the parts struct such that we have easy
// access to the components.
gxf::Expected<Detection2DParts> GetDetection2DList(gxf::Entity message);

}  // namespace isaac_ros
}  // namespace nvidia

#endif  // DETECTNET__DETECTION2_D_ARRAY_MESSAGE_HPP_
