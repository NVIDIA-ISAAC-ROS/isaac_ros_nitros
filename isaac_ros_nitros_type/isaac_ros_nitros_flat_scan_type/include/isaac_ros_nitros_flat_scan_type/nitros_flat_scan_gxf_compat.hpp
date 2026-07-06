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

#ifndef ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_GXF_COMPAT_HPP_
#define ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_GXF_COMPAT_HPP_

#ifdef NITROS_GXF_COMPAT_MODE

#include "isaac_ros_nitros/types/nitros_gxf_compat_traits.hpp"
#include "gxf/core/gxf.h"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosFlatScan;

template<>
struct NitrosGxfCompatTraits<NitrosFlatScan>
{
  static int64_t CreateGxfEntity(
    gxf_context_t context,
    const NitrosFlatScan & msg);

  static NitrosFlatScan CreateFromGxfEntity(
    gxf_context_t context,
    int64_t eid);
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NITROS_GXF_COMPAT_MODE

#endif  // ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_GXF_COMPAT_HPP_
