// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <string>

#include "gxf/core/gxf.h"
#include "gxf/std/extension_factory_helper.hpp"
#include "detection3_d_array_message/detection3_d_array_message.hpp"

extern "C" {

GXF_EXT_FACTORY_BEGIN()

GXF_EXT_FACTORY_SET_INFO(
  0x5560bbc051a511ee, 0x9be8f7b68805842c, "Detection3DTestMessageExtension",
  "Detection3D Message GXF extension",
  "NVIDIA", "1.0.0", "LICENSE");

GXF_EXT_FACTORY_ADD_0(
  0x65d4476051a511ee, 0xb3f727d2ed955144,
  nvidia::isaac::ObjectHypothesis,
  "List of scores and class ids");

GXF_EXT_FACTORY_ADD_0(
  0x782823c851dc11ee, 0xa5dc87b46496e7b8,
  nvidia::isaac::Vector3f,
  "3 Dimensional Vector");


GXF_EXT_FACTORY_END()

}  // extern "C"
