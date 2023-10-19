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

#include "extensions/fiducials/gems/fiducial_info.hpp"
#include "gxf/std/extension_factory_helper.hpp"

GXF_EXT_FACTORY_BEGIN()

GXF_EXT_FACTORY_SET_INFO(
  0xd8d7816ec0485ad4, 0xff795a414bd445ca, "FiducialsTestMessageExtension",
  "Test extension for fiducials messages",
  "NVIDIA", "1.0.0", "LICENSE");

GXF_EXT_FACTORY_ADD_0(
  0xe91d3fa6a42b85ff, 0x966f4e80c607ca9e,
  nvidia::isaac::FiducialInfo,
  "Holds fiducial meta information.");

GXF_EXT_FACTORY_END()
