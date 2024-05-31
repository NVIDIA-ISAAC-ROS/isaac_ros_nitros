// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "extensions/utils/disparity_to_depth.hpp"
#include "extensions/utils/image_loader.hpp"
#include "extensions/utils/udp_receiver.hpp"
#include "gxf/std/extension_factory_helper.hpp"

GXF_EXT_FACTORY_BEGIN()

GXF_EXT_FACTORY_SET_INFO(0x157ff311835a4333, 0x1f3c65ceeb5847b4, "NvIsaacUtilsExtension",
                         "Extension containing miscellaneous utility components", "Isaac SDK",
                         "2.0.0", "LICENSE");

GXF_EXT_FACTORY_ADD(0x0626d66ce3ae11ed, 0x800e63ef7b59e300,
                    nvidia::isaac::DisparityToDepth, nvidia::gxf::Codelet,
                    "Converts disparity maps to depth maps");

GXF_EXT_FACTORY_ADD(0x666c1d1dae4e4e81, 0xd1e4c3bef6f34a68,
                    nvidia::isaac::ImageLoader, nvidia::gxf::Component,
                    "Loads image data into a Tensor");

GXF_EXT_FACTORY_ADD(0xcb7d1d801f5daf96, 0x985e15059bd5a332,
                    nvidia::isaac::UdpReceiver, nvidia::gxf::Codelet,
                    "Receives packets from a UDP socket and publishes them as tensors");

GXF_EXT_FACTORY_END()
