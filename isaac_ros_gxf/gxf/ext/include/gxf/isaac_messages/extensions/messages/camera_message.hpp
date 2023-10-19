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
#pragma once

#include "gxf/multimedia/camera.hpp"
#include "gxf/multimedia/video.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

// Message structure for camera messages
struct CameraMessageParts {
  // Message entity
  gxf::Entity entity;
  // Camera frame
  gxf::Handle<gxf::VideoBuffer> frame;
  // Camera intrinsics
  gxf::Handle<gxf::CameraModel> intrinsics;
  // Camera extrinsics
  gxf::Handle<gxf::Pose3D> extrinsics;
  // Frame sequence number
  gxf::Handle<int64_t> sequence_number;
  // Frame acquisition timestamp
  gxf::Handle<gxf::Timestamp> timestamp;
};

// Creates a camera message in the image format specified by the template parameter `Format`.
// Frame memory is allocated based on `storage_type` and will have dimensions specified by
// `width` and `height`. The memory surface layout of the frame is specified via `layout`.
// Optionally, the `padded` boolean parameter can be passed to determine whether the camera message
// should be padded or not.
template <gxf::VideoFormat Format>
gxf::Expected<CameraMessageParts> CreateCameraMessage(gxf_context_t context,
                                                      uint32_t width,
                                                      uint32_t height,
                                                      gxf::SurfaceLayout layout,
                                                      gxf::MemoryStorageType storage_type,
                                                      gxf::Handle<gxf::Allocator> allocator,
                                                      bool padded = true);

// Creates a camera message in the image format specified by the arguments `buffer_info` and `size`.
// Frame memory is allocated based on `storage_type`.
gxf::Expected<CameraMessageParts> CreateCameraMessage(gxf_context_t context,
                                                      gxf::VideoBufferInfo buffer_info,
                                                      uint64_t size,
                                                      gxf::MemoryStorageType storage_type,
                                                      gxf::Handle<gxf::Allocator> allocator);

// Parses a camera message from the given entity
gxf::Expected<CameraMessageParts> GetCameraMessage(const gxf::Entity message);

// Check if an entity contains a camera message
gxf::Expected<bool> IsCameraMessage(const gxf::Entity entity);

}  // namespace isaac
}  // namespace nvidia
