/*
Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
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

}  // namespace isaac
}  // namespace nvidia
