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

#ifdef NITROS_GXF_COMPAT_MODE

// ============================================================================
// INTERIM: GXF Compatibility Implementation for NitrosImage
// TODO(yuankunz): Remove entire file when all nodes migrated to GXF-free
//
// This file implements GXF<->Buffer conversion for NitrosImage.
// For new GXF-free types: NO compat file needed!
// ============================================================================

#include "isaac_ros_nitros_image_type/nitros_image_gxf_compat.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_details.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/multimedia/video.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NITROS_GXF_COMPAT_FORMATS_BEGIN(NitrosImage)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_rgb8_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_rgba8_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_rgb16_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_bgr8_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_bgra8_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_bgr16_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_mono8_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_mono16_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_nv12_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_nv24_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_32FC1_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_32FC3_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_image_32FC4_t)
NITROS_GXF_COMPAT_FORMATS_END()

// INTERIM: NitrosGxfCompatTraits<NitrosImage> implementation - GXF entity → NitrosImage
NitrosImage NitrosGxfCompatTraits<NitrosImage>::CreateFromGxfEntity(
  gxf_context_t context,
  int64_t eid)
{
  auto entity = nvidia::gxf::Entity::Shared(context, eid);
  if (!entity) {
    throw std::runtime_error("Failed to get GXF entity");
  }

  auto video_buffer = entity->get<nvidia::gxf::VideoBuffer>();
  if (!video_buffer) {
    throw std::runtime_error("Failed to get VideoBuffer from entity");
  }

  auto & vb = video_buffer.value();
  auto info = vb->video_frame_info();

  // Get encoding from GXF format
  auto encoding_it = g_gxf_to_ros_video_format.find(info.color_format);
  if (encoding_it == g_gxf_to_ros_video_format.end()) {
    throw std::runtime_error("Unsupported GXF video format");
  }

  // Get timestamp
  uint32_t timestamp_sec = 0;
  uint32_t timestamp_nsec = 0;
  auto timestamp = entity->get<nvidia::gxf::Timestamp>();
  if (timestamp) {
    uint64_t acqtime = timestamp.value()->acqtime;
    timestamp_sec = acqtime / 1000000000UL;
    timestamp_nsec = acqtime % 1000000000UL;
  }

  // Wrap GXF VideoBuffer memory in NitrosImage (zero-copy)
  NitrosImage img;
  uint8_t * dev_ptr = vb->pointer();
  size_t size = vb->size();
  uint32_t step = info.color_planes[0].stride;

  // Keep GXF entity is alive until theNitrosImage is destroyed.
  // The NITROS image destructor is set to not free the memory
  // and decrement the GXF entity refcount.
  gxf_result_t result = GxfEntityRefCountInc(context, eid);
  if (result != GXF_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("NitrosGxfCompat"),
      "GxfEntityRefCountInc FAILED for eid=%ld: %s", eid, GxfResultStr(result));
    throw std::runtime_error("GxfEntityRefCountInc failed");
  }

  auto deleter = [context, eid](uint8_t * p) {
      (void)p;
      GxfEntityRefCountDec(context, eid);
    };

  [[maybe_unused]] auto write_handle = img.from_external(
    dev_ptr, size, info.width, info.height, step,
    encoding_it->second, nullptr /* avoid recording a write event */, deleter);

  img.timestamp_sec = timestamp_sec;
  img.timestamp_nsec = timestamp_nsec;
  return img;
}

// INTERIM: Template helper to create GXF VideoBuffer (GXF owns memory)
template<VideoFormat T>
static int64_t CreateGxfVideoBuffer(
  const NitrosImage & img,
  nvidia::gxf::Entity & entity)
{
  auto video_buffer = entity.add<nvidia::gxf::VideoBuffer>();
  if (!video_buffer) {
    throw std::runtime_error("Failed to add VideoBuffer to entity");
  }

  auto buffer = NitrosBufferAccessor<NitrosImage>::get_buffer(img);
  uint8_t * dev_ptr = const_cast<uint8_t *>(buffer->get_data());

  NoPaddingColorPlanes<T> planes(img.width);
  nvidia::gxf::VideoFormatSize<T> format_size;
  uint64_t size = format_size.size(img.width, img.height, planes.planes);

  std::vector<nvidia::gxf::ColorPlane> color_planes{planes.planes.begin(), planes.planes.end()};
  nvidia::gxf::VideoBufferInfo buffer_info{
    img.width, img.height, T, color_planes,
    nvidia::gxf::SurfaceLayout::GXF_SURFACE_LAYOUT_PITCH_LINEAR
  };

  // No-Op on GXF destructor - NitrosImage manages memory
  // Pass in the nitros buffer shared_ptr to ensure the nitros buffer is alive
  auto buffer_ptr = buffer;
  video_buffer.value()->wrapMemory(
    buffer_info, size, nvidia::gxf::MemoryStorageType::kDevice, dev_ptr,
    [buffer_ptr](void * p) -> nvidia::gxf::Expected<void> {
      (void)p;
      return nvidia::gxf::Success;
    });

  return entity.eid();
}

// INTERIM: NitrosGxfCompatTraits<NitrosImage> implementation - NitrosImage → GXF entity
int64_t NitrosGxfCompatTraits<NitrosImage>::CreateGxfEntity(
  gxf_context_t context,
  const NitrosImage & img)
{
  auto entity = nvidia::gxf::Entity::New(context);
  if (!entity) {
    throw std::runtime_error("Failed to create GXF entity");
  }

  // Get GXF format from encoding
  auto format_it = g_ros_to_gxf_video_format.find(img.encoding);
  if (format_it == g_ros_to_gxf_video_format.end()) {
    throw std::runtime_error("Unsupported encoding for GXF: " + img.encoding);
  }

  int64_t eid = -1;
  auto color_fmt = format_it->second;

  // Call template function based on format
  switch (color_fmt) {
    case VideoFormat::GXF_VIDEO_FORMAT_RGB:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_RGB>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_RGBA:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_RGBA>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_RGB16:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_RGB16>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_BGR:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_BGR>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_BGRA:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_BGRA>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_BGR16:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_BGR16>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_GRAY:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_GRAY>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_GRAY16:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_GRAY16>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_GRAY32:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_GRAY32>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_NV12_ER:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_NV12_ER>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_NV24_ER:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_NV24_ER>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_RGB32:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_RGB32>(img, entity.value());
      break;
    case VideoFormat::GXF_VIDEO_FORMAT_RGBD32:
      eid = CreateGxfVideoBuffer<VideoFormat::GXF_VIDEO_FORMAT_RGBD32>(img, entity.value());
      break;
    default:
      throw std::runtime_error("GXF compat Unsupported encoding: " + img.encoding);
  }

  // Add timestamp
  auto timestamp = entity.value().add<nvidia::gxf::Timestamp>();
  if (timestamp) {
    uint64_t acqtime = static_cast<uint64_t>(img.timestamp_sec) * 1000000000UL +
      static_cast<uint64_t>(img.timestamp_nsec);
    timestamp.value()->acqtime = acqtime;
  }

  // Increment refcount (will be decremented when entity destroyed)
  GxfEntityRefCountInc(context, eid);

  return eid;
}

NITROS_GXF_COMPAT_REGISTER_CONVERTER(NitrosImage)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NITROS_GXF_COMPAT_MODE
