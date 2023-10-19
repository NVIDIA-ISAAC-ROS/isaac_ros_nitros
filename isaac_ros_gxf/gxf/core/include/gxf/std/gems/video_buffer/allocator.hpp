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
#ifndef NVIDIA_GXF_STD_GEMS_VIDEO_BUFFER_HPP
#define NVIDIA_GXF_STD_GEMS_VIDEO_BUFFER_HPP

#include <array>
#include <vector>
#include "gxf/multimedia/video.hpp"
#include "gxf/std/allocator.hpp"

namespace nvidia {
namespace gxf {
template <gxf::VideoFormat T>
struct NoPaddingColorPlanes {};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_RGB> {
  explicit NoPaddingColorPlanes(uint32_t width) : planes({gxf::ColorPlane("RGB", 3, width * 3)}) {}
  std::array<gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_BGR> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("BGR", 3, width * 3)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_RGBA> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("RGBA", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_BGRA> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("BGRA", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_RGB16> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("RGB", 6, width * 6)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_BGR16> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("BGR", 6, width * 6)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_RGB32> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("RGB", 12, width * 12)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_BGR32> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("BGR", 12, width * 12)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_GRAY> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("gray", 1, width)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_GRAY16> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("gray", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_GRAY32> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("gray", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_GRAY32F> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("gray", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_NV12> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("Y", 1, width),
                nvidia::gxf::ColorPlane("UV", 2, width)}) {}
  std::array<nvidia::gxf::ColorPlane, 2> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_NV12_ER> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("Y", 1, width),
                nvidia::gxf::ColorPlane("UV", 2, width)}) {}
  std::array<nvidia::gxf::ColorPlane, 2> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_NV24> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("Y", 1, width),
                nvidia::gxf::ColorPlane("UV", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 2> planes;
};
template <>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_NV24_ER> {
  explicit NoPaddingColorPlanes(uint32_t width)
      : planes({nvidia::gxf::ColorPlane("Y", 1, width),
                nvidia::gxf::ColorPlane("UV", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 2> planes;
};
template<>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_R8_G8_B8> {
  explicit NoPaddingColorPlanes(size_t width)
    : planes({gxf::ColorPlane("R", 1, width), gxf::ColorPlane("G", 1, width),
    gxf::ColorPlane("B", 1, width)}) {}
  std::array<gxf::ColorPlane, 3> planes;
};
template<>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_R16_G16_B16> {
  explicit NoPaddingColorPlanes(size_t width)
    : planes({gxf::ColorPlane("R", 2, width * 2), gxf::ColorPlane("G", 2, width * 2),
    gxf::ColorPlane("B", 2, width * 2)}) {}
  std::array<gxf::ColorPlane, 3> planes;
};
template<>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_R32_G32_B32> {
  explicit NoPaddingColorPlanes(size_t width)
    : planes(
        {gxf::ColorPlane("R", 4, width * 4), gxf::ColorPlane("G", 4, width * 4),
        gxf::ColorPlane("B", 4, width * 4)}) {}
  std::array<gxf::ColorPlane, 3> planes;
};
template<>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_B8_G8_R8> {
  explicit NoPaddingColorPlanes(size_t width)
    : planes({gxf::ColorPlane("B", 1, width), gxf::ColorPlane("G", 1, width),
    gxf::ColorPlane("R", 1, width)}) {}
  std::array<gxf::ColorPlane, 3> planes;
};
template<>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_B16_G16_R16> {
  explicit NoPaddingColorPlanes(size_t width)
    : planes(
        {gxf::ColorPlane("R", 2, width * 2), gxf::ColorPlane("G", 2, width * 2),
        gxf::ColorPlane("B", 2, width * 2)}) {}
  std::array<gxf::ColorPlane, 3> planes;
};
template<>
struct NoPaddingColorPlanes<gxf::VideoFormat::GXF_VIDEO_FORMAT_B32_G32_R32> {
  explicit NoPaddingColorPlanes(size_t width)
    : planes(
        {gxf::ColorPlane("B", 4, width * 4), gxf::ColorPlane("G", 4, width * 4),
        gxf::ColorPlane("R", 4, width * 4)}) {}
  std::array<gxf::ColorPlane, 3> planes;
};
// This includes the list of video buffer formats that supported for the allocator
constexpr bool IsSupportedVideoFormat(const gxf::VideoFormat format) {
  return format == gxf::VideoFormat::GXF_VIDEO_FORMAT_RGB ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_BGR ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_RGBA ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_BGRA ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_RGB16 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_BGR16 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_RGB32 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_BGR32 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_GRAY ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_GRAY16 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_GRAY32 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_GRAY32F ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_R8_G8_B8 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_B8_G8_R8 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_R16_G16_B16 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_B16_G16_R16 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_R32_G32_B32 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_B32_G32_R32 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_NV12 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_NV12_ER ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_NV24 ||
         format == gxf::VideoFormat::GXF_VIDEO_FORMAT_NV24_ER;
}
template<gxf::VideoFormat T, typename std::enable_if<!IsSupportedVideoFormat(T)>::type* = nullptr>
gxf::Expected<void> AllocateUnpaddedVideoBuffer(
    gxf::Handle<gxf::VideoBuffer> frame, uint32_t width, uint32_t height,
    gxf::MemoryStorageType storage_type,
    gxf::Handle<gxf::Allocator> allocator) {
  GXF_LOG_ERROR("Received unsupported video format!");
  return gxf::Unexpected{GXF_FAILURE};
}
template<gxf::VideoFormat T, typename std::enable_if<IsSupportedVideoFormat(T)>::type* = nullptr>
gxf::Expected<void> AllocateUnpaddedVideoBuffer(
    gxf::Handle<gxf::VideoBuffer> frame, uint32_t width, uint32_t height,
    gxf::MemoryStorageType storage_type, gxf::Handle<gxf::Allocator> allocator) {
  if (width % 2 != 0 || height % 2 != 0) {
    GXF_LOG_ERROR(
        "Error: expected even width and height but received %u width and %u height",
        width, height);
    return gxf::Unexpected{GXF_FAILURE};
  }
  NoPaddingColorPlanes<T> nopadding_planes(width);
  gxf::VideoFormatSize<T> video_format_size;
  auto size = video_format_size.size(width, height, nopadding_planes.planes);
  std::vector<gxf::ColorPlane> color_planes{
      nopadding_planes.planes.begin(), nopadding_planes.planes.end()};
  gxf::VideoBufferInfo buffer_info{width, height, T, color_planes,
      gxf::SurfaceLayout::GXF_SURFACE_LAYOUT_PITCH_LINEAR};
  return frame->resizeCustom(buffer_info, size, storage_type, allocator);
}
template<gxf::VideoFormat T>
gxf::Expected<void> AllocateVideoBuffer(
    gxf::Handle<gxf::VideoBuffer> frame, uint32_t width, uint32_t height,
    gxf::SurfaceLayout layout, gxf::MemoryStorageType storage_type,
    gxf::Handle<gxf::Allocator> allocator) {
    return frame->resize<T>(width, height, layout, storage_type, allocator);
}
}  // namespace gxf
}  // namespace nvidia

#endif
