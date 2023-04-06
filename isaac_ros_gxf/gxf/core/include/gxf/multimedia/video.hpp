/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NVIDIA_GXF_MULTIMEDIA_VIDEO_HPP_
#define NVIDIA_GXF_MULTIMEDIA_VIDEO_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "gxf/core/expected.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/memory_buffer.hpp"
#include "gxf/std/tensor.hpp"

namespace nvidia {
namespace gxf {

#ifdef __aarch64__
static constexpr const uint16_t kGxfAlignValue = 256;
#else
static constexpr const uint16_t kGxfAlignValue = 256;
#endif

// Align frame stride to kGxfAlignValue
static constexpr uint32_t StrideAlign(uint32_t stride) {
  return (stride % kGxfAlignValue == 0) ? stride : ((stride / kGxfAlignValue + 1) * kGxfAlignValue);
}

// Round up to the closest even number
static constexpr uint32_t AlignToEvenDimension(uint32_t value) {
  return value + (value & 1);
}

// Supported raw video media types
enum class VideoFormat : std::int64_t {
  GXF_VIDEO_FORMAT_CUSTOM = 0,
  GXF_VIDEO_FORMAT_YUV420,         // BT.601 multi planar 4:2:0 YUV
  GXF_VIDEO_FORMAT_YUV420_ER,      // BT.601 multi planar 4:2:0 YUV ER
  GXF_VIDEO_FORMAT_YUV420_709,     // BT.709 multi planar 4:2:0 YUV
  GXF_VIDEO_FORMAT_YUV420_709_ER,  // BT.709 multi planar 4:2:0 YUV
  GXF_VIDEO_FORMAT_NV12,           // BT.601 multi planar 4:2:0 YUV with interleaved UV
  GXF_VIDEO_FORMAT_NV12_ER,        // BT.601 multi planar 4:2:0 YUV ER with interleaved UV
  GXF_VIDEO_FORMAT_NV12_709,       // BT.709 multi planar 4:2:0 YUV with interleaved UV
  GXF_VIDEO_FORMAT_NV12_709_ER,    // BT.709 multi planar 4:2:0 YUV ER with interleaved UV
  GXF_VIDEO_FORMAT_RGBA,           // RGBA-8-8-8-8 single plane
  GXF_VIDEO_FORMAT_BGRA,           // BGRA-8-8-8-8 single plane
  GXF_VIDEO_FORMAT_ARGB,           // ARGB-8-8-8-8 single plane
  GXF_VIDEO_FORMAT_ABGR,           // ABGR-8-8-8-8 single plane
  GXF_VIDEO_FORMAT_RGBX,           // RGBX-8-8-8-8 single plane
  GXF_VIDEO_FORMAT_BGRX,           // BGRX-8-8-8-8 single plane
  GXF_VIDEO_FORMAT_XRGB,           // XRGB-8-8-8-8 single plane
  GXF_VIDEO_FORMAT_XBGR,           // XBGR-8-8-8-8 single plane
  GXF_VIDEO_FORMAT_RGB,            // RGB-8-8-8 single plane
  GXF_VIDEO_FORMAT_BGR,            // BGR-8-8-8 single plane
  GXF_VIDEO_FORMAT_R8_G8_B8,       // RGB - unsigned 8 bit multiplanar
  GXF_VIDEO_FORMAT_B8_G8_R8,       // BGR - unsigned 8 bit multiplanar
  GXF_VIDEO_FORMAT_GRAY,           // 8 bit GRAY scale single plane
  GXF_VIDEO_FORMAT_GRAY16,         // 16 bit GRAY scale single plane
  GXF_VIDEO_FORMAT_GRAY32,         // 32 bit GRAY scale single plane
  GXF_VIDEO_FORMAT_GRAY32F,        // float 32 bit GRAY scale single plane
  GXF_VIDEO_FORMAT_RGB16,          // RGB-16-16-16 single plane
  GXF_VIDEO_FORMAT_BGR16,          // BGR-16-16-16 single plane
  GXF_VIDEO_FORMAT_RGB32,          // RGB-32-32-32 single plane
  GXF_VIDEO_FORMAT_BGR32,          // BGR-32-32-32 single plane
  GXF_VIDEO_FORMAT_R16_G16_B16,    // RGB - signed 16 bit multiplanar
  GXF_VIDEO_FORMAT_B16_G16_R16,    // BGR - signed 16 bit multiplanar
  GXF_VIDEO_FORMAT_R32_G32_B32,    // RGB - signed 32 bit multiplanar
  GXF_VIDEO_FORMAT_B32_G32_R32,    // BGR - signed 32 bit multiplanar
  GXF_VIDEO_FORMAT_NV24,           // multi planar 4:4:4 YUV with interleaved UV
  GXF_VIDEO_FORMAT_NV24_ER,        // multi planar 4:4:4 YUV ER with interleaved UV
};

// Supported surface memory types
enum class SurfaceLayout : std::int32_t {
  GXF_SURFACE_LAYOUT_INVALID = 0,
  GXF_SURFACE_LAYOUT_PITCH_LINEAR,  // pitch linear surface memory
  GXF_SURFACE_LAYOUT_BLOCK_LINEAR,  // block linear surface memory
};

template <VideoFormat>
struct VideoTypeTraits;

#define GXF_VIDEO_TYPE_TRAITS(ENUM)                               \
  template <>                                                     \
  struct VideoTypeTraits<VideoFormat::ENUM> {                     \
    static constexpr const char* name = #ENUM;                    \
    static constexpr const VideoFormat value = VideoFormat::ENUM; \
  };

GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_CUSTOM);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_YUV420);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_YUV420_ER);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_YUV420_709);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_YUV420_709_ER);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_NV12);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_NV12_ER);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_NV12_709);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_NV12_709_ER);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_RGBA);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_BGRA);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_ARGB);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_ABGR);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_RGBX);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_BGRX);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_XRGB);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_XBGR);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_RGB);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_BGR);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_R8_G8_B8);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_B8_G8_R8);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_GRAY);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_GRAY16);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_GRAY32);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_GRAY32F);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_RGB16);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_BGR16);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_RGB32);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_BGR32);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_R16_G16_B16);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_B16_G16_R16);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_R32_G32_B32);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_B32_G32_R32);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_NV24);
GXF_VIDEO_TYPE_TRAITS(GXF_VIDEO_FORMAT_NV24_ER);

// Struct to hold the information regarding a single color plane
struct ColorPlane {
  std::string color_space;
  uint8_t bytes_per_pixel;
  int32_t stride;
  uint32_t offset = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint64_t size = 0;

  ColorPlane(const char* c_space, uint8_t c_depth, int32_t c_stride = -1)
      : color_space(c_space), bytes_per_pixel(c_depth), stride(c_stride) {}
};

// Template to compute the memory size of an image
// for different color formats
template <VideoFormat T, typename Enabler = void>
struct VideoFormatSize {
  uint64_t size(uint32_t width, uint32_t height) { return 0; }

  std::vector<ColorPlane> getDefaultColorPlanes(uint32_t width, uint32_t height) { return {}; }
};

//  Specifies YUV420 multi-planar variants
template <VideoFormat T>
struct VideoFormatSize<T, std::enable_if_t<T == VideoFormat::GXF_VIDEO_FORMAT_YUV420 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_YUV420_ER ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_YUV420_709 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_YUV420_709_ER>> {
  std::array<ColorPlane, 3> default_yuv{ColorPlane("Y", 1), ColorPlane("U", 1), ColorPlane("V", 1)};

  uint64_t fillColorPlanes(uint32_t width, uint32_t height,
                           std::array<ColorPlane, 3>& color_planes) {
    uint32_t widthEven = AlignToEvenDimension(width);
    uint32_t heightEven = AlignToEvenDimension(height);
    color_planes[0].width = widthEven;
    color_planes[1].width = widthEven / 2;
    color_planes[2].width = widthEven / 2;
    color_planes[0].height = heightEven;
    color_planes[1].height = heightEven / 2;
    color_planes[2].height = heightEven / 2;

    uint64_t size = 0;
    for (size_t i = 0; i < color_planes.size(); ++i) {
      if (i == 0) {
        color_planes[i].stride =
            color_planes[i].stride == -1
                ? StrideAlign(color_planes[i].width * color_planes[i].bytes_per_pixel)
                : color_planes[i].stride;
      } else {
        color_planes[i].stride =
            color_planes[i].stride == -1 ? color_planes[0].stride / 2 : color_planes[i].stride;
      }
      color_planes[i].size = color_planes[i].stride * color_planes[i].height;
      color_planes[i].offset = size;
      size += color_planes[i].size;
    }
    return size;
  }

  std::vector<ColorPlane> getDefaultColorPlanes(uint32_t width, uint32_t height) {
    auto yuv = default_yuv;
    fillColorPlanes(width, height, yuv);
    std::vector<ColorPlane> result(yuv.begin(), yuv.end());
    return result;
  }

  uint64_t size(uint32_t width, uint32_t height, std::array<ColorPlane, 3>& color_planes) {
    return fillColorPlanes(width, height, color_planes);
  }

  uint64_t size(uint32_t width, uint32_t height) {
    return fillColorPlanes(width, height, default_yuv);
  }
};

// Specifies YUV444 multi-planar variants
template <VideoFormat T>
struct VideoFormatSize<T, std::enable_if_t<T == VideoFormat::GXF_VIDEO_FORMAT_NV24 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_NV24_ER>> {
  std::array<ColorPlane, 2> default_yuv{ColorPlane("Y", 1), ColorPlane("UV", 2)};

  uint64_t fillColorPlanes(uint32_t width, uint32_t height,
                           std::array<ColorPlane, 2>& color_planes) {
    uint32_t widthEven = AlignToEvenDimension(width);
    uint32_t heightEven = AlignToEvenDimension(height);

    uint64_t size = 0;
    for (size_t i = 0; i < color_planes.size(); ++i) {
      color_planes[i].width = widthEven;
      color_planes[i].height = heightEven;
      color_planes[i].stride =
          color_planes[i].stride == -1
              ? StrideAlign(color_planes[i].width * color_planes[i].bytes_per_pixel)
              : color_planes[i].stride;
      color_planes[i].size = color_planes[i].stride * color_planes[i].height;
      color_planes[i].offset = size;
      size += color_planes[i].size;
    }
    return size;
  }

  std::vector<ColorPlane> getDefaultColorPlanes(uint32_t width, uint32_t height) {
    auto yuv = default_yuv;
    fillColorPlanes(width, height, yuv);
    std::vector<ColorPlane> result(yuv.begin(), yuv.end());
    return result;
  }

  uint64_t size(uint32_t width, uint32_t height, std::array<ColorPlane, 2>& color_planes) {
    return fillColorPlanes(width, height, color_planes);
  }

  uint64_t size(uint32_t width, uint32_t height) {
    return fillColorPlanes(width, height, default_yuv);
  }
};

// Specifies NV12 Y/CbCr 4:2:0 multi-planar variants
template <VideoFormat T>
struct VideoFormatSize<T, std::enable_if_t<T == VideoFormat::GXF_VIDEO_FORMAT_NV12 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_NV12_ER ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_NV12_709 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_NV12_709_ER>> {
  std::array<ColorPlane, 2> default_yuv{ColorPlane("Y", 1), ColorPlane("UV", 2)};

  uint64_t fillColorPlanes(uint32_t width, uint32_t height,
                           std::array<ColorPlane, 2>& color_planes) {
    uint32_t widthEven = AlignToEvenDimension(width);
    uint32_t heightEven = AlignToEvenDimension(height);
    color_planes[0].width = widthEven;
    color_planes[1].width = widthEven / 2;
    color_planes[0].height = heightEven;
    color_planes[1].height = heightEven / 2;

    uint64_t size = 0;
    for (size_t i = 0; i < color_planes.size(); ++i) {
      color_planes[i].stride =
          color_planes[i].stride == -1
              ? StrideAlign(color_planes[i].width * color_planes[i].bytes_per_pixel)
              : color_planes[i].stride;
      color_planes[i].size = color_planes[i].stride * color_planes[i].height;
      color_planes[i].offset = size;
      size += color_planes[i].size;
    }
    return size;
  }

  std::vector<ColorPlane> getDefaultColorPlanes(uint32_t width, uint32_t height) {
    auto yuv = default_yuv;
    fillColorPlanes(width, height, yuv);
    std::vector<ColorPlane> result(yuv.begin(), yuv.end());
    return result;
  }

  uint64_t size(uint32_t width, uint32_t height, std::array<ColorPlane, 2>& color_planes) {
    return fillColorPlanes(width, height, color_planes);
  }

  uint64_t size(uint32_t width, uint32_t height) {
    return fillColorPlanes(width, height, default_yuv);
  }
};

// Specifies 8-8-8-8 single plane RGBX/XRGB variants
template <VideoFormat T>
struct VideoFormatSize<
    T, std::enable_if_t<
           T == VideoFormat::GXF_VIDEO_FORMAT_RGBA || T == VideoFormat::GXF_VIDEO_FORMAT_BGRA ||
           T == VideoFormat::GXF_VIDEO_FORMAT_ARGB || T == VideoFormat::GXF_VIDEO_FORMAT_ABGR ||
           T == VideoFormat::GXF_VIDEO_FORMAT_RGBX || T == VideoFormat::GXF_VIDEO_FORMAT_BGRX ||
           T == VideoFormat::GXF_VIDEO_FORMAT_XRGB || T == VideoFormat::GXF_VIDEO_FORMAT_XBGR>> {
  std::array<ColorPlane, 1> defaultRGBA() {
    if (T == VideoFormat::GXF_VIDEO_FORMAT_RGBA) {
      return {ColorPlane("RGBA", 4)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_BGRA) {
      return {ColorPlane("BRGA", 4)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_ARGB) {
      return {ColorPlane("ARGB", 4)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_ABGR) {
      return {ColorPlane("ABGR", 4)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_RGBX) {
      return {ColorPlane("RGBX", 4)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_BGRX) {
      return {ColorPlane("BRGX", 4)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_XRGB) {
      return {ColorPlane("XRGB", 4)};
    } else {  // T == VideoFormat::GXF_VIDEO_FORMAT_XBGR
      return {ColorPlane("XBGR", 4)};
    }
  }

  uint64_t fillColorPlanes(uint32_t width, uint32_t height,
                           std::array<ColorPlane, 1>& color_plane) {
    uint32_t widthEven = AlignToEvenDimension(width);
    uint32_t heightEven = AlignToEvenDimension(height);
    color_plane[0].width = widthEven;
    color_plane[0].height = heightEven;
    color_plane[0].stride = color_plane[0].stride == -1
                                ? StrideAlign(color_plane[0].width * color_plane[0].bytes_per_pixel)
                                : color_plane[0].stride;
    color_plane[0].size = color_plane[0].stride * color_plane[0].height;
    color_plane[0].offset = 0;
    return color_plane[0].size;
  }

  std::vector<ColorPlane> getDefaultColorPlanes(uint32_t width, uint32_t height) {
    auto rgba = defaultRGBA();
    fillColorPlanes(width, height, rgba);
    std::vector<ColorPlane> result(rgba.begin(), rgba.end());
    return result;
  }

  uint64_t size(uint32_t width, uint32_t height, std::array<ColorPlane, 1>& color_planes) {
    return fillColorPlanes(width, height, color_planes);
  }

  uint64_t size(uint32_t width, uint32_t height) {
    auto rgba = defaultRGBA();
    return fillColorPlanes(width, height, rgba);
  }
};

// Specifies x-x-x(8/16/32) bit single plane RGB/BGR variants
template <VideoFormat T>
struct VideoFormatSize<
    T, std::enable_if_t<
           T == VideoFormat::GXF_VIDEO_FORMAT_RGB || T == VideoFormat::GXF_VIDEO_FORMAT_BGR ||
           T == VideoFormat::GXF_VIDEO_FORMAT_RGB16 || T == VideoFormat::GXF_VIDEO_FORMAT_BGR16 ||
           T == VideoFormat::GXF_VIDEO_FORMAT_RGB32 || T == VideoFormat::GXF_VIDEO_FORMAT_BGR32>> {
  std::array<ColorPlane, 1> defaultRGB() {
    if (T == VideoFormat::GXF_VIDEO_FORMAT_RGB) {
      return {ColorPlane("RGB", 3)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_BGR) {
      return {ColorPlane("BGR", 3)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_RGB16) {
      return {ColorPlane("RGB", 3 * 2)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_BGR16) {
      return {ColorPlane("BGR", 3 * 2)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_RGB32) {
      return {ColorPlane("RGB", 3 * 4)};
    } else {  // T == VideoFormat::GXF_VIDEO_FORMAT_BGR32
      return {ColorPlane("BGR", 3 * 4)};
    }
  }

  uint64_t fillColorPlanes(uint32_t width, uint32_t height,
                           std::array<ColorPlane, 1>& color_plane) {
    uint32_t widthEven = AlignToEvenDimension(width);
    uint32_t heightEven = AlignToEvenDimension(height);
    color_plane[0].width = widthEven;
    color_plane[0].height = heightEven;
    color_plane[0].stride = color_plane[0].stride == -1
                                ? StrideAlign(color_plane[0].width * color_plane[0].bytes_per_pixel)
                                : color_plane[0].stride;
    color_plane[0].size = color_plane[0].stride * color_plane[0].height;
    color_plane[0].offset = 0;
    return color_plane[0].size;
  }

  std::vector<ColorPlane> getDefaultColorPlanes(uint32_t width, uint32_t height) {
    auto rgb = defaultRGB();
    fillColorPlanes(width, height, rgb);
    std::vector<ColorPlane> result(rgb.begin(), rgb.end());
    return result;
  }

  uint64_t size(uint32_t width, uint32_t height, std::array<ColorPlane, 1>& color_planes) {
    return fillColorPlanes(width, height, color_planes);
  }

  uint64_t size(uint32_t width, uint32_t height) {
    auto rgb = defaultRGB();
    return fillColorPlanes(width, height, rgb);
  }
};

// Specifies x-x-x(8/16/32) bit multi planar RGB/BGR variants
template <VideoFormat T>
struct VideoFormatSize<T, std::enable_if_t<T == VideoFormat::GXF_VIDEO_FORMAT_R8_G8_B8 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_B8_G8_R8 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_R16_G16_B16 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_B16_G16_R16 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_R32_G32_B32 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_B32_G32_R32>> {
  std::array<ColorPlane, 3> defaultRGB() {
    if (T == VideoFormat::GXF_VIDEO_FORMAT_R8_G8_B8) {
      return {ColorPlane("R", 1), ColorPlane("G", 1), ColorPlane("B", 1)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_B8_G8_R8) {
      return {ColorPlane("B", 1), ColorPlane("G", 1), ColorPlane("R", 1)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_R16_G16_B16) {
      return {ColorPlane("R", 2), ColorPlane("G", 2), ColorPlane("B", 2)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_B16_G16_R16) {
      return {ColorPlane("B", 2), ColorPlane("G", 2), ColorPlane("R", 2)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_R32_G32_B32) {
      return {ColorPlane("R", 4), ColorPlane("G", 4), ColorPlane("B", 4)};
    } else {  // T == VideoFormat::GXF_VIDEO_FORMAT_B32_G32_R32
      return {ColorPlane("B", 4), ColorPlane("G", 4), ColorPlane("R", 4)};
    }
  }

  uint64_t fillColorPlanes(uint32_t width, uint32_t height,
                           std::array<ColorPlane, 3>& color_planes) {
    uint32_t widthEven = AlignToEvenDimension(width);
    uint32_t heightEven = AlignToEvenDimension(height);

    uint64_t size = 0;
    for (size_t i = 0; i < color_planes.size(); ++i) {
      color_planes[i].width = widthEven;
      color_planes[i].height = heightEven;
      color_planes[i].stride =
          color_planes[i].stride == -1
              ? StrideAlign(color_planes[i].width * color_planes[i].bytes_per_pixel)
              : color_planes[i].stride;
      color_planes[i].size = color_planes[i].stride * color_planes[i].height;
      color_planes[i].offset = size;
      size += color_planes[i].size;
    }
    return size;
  }

  std::vector<ColorPlane> getDefaultColorPlanes(uint32_t width, uint32_t height) {
    auto rgb = defaultRGB();
    fillColorPlanes(width, height, rgb);
    std::vector<ColorPlane> result(rgb.begin(), rgb.end());
    return result;
  }

  uint64_t size(uint32_t width, uint32_t height, std::array<ColorPlane, 3>& color_planes) {
    return fillColorPlanes(width, height, color_planes);
  }

  uint64_t size(uint32_t width, uint32_t height) {
    auto rgb = defaultRGB();
    return fillColorPlanes(width, height, rgb);
  }
};

// Specifies x-x-x(8/16/32) bit single plane GRAY scale
template <VideoFormat T>
struct VideoFormatSize<T, std::enable_if_t<T == VideoFormat::GXF_VIDEO_FORMAT_GRAY ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_GRAY16 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_GRAY32 ||
                                           T == VideoFormat::GXF_VIDEO_FORMAT_GRAY32F>> {
  std::array<ColorPlane, 1> defaultGray() {
    if (T == VideoFormat::GXF_VIDEO_FORMAT_GRAY) {
      return {ColorPlane("gray", 1)};
    } else if (T == VideoFormat::GXF_VIDEO_FORMAT_GRAY16) {
      return {ColorPlane("gray", 2)};
    } else {  // T == VideoFormat::GXF_VIDEO_FORMAT_GRAY32 & VideoFormat::GXF_VIDEO_FORMAT_GRAY32F
      return {ColorPlane("gray", 4)};
    }
  }

  uint64_t fillColorPlanes(uint32_t width, uint32_t height,
                           std::array<ColorPlane, 1>& color_plane) {
    uint32_t widthEven = AlignToEvenDimension(width);
    uint32_t heightEven = AlignToEvenDimension(height);
    color_plane[0].width = widthEven;
    color_plane[0].height = heightEven;
    color_plane[0].stride = color_plane[0].stride == -1
                                ? StrideAlign(color_plane[0].width * color_plane[0].bytes_per_pixel)
                                : color_plane[0].stride;
    color_plane[0].size = color_plane[0].stride * color_plane[0].height;
    color_plane[0].offset = 0;
    return color_plane[0].size;
  }

  std::vector<ColorPlane> getDefaultColorPlanes(uint32_t width, uint32_t height) {
    auto gray = defaultGray();
    fillColorPlanes(width, height, gray);
    std::vector<ColorPlane> result(gray.begin(), gray.end());
    return result;
  }

  uint64_t size(uint32_t width, uint32_t height, std::array<ColorPlane, 1>& color_plane) {
    return fillColorPlanes(width, height, color_plane);
  }

  uint64_t size(uint32_t width, uint32_t height) {
    auto gray = defaultGray();
    return fillColorPlanes(width, height, gray);
  }
};

// Descriptor for a VideoBuffer
struct VideoBufferInfo {
  // width of a video frame
  uint32_t width;
  // height of a video frame
  uint32_t height;
  // color format of a video frame
  VideoFormat color_format;
  // Color plane info
  std::vector<ColorPlane> color_planes;
  // surface memory layout of a video frame
  SurfaceLayout surface_layout;
};

// A media type which stores information corresponding to a video frame
// resize(...) function is used to allocate memory for the video frame based on
// height, width and color format of the frame
class VideoBuffer {
 public:
  VideoBuffer() = default;

  ~VideoBuffer() { memory_buffer_.freeBuffer(); }

  VideoBuffer(const VideoBuffer&) = delete;

  VideoBuffer(VideoBuffer&& other) { *this = std::move(other); }

  VideoBuffer& operator=(const VideoBuffer&) = delete;

  VideoBuffer& operator=(VideoBuffer&& other) {
    buffer_info_ = other.buffer_info_;
    memory_buffer_ = std::move(other.memory_buffer_);

    return *this;
  }

  // Resizes the video frame and allocates the corresponding memory with the allocator provided
  // Any data previously stored in the frame would be freed
  template <VideoFormat C>
  Expected<void> resize(uint32_t width, uint32_t height, SurfaceLayout layout,
                        MemoryStorageType storage_type, Handle<Allocator> allocator) {
    VideoTypeTraits<C> video_type;
    VideoFormatSize<C> color_format;
    uint64_t size = color_format.size(width, height);
    auto color_planes = color_format.getDefaultColorPlanes(width, height);
    VideoBufferInfo buffer_info{width, height, video_type.value, color_planes, layout};
    return resizeCustom(buffer_info, size, storage_type, allocator);
  }

  // Type of the callback function to release memory passed to the VideoFrame using the
  // wrapMemory method
  using release_function_t = MemoryBuffer::release_function_t;

  // Wrap existing memory inside the VideoBuffer. A callback function of type release_function_t
  // may be passed that will be called when the VideoBuffer wants to release the memory.
  Expected<void> wrapMemory(VideoBufferInfo buffer_info, uint64_t size,
                            MemoryStorageType storage_type, void* pointer,
                            release_function_t release_func);

  // Moves the existing video buffer to a tensor element specified by the handle
  Expected<void> moveToTensor(Handle<Tensor>& tensor);


  // Moves memory buffer from tensor to video buffer works on rank 2 and 3 tensors,
  // ordering of dimensions in tensor is assumed to be [whc]
  // OBS: If successful, tensor destructor will be called and handle set to null
  template <VideoFormat C>
  Expected<void> createFromTensor(Handle<Tensor>& tensor, SurfaceLayout layout);

  // VideoBufferInfo of the video frame
  VideoBufferInfo video_frame_info() const { return buffer_info_; }

  // The type of memory where the frame data is stored.
  MemoryStorageType storage_type() const { return memory_buffer_.storage_type(); }

  // Size of the video frame in bytes
  uint64_t size() const { return memory_buffer_.size(); }

  // Raw pointer to the first byte of the video frame
  byte* pointer() const { return memory_buffer_.pointer(); }

  // Resizes the video frame and allocates the corresponding memory with the allocator provided
  // Any data previously stored in the frame would be freed
  Expected<void> resizeCustom(VideoBufferInfo buffer_info, uint64_t size,
                              MemoryStorageType storage_type, Handle<Allocator> allocator);

  // Helper function to get primitive types for color formats that are valid (planar) for moving
  // from  Tensor to VideoBuffer, and vice versa. If color_format is not valid,
  // PrimitiveType::kCustom is returned
  static Expected<PrimitiveType> getPlanarPrimitiveType(VideoFormat color_format) {
    PrimitiveType primitive_type;
    switch (color_format) {
        case   VideoFormat::GXF_VIDEO_FORMAT_RGBA:          // RGBA-8-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_BGRA:          // BGRA-8-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_ARGB:          // ARGB-8-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_ABGR:          // ABGR-8-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_RGBX:          // RGBX-8-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_BGRX:          // BGRX-8-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_XRGB:          // XRGB-8-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_XBGR:          // XBGR-8-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_RGB:           // RGB-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_BGR:           // BGR-8-8-8 single plane
        case   VideoFormat::GXF_VIDEO_FORMAT_R8_G8_B8:      // RGB - unsigned 8 bit multiplanar
        case   VideoFormat::GXF_VIDEO_FORMAT_B8_G8_R8:      // BGR - unsigned 8 bit multiplanar
        case   VideoFormat::GXF_VIDEO_FORMAT_GRAY: {        // 8 bit GRAY scale single plane
          primitive_type = PrimitiveType::kUnsigned8;
        } break;

        case  VideoFormat::GXF_VIDEO_FORMAT_GRAY16:         // 16 bit GRAY scale single plane
        case  VideoFormat::GXF_VIDEO_FORMAT_RGB16:          // RGB-16-16-16 single plane
        case  VideoFormat::GXF_VIDEO_FORMAT_BGR16:          // BGR-16-16-16 single plane
        case  VideoFormat::GXF_VIDEO_FORMAT_R16_G16_B16:    // RGB - signed 16 bit multiplanar
        case  VideoFormat::GXF_VIDEO_FORMAT_B16_G16_R16: {  // BGR - signed 16 bit multiplanar
          primitive_type = PrimitiveType::kUnsigned16;
        } break;

        case VideoFormat::GXF_VIDEO_FORMAT_GRAY32:          // 32 bit GRAY scale single plane
        case VideoFormat::GXF_VIDEO_FORMAT_RGB32:           // RGB-32-32-32 single plane
        case VideoFormat::GXF_VIDEO_FORMAT_BGR32:           // BGR-32-32-32 single plane
        case VideoFormat::GXF_VIDEO_FORMAT_R32_G32_B32:     // RGB - signed 32 bit multiplanar
        case VideoFormat::GXF_VIDEO_FORMAT_B32_G32_R32: {   // BGR - signed 32 bit multiplanar
          primitive_type = PrimitiveType::kUnsigned32;
        } break;

        case VideoFormat::GXF_VIDEO_FORMAT_GRAY32F: {       // 32 bit GRAY scale single plane
          primitive_type = PrimitiveType::kFloat32;
        } break;
        default: {                                          // Non-planar type given
          GXF_LOG_ERROR("VideoFormat is of non-planar color format (%d),"
                        " which cannot be moved from tensor", color_format);
          return Unexpected{GXF_INVALID_DATA_FORMAT};
        } break;
    }

    return primitive_type;
  }

 private:
  VideoBufferInfo buffer_info_;
  MemoryBuffer memory_buffer_;
};

template <VideoFormat C>
Expected<void> VideoBuffer::createFromTensor(Handle<Tensor>& tensor,
                                             SurfaceLayout layout) {
  if (!tensor) {
    GXF_LOG_ERROR("createFromTensor received invalid tensor handle");
    return Unexpected{GXF_ARGUMENT_NULL};
  }

  // Tensor must have rank 2 (single-plane image) or 3 (multi-plane image)
  const uint32_t rank = tensor->rank();
  if ((rank < 2) || (rank > 3)) {
      GXF_LOG_ERROR("Tensor cannot be moved to VideoBuffer."
                    " Invalid rank=[%d], should be 2 or 3", rank);
      return Unexpected{GXF_INVALID_DATA_FORMAT};
  }

  // Get primitive type and sanity check color format
  VideoTypeTraits<C> video_type;
  VideoFormatSize<C> color_format;
  Expected<PrimitiveType> primitive_type = getPlanarPrimitiveType(video_type.value);
  if (!primitive_type) { return ForwardError(primitive_type); }

  // Ensure video buffer and tensor has same data type
  PrimitiveType primite_type_tensor = tensor->element_type();

  if (primitive_type.value() != primite_type_tensor) {
      GXF_LOG_ERROR("Type of video buffer (%d) is different from"
                    " type of tensor (%d)", primitive_type.value(), primite_type_tensor);
      return Unexpected{GXF_INVALID_DATA_FORMAT};
  }

  // Get tensor dimensions
  auto width = static_cast<uint32_t>(tensor->shape().dimension(0));
  auto height = static_cast<uint32_t>(tensor->shape().dimension(1));
  auto channels = static_cast<uint32_t>(tensor->shape().dimension(2));

  // Get color planes
  auto color_planes = color_format.getDefaultColorPlanes(width, height);

  // Sanity check that number of tensor channels corresponds to video format
  if (channels != color_planes.size()) {
      GXF_LOG_ERROR("Number of channels in tensor (%d) is "
                    " different from video buffer (%d)", channels, color_planes.size());
      return Unexpected{GXF_INVALID_DATA_FORMAT};
  }

  // Set buffer info
  VideoBufferInfo buffer_info{width, height, video_type.value, color_planes, layout};
  buffer_info_ = buffer_info;

  // Move memory buffer
  memory_buffer_ = tensor->move_buffer();

  // Explicit call to destructor as tensor is no longer usable (also set to Null handle)
  tensor->~Tensor();
  tensor = Handle<Tensor>::Null();

  return Success;
}

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_MULTIMEDIA_VIDEO_HPP_
