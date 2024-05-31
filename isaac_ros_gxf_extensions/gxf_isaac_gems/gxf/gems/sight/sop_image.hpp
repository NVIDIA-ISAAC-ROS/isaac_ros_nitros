// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <memory>
#include <utility>
#include <vector>

#include "gems/core/image/image.hpp"
#include "gems/core/math/types.hpp"
#include "gems/image/io.hpp"
#include "gems/sight/sop_serializer.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight Operation Image.
// Helper to serialize an image in JPG (default) or PNG format
class SopImage : public SopSerializer {
 public:
  // Creates empty image
  SopImage() = default;
  SopImage(SopImage&&) = default;
  ~SopImage() override = default;

  // Create an image in png or jpg format and whether or not you flip it.
  template <typename Image>
  SopImage(const Image& img, bool png, bool flip) {
    if (!img.empty()) {
      flip_ = flip;
      if (png) {
        ::nvidia::isaac::EncodePng(img, image_);
      } else {
        ::nvidia::isaac::EncodeJpeg(img, /* quality = */ 50, image_);
      }
    }
  }

  // Create an image in png or jpg format.
  template <typename Image>
  SopImage(const Image& img, bool png) {
    if (!img.empty()) {
      flip_ = false;
      if (png) {
        ::nvidia::isaac::EncodePng(img, image_);
      } else {
        ::nvidia::isaac::EncodeJpeg(img, /* quality = */ 50, image_);
      }
    }
  }

  // Create an image in jpg format with a custom quality.
  template <typename Image>
  static SopImage Jpg(const Image& img, int quality) {
    SopImage sop;
    if (!img.empty()) {
      sop.flip_ = false;
      ::nvidia::isaac::EncodeJpeg(img, quality, sop.image_);
    }
    return sop;
  }

  // Create an image in jpg format with a custom quality  and whether or not you flip it.
  template <typename Image>
  static SopImage Jpg(const Image& img, int quality, bool flip) {
    SopImage sop;
    if (!img.empty()) {
      sop.flip_ = flip;
      ::nvidia::isaac::EncodeJpeg(img, quality, sop.image_);
    }
    return sop;
  }

  // Create an image in jpg format.
  template <typename Image>
  SopImage(const Image& img) {
    if (!img.empty()) {
      flip_ = false;
      ::nvidia::isaac::EncodeJpeg(img, 50, image_);
    }
  }

  // Construct a sop by calling fromBinary
  SopImage(BufferSerialization& buffer);

  bool toBinary(BufferSerialization& buffer) const override;
  bool fromBinary(BufferSerialization& buffer) override;

 private:
  bool flip_;
  std::vector<uint8_t> image_;
};

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
