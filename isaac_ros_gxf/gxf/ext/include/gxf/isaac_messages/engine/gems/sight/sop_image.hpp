/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/serialization/base64.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace sight {

// Sight Operation Image.
// Helper to serialize an image in JPG (default) or PNG format
class SopImage {
 public:
  // Delete copy constructor
  SopImage(const SopImage&) = delete;
  SopImage(SopImage&&) = default;

  // Create an image in png format.
  template <typename Image>
  static SopImage Png(const Image& img) {
    SopImage soi;
    std::vector<uint8_t> png;
    if (!img.empty()) {
      EncodePng(img, png);
    }
    soi.json_["t"] = "img";
    soi.json_["data"] =
        "data:image/png;base64," + serialization::Base64Encode(png.data(), png.size());
    return soi;
  }

  // Create an image in jpg format.
  template <typename Image>
  static SopImage Jpg(const Image& img) {
    SopImage soi;
    std::vector<uint8_t> jpg;
    if (!img.empty()) {
      EncodeJpeg(img, 75, jpg);
    }
    soi.json_["t"] = "img";
    soi.json_["data"] =
        "data:image/jpg;base64," + serialization::Base64Encode(jpg.data(), jpg.size());
    return soi;
  }

 private:
  friend const Json& ToJson(const SopImage&);
  friend Json ToJson(SopImage&&);

  // Private to allow construction from the static function
  SopImage() = default;

  Json json_;
};

// Returns the json of a SopImage
const Json& ToJson(const SopImage&);
Json ToJson(SopImage&&);

}  // namespace sight
}  // namespace isaac
