// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/serialization/base64.hpp"
#include "engine/gems/serialization/json.hpp"

namespace nvidia {
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
}  // namespace nvidia
