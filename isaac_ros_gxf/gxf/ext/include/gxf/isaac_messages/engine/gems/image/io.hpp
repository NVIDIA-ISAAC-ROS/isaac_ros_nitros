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

#include <string>
#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/algorithm/string_utils.hpp"

namespace nvidia {
namespace isaac {

// Gets the dimensions of an image. The dimensions are written to `shape` in the order:
// rows (0), cols (1), channels (2).
bool LoadImageShape(const std::string& filename, Vector3i& shape);

// Loads an image from a PNG file. The function will return false if the PNG could not be loaded.
// This can for example happen if the filename is invalid or if the PNG doesn't have the requested
// format.
bool LoadPng(const std::string& filename, Image1ub& image);
bool LoadPng(const std::string& filename, Image1ui16& image);
bool LoadPng(const std::string& filename, Image3ub& image);
bool LoadPng(const std::string& filename, Image4ub& image);
bool LoadPng(const std::string& filename, ImageView1ub& tensor);
bool LoadPng(const std::string& filename, ImageView3ub& tensor);
bool LoadPng(const std::string& filename, CpuTensorView2ub& tensor);

// Loads an image from a JPEG file. The function will return false if the JPEG could not be loaded.
// This can for example happen if the filename is invalid or if the JPEG doesn't have the requested
// format.
bool LoadJpeg(const std::string& filename, Image1ub& image);
bool LoadJpeg(const std::string& filename, Image3ub& image);
bool LoadJpeg(const std::string& filename, ImageView1ub& image);
bool LoadJpeg(const std::string& filename, ImageView3ub& image);

// Loads an image from JPEG file or PNG file. The function will return false if the image could not
// be loaded.
bool LoadImage(const std::string& filename, Image1ub& image);
bool LoadImage(const std::string& filename, Image3ub& image);
bool LoadImage(const std::string& filename, ImageView1ub& image);
bool LoadImage(const std::string& filename, ImageView3ub& image);

// Saves an image to a file in the PNG format. The function will return false if the PNG could not
// saved. This might for example happen if the file could not be opened.
bool SavePng(const ImageConstView1ub& image, const std::string& filename);
bool SavePng(const ImageConstView3ub& image, const std::string& filename);
bool SavePng(const ImageConstView4ub& image, const std::string& filename);
bool SavePng(const ImageConstView1ui16& image, const std::string& filename);

// Saves an image to a file in the JPEG format. The function will return false if the JPEG could not
// saved. This might for example happen, if the file could not be opened. Quality of 100 means
// nearly lossless, but slow to encode and result in bigger images. While quality of 1 is fast
// and produce very small image with a very low quality. (Note: 75 seems a good compromise)
bool SaveJpeg(const ImageConstView1ub& image, const std::string& filename, const int quality = 75);
bool SaveJpeg(const ImageConstView3ub& image, const std::string& filename, const int quality = 75);

// Encodes an image into a buffer in the PNG format
void EncodePng(const ImageConstView1ub& image, std::vector<uint8_t>& encoded);
void EncodePng(const ImageConstView3ub& image, std::vector<uint8_t>& encoded);
void EncodePng(const ImageConstView4ub& image, std::vector<uint8_t>& encoded);

// Encodes an image into a file in the JPEG format
// Quality of 100 is nearly lossless but slow to encode and result in bigger images while 1 is fast
// and produce very small image with a very low quality. (Note: 75 seems a good compromise)
void EncodeJpeg(const ImageConstView1ub& image, int quality, std::vector<uint8_t>& encoded);
void EncodeJpeg(const ImageConstView3ub& image, int quality, std::vector<uint8_t>& encoded);

// Decodes a PNG encoded byte string. The function will return false if the PNG could not be
// decoded, for example if the byte string is not a valid PNG.
bool DecodePng(const std::string& bytes, Image1ub& image);
bool DecodePng(const std::string& bytes, Image1ui16& image);
bool DecodePng(const std::string& bytes, Image3ub& image);
bool DecodePng(const std::string& bytes, Image4ub& image);
bool DecodePng(const std::string& bytes, ImageView1ub& tensor);
bool DecodePng(const std::string& bytes, ImageView3ub& tensor);
bool DecodePng(const std::string& bytes, CpuTensorView2ub& tensor);

// Gets the dimensions of an image from a PNG encoded byte string. The dimensions are written to
// `shape` in the order: rows (0), cols (1), channels (2).
bool DecodePngShape(const std::string& bytes, Vector3i& shape);

}  // namespace isaac
}  // namespace nvidia
